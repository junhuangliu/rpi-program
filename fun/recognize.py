#!/usr/bin/env python3
"""农田图两阶段识别节点。

功能：
  1. 提供 ROS2 服务 /recognize/run（std_srvs/Trigger）：读取指定图片（默认项目根
     maixcam_latest.jpg，即 MaixCAM 最近一张），两阶段识别：
     阶段一 检测模型（YOLO ONNX）：定位图中「农田图」区域（普通边界框，忽略背景）
     阶段二 分类模型（ONNX）：对裁剪出的干净农田图分类 → 返回 labels.txt 中对应类别
  2. 返回约定：success=True, message=类别（labels.txt 内容，当前 1/2/3）；
     检测/分类置信度不足 → success=True, message="0"；
     读图失败 → "ERR_IMG"；模型缺失 → "ERR_MODEL"（起点器按 message 判断即可）
  3. 模型在首次服务调用时懒加载；节点在模型缺失/onnxruntime 未装时也能正常启停

参数：
  --det-model  阶段一检测模型 ONNX（默认 models/detect/model.onnx）
  --cls-model  阶段二分类模型 ONNX（默认 models/classify/model.onnx）
  --labels     类别文件，一行一类、顺序=模型输出索引（默认 models/classify/labels.txt）
  --det-size   检测输入边长（默认 640）
  --cls-size   分类输入边长（默认 224）
  --det-conf   检测置信度阈值（默认 0.5）
  --cls-conf   分类置信度阈值（默认 0.5）
  --iou        NMS IoU 阈值（默认 0.45）
  --img        待识别图片路径（默认项目根 maixcam_latest.jpg）
  --vis        检测标注图输出路径（默认项目根 recognize_debug.jpg，每次覆盖；传 "" 关闭）

用法：
  python3 start.py recognize [参数...]
  ros2 service call /recognize/run std_srvs/srv/Trigger "{}"
"""

import argparse
import os
import sys

try:
    import numpy as np
    import cv2
except ModuleNotFoundError as e:
    sys.exit(f"[错误] 缺少依赖 numpy/opencv: {e}")

try:
    import onnxruntime as ort
    _ORT_ERR = None
except ModuleNotFoundError as e:
    _ORT_ERR = e

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e

SERVICE_NAME = "recognize/run"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class RecognizeNode(Node):
    """两阶段识别节点：检测定位「农田图」→ 裁剪 → 分类。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("recognize")
        self._det_model = args.det_model
        self._cls_model = args.cls_model
        self._labels_file = args.labels
        self._det_size = args.det_size
        self._cls_size = args.cls_size
        self._det_conf = args.det_conf
        self._cls_conf = args.cls_conf
        self._iou = args.iou
        self._img = args.img
        self._vis = args.vis or None

        self._sess_det = None
        self._sess_cls = None
        self._labels = None       # 加载失败用索引+1 兜底
        self._load_labels()
        self._srv = self.create_service(Trigger, SERVICE_NAME, self.handle_recognize)
        self.get_logger().info(
            f"两阶段识别服务: /{SERVICE_NAME}  (检测: {os.path.basename(self._det_model)}, "
            f"分类: {os.path.basename(self._cls_model)}, 类别: {self._labels or '<索引+1>'})")

    def _load_labels(self) -> None:
        try:
            with open(self._labels_file, encoding="utf-8") as f:
                labels = [ln.strip() for ln in f if ln.strip()]
            if labels:
                self._labels = labels
                return
        except OSError:
            pass
        self.get_logger().warn(f"类别文件不可用: {self._labels_file}，将用索引+1 兜底")

    def _ensure_session(self) -> str | None:
        """懒加载两个 ONNX 会话；返回错误信息或 None。"""
        if _ORT_ERR is not None:
            return f"onnxruntime 未安装: {_ORT_ERR}"
        if self._sess_det is None:
            if not os.path.isfile(self._det_model):
                return f"检测模型缺失: {self._det_model}"
            self._sess_det = ort.InferenceSession(
                self._det_model, providers=["CPUExecutionProvider"])
        if self._sess_cls is None:
            if not os.path.isfile(self._cls_model):
                return f"分类模型缺失: {self._cls_model}"
            self._sess_cls = ort.InferenceSession(
                self._cls_model, providers=["CPUExecutionProvider"])
        return None

    def handle_recognize(self, request, response):
        """服务回调：读图 → 检测 → 裁剪 → 分类 → message=类别（失败/低置信为 0）。"""
        img = cv2.imread(self._img)
        if img is None:
            self.get_logger().warn(f"读图失败: {self._img}")
            response.success = False
            response.message = "ERR_IMG"
            return response
        self.get_logger().info(f"开始识别: {self._img} ({img.shape[1]}x{img.shape[0]})")

        err = self._ensure_session()
        if err:
            self.get_logger().warn(err)
            response.success = False
            response.message = "ERR_MODEL"
            return response

        box = self._detect(img)
        if box is None:
            self.get_logger().warn("未检测到「农田图」目标")
            response.success = True
            response.message = "0"
            return response

        label, conf = self._classify(self._crop(img, box))
        self.get_logger().info(f"检测框={box}, 分类={label} (conf={conf:.3f})")
        self._save_vis(img, box, label, conf)
        response.success = True
        response.message = label if conf >= self._cls_conf else "0"
        return response

    def _save_vis(self, img, box, label, conf) -> None:
        """检测位置标注图：画边界框 + 类别/置信度文本，存 self._vis（无则不存）。"""
        if not self._vis:
            return
        x1, y1, x2, y2, _ = box
        vis = img.copy()
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{label} {conf:.2f}"
        tx, ty = x1, max(y1 - 8, 0) + 2
        cv2.putText(vis, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(self._vis, vis)
        self.get_logger().info(f"标注图已保存: {self._vis}")

    # ---------------- 阶段一：目标检测（YOLOv8 ONNX） ----------------
    def _detect(self, img) -> tuple | None:
        """YOLO ONNX 推理 + NMS，返回原图坐标系最大值框 (x1,y1,x2,y2,conf) 或 None。"""
        canvas, scale, dw, dh = self._letterbox(img, self._det_size)
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (self._det_size, self._det_size),
                                     (0, 0, 0), swapRB=False)
        out = self._sess_det.run(None, {self._sess_det.get_inputs()[0].name: blob})[0][0]
        if out.shape[0] <= out.shape[1]:          # [features, N] → [N, features]
            out = out.T

        boxes, scores = [], []
        for row in out:
            obj_scores = row[4:]
            idx = int(obj_scores.argmax())
            conf = float(obj_scores[idx])
            if conf < self._det_conf:
                continue
            cx, cy, w, h = row[:4]
            boxes.append([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])
            scores.append(conf)
        if not boxes:
            return None

        idxs = self._nms(np.array(boxes), np.array(scores), self._iou)
        best = max((i for i in idxs), key=lambda i: scores[i])
        x1, y1, x2, y2 = boxes[best]
        h_img, w_img = img.shape[:2]
        box = (int((x1 - dw) / scale), int((y1 - dh) / scale),
               int((x2 - dw) / scale), int((y2 - dh) / scale))
        box = self._clip(box, w_img, h_img)
        return (*box, scores[best])

    # ---------------- 阶段二：图像分类（ONNX） ----------------
    def _classify(self, crop) -> tuple[str, float]:
        """裁剪图归一化后推理，返回 (类别, 置信度)。"""
        resized = cv2.resize(crop, (self._cls_size, self._cls_size))
        blob = cv2.dnn.blobFromImage(resized, 1 / 255.0, (self._cls_size, self._cls_size),
                                     (0, 0, 0), swapRB=False)
        logits = self._sess_cls.run(None, {self._sess_cls.get_inputs()[0].name: blob})[0][0]
        exp = np.exp(logits - logits.max())
        probs = exp / exp.sum()
        idx = int(probs.argmax())
        conf = float(probs[idx])
        label = self._labels[idx] if self._labels and idx < len(self._labels) else str(idx + 1)
        return label, conf

    # ---------------- 工具 ----------------
    @staticmethod
    def _letterbox(img, size):
        h, w = img.shape[:2]
        scale = size / max(h, w)
        new_w, new_h = max(1, round(w * scale)), max(1, round(h * scale))
        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.full((size, size, 3), 114, np.uint8)
        dw, dh = (size - new_w) // 2, (size - new_h) // 2
        canvas[dh:dh + new_h, dw:dw + new_w] = resized
        return canvas, scale, dw, dh

    @staticmethod
    def _clip(box, w, h):
        x1, y1, x2, y2 = box
        return (max(0, x1), max(0, y1), min(w - 1, x2), min(h - 1, y2))

    @staticmethod
    def _crop(img, box):
        x1, y1, x2, y2 = box[:4]
        return img[y1:y2 + 1, x1:x2 + 1]

    @staticmethod
    def _iou(a, b):
        x1 = np.maximum(a[0], b[:, 0])
        y1 = np.maximum(a[1], b[:, 1])
        x2 = np.minimum(a[2], b[:, 2])
        y2 = np.minimum(a[3], b[:, 3])
        inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
        return inter / (area_a + area_b - inter + 1e-6)

    @classmethod
    def _nms(cls, boxes, scores, iou_thresh):
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            if order.size == 1:
                break
            rest = order[1:]
            order = rest[cls._iou(boxes[i], boxes[rest]) <= iou_thresh]
        return keep


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="农田图两阶段识别节点")
    parser.add_argument("--det-model", default=os.path.join(ROOT, "models/detect/model.onnx"))
    parser.add_argument("--cls-model", default=os.path.join(ROOT, "models/classify/model.onnx"))
    parser.add_argument("--labels", default=os.path.join(ROOT, "models/classify/labels.txt"))
    parser.add_argument("--det-size", type=int, default=640)
    parser.add_argument("--cls-size", type=int, default=224)
    parser.add_argument("--det-conf", type=float, default=0.5)
    parser.add_argument("--cls-conf", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--img", default=os.path.join(ROOT, "maixcam_latest.jpg"))
    parser.add_argument("--vis", default=os.path.join(ROOT, "recognize_debug.jpg"))
    args = parser.parse_known_args()[0]

    rclpy.init()
    node = RecognizeNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()