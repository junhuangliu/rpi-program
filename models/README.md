# 模型目录（两阶段识别）

识别管线：`REC → 拍照(snap) → 检测定位「农田图」→ 裁剪 → 分类 → 回传类别`

## 目录结构

```
models/
  detect/                阶段一：目标检测（YOLOv8 导出 ONNX）
    model.onnx           训练好的检测权重放这（导出命令见下）
  classify/              阶段二：农田分类（任何 CNN 导出 ONNX）
    model.onnx           分类权重放这
    labels.txt           类别名，一行一个，顺序 = 模型输出索引
  README.md              本文件
```

## 权重放置

训练在 PC 上进行，训练好后导出 ONNX 再拷贝到树莓派：

- 检测（YOLOv8，ultralytics）：
  `yolo export model=best.pt format=onnx` → 把生成 `best.onnx` 重命名放 `models/detect/model.onnx`
- 分类：PyTorch/TF 训练好导出 ONNX（输入需为 NCHW float32，尺寸自定），放 `models/classify/model.onnx`
  输入尺寸需与启动参数 `--cls-size` 一致（默认 224）

类别文件 `models/classify/labels.txt`：一行一类，顺序与模型输出索引一致；当前为 `1/2/3`，核对类别后修改。

## 运行

```
./control.sh start recognize        # 启动识别节点（默认读 models/ 下权重）
./control.sh rc                     # 调 /recognize/run 打印识别结果
```

参数可用 `--det-model / --cls-model / --labels / --det-size / --cls-size / --det-conf / --cls-conf / --img` 覆盖。

## 依赖

```
pip install onnxruntime
```
（检测+分类统一 ONNX 推理；权重就位前节点可正常启停，识别返回 ERR_MODEL 属预期）