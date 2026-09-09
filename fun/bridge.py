#!/usr/bin/env python3
"""上位机串口桥接节点。

功能：
  1. 10Hz 下行：订阅 TF map->base_link，组坐标帧（x,y,yaw）发送给上位机
  2. 上行监听：解析上位机命令帧，查命令表并调用对应 ROS2 服务，回传结果帧
  3. 已接服务：TASK1 -> /camera/command（OpenMV 视觉任务）；QR -> /scanner/query（最近条码）

用法：
  python3 start.py bridge -p/--port <串口> [-b/--baudrate 115200]

  默认帧协议（占位，待上位机侧确认后调整 FRAME_HEAD/FRAME_TAIL/FIELD_SEP）：
    下行坐标:  #POS,x,y,yaw   （无 SLAM 时 #POS,no_tf）
    上行命令:  #<命令>[,参数...]     例: #TASK1 / #QR
    回传结果:  #RES,<内容>
    行尾:      \r\n
  命令表以 self.commands 为准（待用户提供完整命令表后扩充）。

说明：
  - 上位机为 USB 虚拟串口；端口需用 -p 指定（插入后若需自动识别，把其 VID/PID
    加入 serial_ports.py）
  - 节点使用 MultiThreadedExecutor：坐标定时器与命令处理互不阻塞（服务调用最长等待耗时不阻挡 10Hz 坐标发送）
"""

import argparse
import math
import os
import sys
import threading
import time

import serial

from fun import serial_ports

# ================= 帧协议常量（占位，待上位机确认后修改） =================
FRAME_HEAD = "#"          # 帧头
FRAME_TAIL = "$"          # 帧尾
FIELD_SEP = ","           # 字段分隔符
LINE_END = b"\r\n"        # 行尾
# ===========================================================================
BAUDRATE = 115200
POS_RATE_HZ = 10.0        # 坐标发送频率
STATUS_NO_TF = "no_tf"    # 无 SLAM TF 时的占位状态
CAMERA_TIMEOUT = 20.0
SCANNER_TIMEOUT = 10.0

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from tf2_ros import Buffer, TransformListener
    from openmv_msgs.srv import Command
    from std_srvs.srv import Trigger

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e


def quat_to_yaw(q) -> float:
    """四元数转偏航角（弧度）。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class BridgeNode(Node):
    """上位机桥接节点：10Hz 坐标下行 + 命令上行转发服务。"""

    def __init__(self, port: str, baudrate: int) -> None:
        super().__init__("host_bridge")
        self.port = port
        self.ser = serial.Serial(port, baudrate, timeout=0.2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cli_camera = self.create_client(Command, "camera/command")
        self.cli_scanner = self.create_client(Trigger, "scanner/query")
        self.commands = {
            "TASK1": self._cmd_camera,
            "QR": self._cmd_scanner,
        }
        self.timer = self.create_timer(1.0 / POS_RATE_HZ, self.on_pos_timer)
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self.read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f"串口已打开: {port} @ {baudrate}, 坐标 {POS_RATE_HZ:g}Hz, "
            f"命令表: {list(self.commands)}")

    # ---------------- 下行：周期坐标 ----------------
    def on_pos_timer(self) -> None:
        """10Hz：读取 map->base_link TF 并发送坐标帧。"""
        try:
            f = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            t, q = f.transform.translation, f.transform.rotation
            payload = f"POS{FIELD_SEP}{t.x:.3f}{FIELD_SEP}{t.y:.3f}{FIELD_SEP}{quat_to_yaw(q):.3f}"
        except Exception:
            payload = f"POS{FIELD_SEP}{STATUS_NO_TF}"
        self.ser.write(self._frame(payload))
        self.ser.flush()

    # ---------------- 上行：命令处理 ----------------
    def read_loop(self) -> None:
        """监听串口，解析命令帧并按命令表调用服务后回传结果。"""
        while rclpy.ok() and not self._stop.is_set():
            line = self.ser.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            fields = self._parse_frame(text)
            if not fields:
                continue
            cmd = fields[0]
            handler = self.commands.get(cmd)
            if handler is None:
                self._reply(f"UNKNOWN:{cmd}")
                continue
            try:
                result = handler(fields)
            except Exception as e:
                result = f"ERROR:{e}"
            self._reply(result)

    def _cmd_camera(self, fields: list[str]) -> str:
        """调用 /camera/command 执行 OpenMV 任务，返回最终回显。"""
        sub = fields[1] if len(fields) > 1 else "TASK1"
        if not self.cli_camera.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("camera/command 服务不可用")
        req = Command.Request()
        req.command = sub
        fut = self.cli_camera.call_async(req)
        resp = self._wait_future(fut, CAMERA_TIMEOUT)
        return resp.result

    def _cmd_scanner(self, fields: list[str]) -> str:
        """调用 /scanner/query 获取最近条码。"""
        if not self.cli_scanner.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("scanner/query 服务不可用")
        fut = self.cli_scanner.call_async(Trigger.Request())
        resp = self._wait_future(fut, SCANNER_TIMEOUT)
        return resp.message

    @staticmethod
    def _wait_future(fut, timeout: float):
        """轮询等待 rclpy future 完成，超时抛 RuntimeError。"""
        start = time.time()
        while rclpy.ok() and not fut.done() and time.time() - start < timeout:
            time.sleep(0.05)
        if not fut.done():
            raise RuntimeError(f"服务响应超时({timeout:g}s)")
        return fut.result()

    # ---------------- 帧工具 ----------------
    @staticmethod
    def _parse_frame(text: str) -> list[str]:
        """去掉帧头帧尾，按字段分隔符拆分。"""
        if text.startswith(FRAME_HEAD):
            text = text[len(FRAME_HEAD):]
        if text.endswith(FRAME_TAIL):
            text = text[:-len(FRAME_TAIL)]
        return [s for s in text.split(FIELD_SEP) if s]

    @staticmethod
    def _frame(payload: str) -> bytes:
        """组帧并加行尾。"""
        return f"{FRAME_HEAD}{payload}{FRAME_TAIL}".encode() + LINE_END

    def _reply(self, content: str) -> None:
        """回传结果帧。"""
        self.ser.write(self._frame(f"RES{FIELD_SEP}{content}"))
        self.ser.flush()


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="上位机串口桥接节点")
    parser.add_argument("-p", "--port", required=False, help="上位机串口")
    parser.add_argument("-b", "--baudrate", type=int, default=BAUDRATE, help=f"波特率(默认 {BAUDRATE})")
    args, _ = parser.parse_known_args()

    port = args.port
    if not port:
        print("[错误] 未指定上位机串口，请用 -p/--port 指定（如 -p /dev/ttyACM2）")
        sys.exit(1)

    rclpy.init()
    executor = MultiThreadedExecutor()
    node = BridgeNode(port, args.baudrate)
    try:
        executor.add_node(node)
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node._stop.set()
        executor.shutdown()
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()