#!/usr/bin/env python3
"""OpenMV 摄像头任务服务节点。

功能：
  1. 通过 USB 虚拟串口（REPL）与 OpenMV N6 通信
  2. 提供 ROS2 Service /camera/command（openmv_msgs/srv/Command）：
     收到命令后发送到 OpenMV，解析 print 回显，作为服务响应返回
  3. 每次执行结果同时发布 /camera/result（std_msgs/String）、打印并追加 camera.log

用法：
  python3 start.py camera [-p/--port /dev/ttyACM1] [-t/--timeout 秒]

  调用示例：
    ros2 service call /camera/command openmv_msgs/srv/Command "{command: 'TASK1'}"

说明：
  - 命令经 USB REPL 发送（\\r\\n 结尾），OpenMV 需运行 main.py 命令交互模式
  - main.py 中 send() 的结果帧 #<payload>$ 走 UART3 物理引脚，USB 读不到；
    本节点读取的是 print 回显（如 TASK1_OK: 123456 / TASK1_TIMEOUT: 000000）
  - 命令执行耗时由 -t 控制（默认 12s，需覆盖 TASK1 最多 3s + 回显时间）
"""

import argparse
import os
import re
import sys
import time

import serial

from fun import serial_ports

DEFAULT_TIMEOUT = 12.0
CMD_EOL = b"\r\n"
RESULT_TOPIC = "camera/result"
SERVICE_NAME = "camera/command"
LOG_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "camera.log")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from openmv_msgs.srv import Command

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e


def classify_line(line: str) -> tuple[bool, str] | None:
    """按 OpenMV main.py 的 print 回显解析一行；无法识别返回 None。"""
    m = re.match(r"TASK1_OK:\s*(.+)$", line)
    if m:
        return True, f"TASK1_OK: {m.group(1)}"
    m = re.match(r"TASK1_TIMEOUT:\s*(.+)$", line)
    if m:
        return False, f"TASK1_TIMEOUT: {m.group(1)}"
    if line in ("SNAPSHOT_OK", "TRACK_OK", "IRRIGATION_DONE"):
        return True, line
    if line == "UNKNOWN":
        return False, line
    return None


class OpenMVNode(Node):
    """OpenMV 服务节点：接收命令 -> 串口发送 -> 解析回显 -> 返回响应。"""

    def __init__(self, port: str, timeout: float) -> None:
        super().__init__("openmv_node")
        self.port = port
        self.timeout = timeout
        self.srv = self.create_service(Command, SERVICE_NAME, self.handle_command)
        self.pub = self.create_publisher(String, RESULT_TOPIC, 10)
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.get_logger().info(f"串口已打开: {port}, 服务: /{SERVICE_NAME}, 结果话题: /{RESULT_TOPIC}")
        self.get_logger().info(f"调用: ros2 service call /{SERVICE_NAME} openmv_msgs/srv/Command \"{{command: 'TASK1'}}\"")

    def handle_command(self, request, response):
        """服务回调：执行命令并填充响应。"""
        self.get_logger().info(f"收到命令: {request.command}")
        success, text = self.execute(request.command)
        response.result = text if text else ""
        response.success = success if success is not None else False
        self.publish_result(text or "(no marker)", request.command, success)
        return response

    def execute(self, command: str) -> tuple[bool | None, str | None]:
        """发送命令并轮询回显至出现标志行；返回 (success, 标志行) 或 (None, None)。"""
        self.ser.reset_input_buffer()
        self.ser.write(command.encode("utf-8") + CMD_EOL)
        buf = ""
        start = time.time()
        while time.time() - start < self.timeout:
            chunk = self.ser.read(4096).decode("utf-8", errors="replace")
            if not chunk:
                continue
            buf += chunk
            lines = buf.splitlines()
            if len(lines) > 60:            # 防止无限累加，只保留尾部
                lines = lines[-30:]
                buf = "\n".join(lines)
            for line in reversed(lines[-15:]):
                line = line.strip()
                if not line:
                    continue
                hit = classify_line(line)
                if hit:
                    return hit
        return None, None

    def publish_result(self, text: str, command: str, success: bool | None) -> None:
        """打印、追加 camera.log 并发布结果话题。"""
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] 命令: {command} -> {text}  (success={success})")
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(f"{ts} {command} {text} success={success}\n")
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="OpenMV 摄像头任务服务节点")
    parser.add_argument("-p", "--port", help="OpenMV 串口，默认自动探测")
    parser.add_argument("-t", "--timeout", type=float, default=DEFAULT_TIMEOUT, help="执行超时秒(默认 12)")
    args, _ = parser.parse_known_args()

    port = args.port or serial_ports.find_openmv_port()
    if port is None:
        sys.exit(1)

    rclpy.init()
    node = OpenMVNode(port, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()