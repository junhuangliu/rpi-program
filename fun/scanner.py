#!/usr/bin/env python3
"""扫码模块 Python 接收节点。

功能：
  1. 通过串口（USB 串口/VCP 模式）按行读取扫码枪输出的条码 ASCII 数据
  2. 将条码发布到 ROS2 话题 /scanner/barcode（std_msgs/String）
  3. 终端打印并追加写入 scans.log（每行一条条码）
  4. 提供 ROS2 Service /scanner/query（std_srvs/srv/Trigger）：
     上位机随时可调用来获取最近一次扫码内容（缓存）；从未扫过返回 success=false + "NO_SCAN"

用法：
  python3 start.py scanner [-p/--port /dev/ttyACM0] [-b/--baudrate <速率>]

  查询示例：
    ros2 service call /scanner/query std_srvs/srv/Trigger

说明：
- 默认自动探测扫码枪串口（通过 /dev/serial/by-id 稳定识别，区分雷达）
- 端口可用 -p 参数显式指定
- 波特率默认为 115200，编码为 GBK（扫码枪出厂串口参数）；可用 -b 指定其他速率
"""

import argparse
import os
import sys
import time

import serial

from fun import serial_ports

DEFAULT_BAUDRATE = 115200
BARCODE_ENCODING = "gbk"
TOPIC = "scanner/barcode"
QUERY_SERVICE = "scanner/query"
LOG_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scans.log")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    class ScannerNode(Node):
        """扫码接收节点：串口按行读取条码并发布到 ROS2 话题。"""

        def __init__(self, port: str, baudrate: int) -> None:
            super().__init__("scanner_node")
            self.pub = self.create_publisher(String, TOPIC, 10)
            self.last_barcode = ""
            self.srv = self.create_service(Trigger, QUERY_SERVICE, self.handle_query)
            self.ser = serial.Serial(port, baudrate, timeout=0.2)
            self.get_logger().info(f"串口已打开: {port} @ {baudrate}, 发布话题: /{TOPIC}")
            self.get_logger().info(f"服务: /{QUERY_SERVICE} (查询最近条码)")
            self.get_logger().info("请扫描条码（Ctrl+C 退出）")

        def read_loop(self) -> None:
            """主循环：串口按行读取并泵服务回调；timeout 时返回空串。"""
            try:
                while rclpy.ok():
                    line = self.ser.readline()
                    if line:
                        barcode = line.decode(BARCODE_ENCODING, errors="replace").strip()
                        if barcode:
                            self.handle_barcode(barcode)
                    rclpy.spin_once(self, timeout_sec=0.2)
            except serial.SerialException as e:
                self.get_logger().error(f"串口异常: {e}")
            finally:
                self.ser.close()

        def handle_query(self, request, response):
            """查询服务回调：返回最近一次扫码内容。"""
            if self.last_barcode:
                response.success = True
                response.message = self.last_barcode
            else:
                response.success = False
                response.message = "NO_SCAN"
            self.get_logger().info(f"查询最近条码 -> {response.message} (success={response.success})")
            return response

        def handle_barcode(self, barcode: str) -> None:
            """处理一条条码：缓存、终端打印、追加日志、发布 ROS2 话题。"""
            self.last_barcode = barcode
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{ts}] 扫码: {barcode}")
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write(f"{ts} {barcode}\n")
            msg = String()
            msg.data = barcode
            self.pub.publish(msg)

    parser = argparse.ArgumentParser(description="扫码模块接收并发布 ROS2 话题")
    parser.add_argument("-p", "--port", help="串口设备，默认自动探测 /dev/ttyACM*")
    parser.add_argument("-b", "--baudrate", type=int, default=DEFAULT_BAUDRATE, help="波特率(默认 115200)")
    args, _ = parser.parse_known_args()

    port = args.port or serial_ports.find_scanner_port()
    if port is None:
        sys.exit(1)

    rclpy.init()
    node = ScannerNode(port, args.baudrate)
    try:
        node.read_loop()
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()