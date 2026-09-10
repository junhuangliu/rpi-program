#!/usr/bin/env python3
"""扫码模块 Python 接收节点。

功能：
  1. 通过串口（USB 串口/VCP 模式）按条码结束符分帧读取扫码枪输出的条码数据
     （整条内容原样保留，含内容内部回车，不按换行切分；避免一个二维码被拆成多条）
  2. 将条码发布到 ROS2 话题 /scanner/barcode（std_msgs/String）
  3. 终端打印并追加写入 scans.log（每行一条条码）
  4. 提供 ROS2 Service /scanner/query（std_srvs/srv/Trigger）：
     上位机随时可调用来获取最近一次扫码内容（缓存）；从未扫过返回 success=false + "NO_SCAN"

用法：
  python3 start.py scanner [-p/--port /dev/ttyACM0] [-b/--baudrate <速率>] [-e/--eol \t]

  查询示例：
    ros2 service call /scanner/query std_srvs/srv/Trigger

说明：
- 默认自动探测扫码枪串口（通过 /dev/serial/by-id 稳定识别，区分雷达）
- 端口可用 -p 参数显式指定
- 波特率默认为 115200，编码为 GBK（扫码枪出厂串口参数）；可用 -b 指定其他速率
- 结束符（--eol）默认 Tab(\\t)：需扫码枪设置"条码后缀=Tab"，否则含 \\n 的二维码会被拆断。
  若枪用回车结束则指定 -e '\\n' 可退回旧式按行模型。
"""

import argparse
import os
import re
import sys
import time

import serial

from fun import serial_ports

DEFAULT_BAUDRATE = 115200
DEFAULT_EOL = "\t"               # 条码结束符：须与扫码枪"后缀"设置一致（默认 Tab）
BARCODE_ENCODING = "gbk"
TOPIC = "scanner/barcode"
QUERY_SERVICE = "scanner/query"
QUERY_PARSED_SERVICE = "scanner/query_parsed"
CODE_MAP = {"轻微干旱": "1", "一般干旱": "2", "严重干旱": "3"}
LOG_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scans.log")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e


def eol_bytes(s: str) -> bytes:
    """把命令行结束符字符串（支持 \\t \\r \\n \\x02 等转义）转成 bytes。"""
    return s.encode("utf-8").decode("unicode_escape").encode("utf-8")


def parse_code(content: str) -> str:
    """解析最近扫码内容：首段=编号(纯数字)，后 4 段=情况(轻微=1/一般=2/严重=3)。

    返回编号+映射数字直接拼接（例：1246+2213 → "12462213"）。
    段数不符 / 编号非数字 / 未知情况均抛 ValueError（整体报错）。
    """
    segments = [s.strip() for s in re.split(r"\r\n|\r|\n", content)]
    segments = [s for s in segments if s]
    if len(segments) != 5:
        raise ValueError(f"段数不符: {len(segments)}，期望 1个编号+4行情况")
    code, states = segments[0], segments[1:]
    if not code.isdigit():
        raise ValueError(f"编号非数字: {code!r}")
    mapped = []
    for st in states:
        if st not in CODE_MAP:
            raise ValueError(f"未知情况: {st!r}")
        mapped.append(CODE_MAP[st])
    return code + "".join(mapped)


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    class ScannerNode(Node):
        """扫码接收节点：串口按结束符切分条码（整条内容原样保留）并发布到 ROS2 话题。"""

        def __init__(self, port: str, baudrate: int, eol: str) -> None:
            super().__init__("scanner_node")
            self.pub = self.create_publisher(String, TOPIC, 10)
            self.last_barcode = ""
            self.srv = self.create_service(Trigger, QUERY_SERVICE, self.handle_query)
            self.srv_parsed = self.create_service(Trigger, QUERY_PARSED_SERVICE, self.handle_query_parsed)
            self.ser = serial.Serial(port, baudrate, timeout=0.2)
            self.eol = eol_bytes(eol)
            self.get_logger().info(f"串口已打开: {port} @ {baudrate}, 发布话题: /{TOPIC}")
            self.get_logger().info(f"服务: /{QUERY_SERVICE} (查询最近条码); /{QUERY_PARSED_SERVICE} (解析后查询)")
            self.get_logger().info(f"条码结束符: {self.eol!r} (整条内容保留，不按换行切分)")
            self.get_logger().info("请扫描条码（Ctrl+C 退出）")

        def read_loop(self) -> None:
            """主循环：分块缓存串口字节，遇结束符才切出一条条码；timeout 时轮询服务回调。"""
            buf = b""
            try:
                while rclpy.ok():
                    n = self.ser.in_waiting or 1
                    chunk = self.ser.read(n)
                    if chunk:
                        buf += chunk
                        parts = buf.split(self.eol)
                        buf = parts.pop()  # 未终结的残尾留到下次
                        for piece in parts:
                            if piece:
                                barcode = piece.decode(BARCODE_ENCODING, errors="replace")
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

        def handle_query_parsed(self, request, response):
            """解析查询服务回调：对最近扫码内容执行 parse_code 后返回（失败时 ERROR:<原因>）。"""
            if not self.last_barcode:
                response.success = False
                response.message = "NO_SCAN"
            else:
                try:
                    response.success = True
                    response.message = parse_code(self.last_barcode)
                except ValueError as e:
                    response.success = False
                    response.message = f"ERROR: {e}"
            self.get_logger().info(
                f"查询解析条码 -> {response.message} (success={response.success})")
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
    parser.add_argument("-e", "--eol", type=str, default=DEFAULT_EOL,
                        help=f"条码结束符(默认 {DEFAULT_EOL!r}，支持转义如 \\n、\\x02)")
    args, _ = parser.parse_known_args()

    port = args.port or serial_ports.find_scanner_port()
    if port is None:
        sys.exit(1)

    rclpy.init()
    node = ScannerNode(port, args.baudrate, args.eol)
    try:
        node.read_loop()
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()