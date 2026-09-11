#!/usr/bin/env python3
"""MaixCAM 网络数据接收节点。

功能：
  1. 启动 TCP Server（默认 0.0.0.0:8080），等待 MaixCAM 主动连接
  2. 按行（\\n）分帧接收 MaixCAM 数据，每条处理为：
       缓存最近一条 + 终端打印 + 追加 maixcam.log + 发布 /maixcam/data
  3. 提供 ROS2 Service /maixcam/query（std_srvs/srv/Trigger）：
       返回最近收到的一条数据；从未收到返回 success=false + "NO_DATA"
  4. 健壮性：
       - accept 循环在独立线程持续监听，支持多连接与任意重连（进程不退出）
       - 每连接独立线程 + recv 超时（默认 5s）：MaixCAM 网络异常消失（TCP
         半开）时超时主动关闭该连接，不影响后续连接接管

用法：
  python3 start.py maixcam [-p/--port 8080] [-t/--timeout 5]

  MaixCAM 侧只需连接 <树莓派IP>:8080 并按行发送数据，例如：
    import socket
    s = socket.socket(); s.connect(("192.168.x.x", 8080))
    s.send(("hello\\n").encode())

  查询示例：
    ros2 service call /maixcam/query std_srvs/srv/Trigger
"""

import argparse
import os
import socket
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e

HOST = "0.0.0.0"                 # 监听所有网卡
DEFAULT_PORT = 8080
RECV_TIMEOUT = 5.0               # 连接 recv 超时(s)，防 TCP 半开卡死
TOPIC = "maixcam/data"
QUERY_SERVICE = "maixcam/query"
LOG_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "maixcam.log")


class MaixCAMNode(Node):
    """MaixCAM 接收节点：TCP server 收数据 -> 缓存/日志/topic；query 服务返回最近一条。"""

    def __init__(self, host: str, port: int, recv_timeout: float) -> None:
        super().__init__("maixcam_node")
        self.host = host
        self.port = port
        self.recv_timeout = recv_timeout
        self.last_data = ""
        self.lock = threading.Lock()
        self.pub = self.create_publisher(String, TOPIC, 10)
        self.srv = self.create_service(Trigger, QUERY_SERVICE, self.handle_query)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(5)
        self.server.settimeout(1.0)          # accept 超时，便于响应退出
        self._stop = threading.Event()
        self._acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self._acceptor.start()
        self.get_logger().info(
            f"TCP Server 已启动: {host}:{port}, 发布话题 /{TOPIC}, "
            f"服务 /{QUERY_SERVICE}, recv 超时 {recv_timeout}s, 等待 MaixCAM 连接...")

    def accept_loop(self) -> None:
        """持续 accept，每个连接开独立线程处理；监听直到退出。"""
        while rclpy.ok() and not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.get_logger().info(f"MaixCAM 已连接: {addr}")
            t = threading.Thread(target=self.handle_client, args=(conn, addr),
                                 daemon=True)
            t.start()

    def handle_client(self, conn: socket.socket, addr) -> None:
        """处理单个连接：按行分帧接收，每条处理；断开/超时/异常后关闭连接。"""
        conn.settimeout(self.recv_timeout)
        buffer = ""
        try:
            while rclpy.ok() and not self._stop.is_set():
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    self.get_logger().warn(f"MaixCAM({addr}) 连接超时，主动断开")
                    break
                if not data:
                    self.get_logger().info(f"MaixCAM 断开: {addr}")
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self.handle_data(line)
        except Exception as e:
            self.get_logger().error(f"MaixCAM({addr}) 连接异常: {e}")
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def handle_data(self, line: str) -> None:
        """处理一条数据：缓存、打印、日志、发布 topic。"""
        with self.lock:
            self.last_data = line
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] MaixCAM: {line}")
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(f"{ts} {line}\n")
        msg = String()
        msg.data = line
        self.pub.publish(msg)
        self.get_logger().info(f"发布 /{TOPIC}: {line}")

    def handle_query(self, request, response):
        """查询服务回调：返回最近收到的数据；从未收到返回 NO_DATA。"""
        with self.lock:
            data = self.last_data
        if data:
            response.success = True
            response.message = data
        else:
            response.success = False
            response.message = "NO_DATA"
        self.get_logger().info(
            f"查询最近数据 -> {response.message} (success={response.success})")
        return response

    def shutdown(self) -> None:
        """停止 server socket 并等待 acceptor 退出。"""
        self._stop.set()
        try:
            self.server.close()
        except OSError:
            pass


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="MaixCAM 网络数据接收节点")
    parser.add_argument("-p", "--port", type=int, default=DEFAULT_PORT,
                        help=f"TCP 监听端口(默认 {DEFAULT_PORT})")
    parser.add_argument("-t", "--timeout", type=float, default=RECV_TIMEOUT,
                        help=f"连接 recv 超时秒(默认 {RECV_TIMEOUT})，防 TCP 半开")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MaixCAMNode(HOST, args.port, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()