#!/usr/bin/env python3
"""MaixCAM 网络数据接收节点。

功能：
  1. 启动 TCP Server（默认 0.0.0.0:6000），等待 MaixCAM 主动连接
  2. 按行（\\n）分帧接收 MaixCAM 数据，每条处理为：
       缓存最近一条 + 终端打印 + 追加 maixcam.log + 发布 /maixcam/data
  3. 支持图片帧：MaixCAM 发来的 "IMG <len>\n" + JPEG 字节（如 snap 命令的
       返回），解析后存 snapshots/ 目录 + 覆盖 maixcam_latest.jpg，发布
       /maixcam/photo（String=图片路径）
  4. 提供 ROS2 Service /maixcam/snap（std_srvs/srv/Trigger）：
       原子取图：自动发 mode=pic + snap，等待一帧新 JPEG 返回（成功 message=
       图片路径；失败 NO_CONN / TIMEOUT）
  5. 提供 ROS2 Service /maixcam/query（std_srvs/srv/Trigger）：
       返回最近收到的一条数据；从未收到返回 success=false + "NO_DATA"
  6. 订阅 /maixcam/send（std_msgs/String）：向当前活动的 MaixCAM 连接
       回发消息（自动补 \\n，MaixCAM 端用 recv/readline 即可收到）
  7. 健壮性（长连接场景）：
       - accept 循环在独立线程持续监听，支持多连接与任意重连（进程不退出）
       - 每连接独立线程，默认不设 recv 超时：数据不定时到达时连接持续保持；
         同时启用 TCP keepalive，MaixCAM 掉电/拔线（无 FIN）时由内核在
         keepalive 探测失败后自动断开，不影响后续连接接管
       - 如需要，可用 -t <秒> 设定 recv 超时（0 或负值 = 不超时）

用法：
  python3 start.py maixcam [-p/--port 6000] [-t/--timeout 秒]

MaixCAM 侧只需连接 <树莓派IP>:6000 并按行发送数据，例如：
    import socket
    s = socket.socket(); s.connect(("192.168.x.x", 6000))
    s.send(("hello\n").encode())
    # 接收树莓派下发的消息：
    #   line = s.makefile("r").readline()  或 s.recv(1024)
    # 发图（对应 snap 命令）：先发 "IMG <len>\n" 再发 len 字节 JPEG
    #   s.send(("IMG %d\n" % len(jpg) + jpg))

  查询示例：
    ros2 service call /maixcam/query std_srvs/srv/Trigger

  取图示例（本机触发 MaixCAM 拍照；也可用 control.sh mc）：
    ros2 service call /maixcam/snap std_srvs/srv/Trigger

  下发示例（正向 MaixCAM 发送，自动补 \\n）：
    ros2 topic pub -1 /maixcam/send std_msgs/String "{data: 'award0'}"
    ros2 topic pub -r 1 /maixcam/send std_msgs/String "{data: 'ping'}"
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
DEFAULT_PORT = 6000
RECV_TIMEOUT = None              # 默认不超时=持续连接；-t 秒可设 recv 超时限制
SNAP_WAIT = 5.0                  # /maixcam/snap 等待新图片帧的超时(s)
TOPIC = "maixcam/data"
SEND_TOPIC = "maixcam/send"
PHOTO_TOPIC = "maixcam/photo"    # 发布已收到图片的路径
QUERY_SERVICE = "maixcam/query"
SNAP_SERVICE = "maixcam/snap"    # 原子取图服务
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_FILE = os.path.join(ROOT_DIR, "maixcam.log")
SNAP_DIR = os.path.join(ROOT_DIR, "snapshots")
LATEST_IMG = os.path.join(ROOT_DIR, "maixcam_latest.jpg")


class MaixCAMNode(Node):
    """MaixCAM 接收节点：TCP server 收数据 -> 缓存/日志/topic；query 服务返回最近一条。"""

    def __init__(self, host: str, port: int, recv_timeout: float) -> None:
        super().__init__("maixcam_node")
        self.host = host
        self.port = port
        self.recv_timeout = recv_timeout
        self.last_data = ""
        self.lock = threading.Lock()
        self._active_conns = {}            # addr -> conn，当前活动的 MaixCAM 连接（用于回发消息）
        self.latest_jpg = b""              # 最近一帧 JPEG 字节
        self.image_seq = 0                 # 已收图片帧计数（snap 用于判定"新帧"）
        self._last_img_path = ""
        self.image_event = threading.Event()
        self.pub = self.create_publisher(String, TOPIC, 10)
        self.photo_pub = self.create_publisher(String, PHOTO_TOPIC, 10)
        self.srv = self.create_service(Trigger, QUERY_SERVICE, self.handle_query)
        self.srv_snap = self.create_service(Trigger, SNAP_SERVICE, self.handle_snap)
        self.send_sub = self.create_subscription(
            String, SEND_TOPIC, self.handle_send, 10)
        os.makedirs(SNAP_DIR, exist_ok=True)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(5)
        self.server.settimeout(1.0)          # accept 超时，便于响应退出
        self._stop = threading.Event()
        self._acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self._acceptor.start()
        self.get_logger().info(
            f"TCP Server 已启动: {host}:{port}, 发布话题 /{TOPIC}/, /{PHOTO_TOPIC}(图), "
            f"订阅 /{SEND_TOPIC}(下发), 服务 /{QUERY_SERVICE} 和 /{SNAP_SERVICE}(取图), "
            f"keepalive 已启用, "
            f"recv 超时 {'不超时(持续连接)' if self.recv_timeout is None else f'{self.recv_timeout}s'}, "
            "等待 MaixCAM 连接...")

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
            with self.lock:
                self._active_conns[addr] = conn
            t = threading.Thread(target=self.handle_client, args=(conn, addr),
                                 daemon=True)
            t.start()

    def handle_client(self, conn: socket.socket, addr) -> None:
        """处理单个连接：字节级分帧接收文本行与 IMG 图片帧；断开/超时/异常后关闭。"""
        # 长连接 keepalive：掉电/拔线（无 FIN）由内核探测后断开，不设 recv 超时则永久保持
        try:
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)
        except OSError:
            pass
        conn.settimeout(self.recv_timeout)
        buffer = b""
        pending = 0
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
                buffer += data
                while True:
                    if pending:
                        if len(buffer) < pending:
                            break
                        jpeg = buffer[:pending]
                        buffer = buffer[pending:]
                        pending = 0
                        self.handle_image(addr, jpeg)
                        continue
                    nl = buffer.find(b"\n")
                    if nl < 0:
                        break
                    line, buffer = buffer.split(b"\n", 1)
                    text = line.strip().decode("utf-8", errors="ignore")
                    if text.startswith("IMG "):
                        try:
                            pending = int(text[4:].strip())
                        except ValueError:
                            pending = 0
                    elif text:
                        self.handle_data(text)
        except Exception as e:
            self.get_logger().error(f"MaixCAM({addr}) 连接异常: {e}")
        finally:
            with self.lock:
                self._active_conns.pop(addr, None)
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

    def handle_image(self, addr, jpeg: bytes) -> None:
        """处理一帧 JPEG：落盘 snapshots/ + 覆盖 latest，发布 /maixcam/photo。"""
        if not jpeg:
            return
        with self.lock:
            seq = self.image_seq
            self.image_seq += 1
            self.latest_jpg = jpeg
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{seq:03d}.jpg"
        path = os.path.join(SNAP_DIR, name)
        try:
            with open(path, "wb") as f:
                f.write(jpeg)
            with open(LATEST_IMG, "wb") as f:
                f.write(jpeg)
            with self.lock:
                self._last_img_path = path
        except OSError as e:
            self.get_logger().error(f"图片落盘失败: {e}")
            return
        self.image_event.set()
        print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] 已收到图片 {path} ({len(jpeg)} B)")
        self.get_logger().info(f"MaixCAM({addr}) 已收到图片: {path} ({len(jpeg)} B)")
        msg = String()
        msg.data = path
        self.photo_pub.publish(msg)

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

    def send_line(self, line: str) -> bool:
        """向当前所有活动 MaixCAM 连接发送一行（自动补 \\n）；无连接返回 False。"""
        payload = (line if line.endswith("\n") else line + "\n").encode("utf-8")
        with self.lock:
            conns = list(self._active_conns.values())
        if not conns:
            self.get_logger().warn(f"下发 {line!r} 失败: 当前无活动连接")
            return False
        ok = True
        for conn in conns:
            try:
                conn.sendall(payload)
                self.get_logger().info(f"已发送到 MaixCAM: {line!r}")
            except OSError as e:
                ok = False
                self.get_logger().warn(f"发送失败({conn.getpeername()}): {e}")
        return ok

    def handle_send(self, msg: String) -> None:
        """下发回调：把 msg.data 通过当前活动的 MaixCAM 连接发回（自动补 \\n）。"""
        if not msg.data:
            return
        self.send_line(msg.data)

    def handle_snap(self, request, response):
        """取图服务回调：自动发 mode=pic + snap，等待一帧新 JPEG 返回其路径。

        成功 -> success=True, message=<图片路径>；无连接 -> NO_CONN；超时 -> TIMEOUT。
        """
        with self.lock:
            base_seq = self.image_seq
        self.image_event.clear()
        if not self.send_line("mode=pic") or not self.send_line("snap"):
            response.success = False
            response.message = "NO_CONN"
            return response
        deadline = time.monotonic() + SNAP_WAIT
        path = ""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self.image_event.wait(remaining):
                with self.lock:
                    if self.image_seq > base_seq:
                        path = self._last_img_path
                        self.image_event.clear()
                        break
                    self.image_event.clear()
        if not path:
            self.get_logger().warn("snap 等待超时(未收到新图片)")
            response.success = False
            response.message = "TIMEOUT"
            return response
        self.get_logger().info(f"snap 成功: {path}")
        response.success = True
        response.message = path
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
                        help="连接 recv 超时秒(默认不超时=持续连接；0/缺省=不超时，>0 为秒数)")
    args, _ = parser.parse_known_args()
    if args.timeout is not None and args.timeout <= 0:
        args.timeout = None

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