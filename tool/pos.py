#!/usr/bin/env python3
"""实时显示与上位机收到的 #POS 完全一致的坐标帧（无 STM32 串口也能看）。

数据来源与 bridge 相同：
  - 读 odom->base_link（与 bridge 默认 -f/--frame odom 一致，激光里程计、位移准）
  - 整车初始系换算与 bridge 共用 fun/coords.py（首次位姿归零 → 启动即 #POS,0,0,0）

用法：
  ./control.sh pos          或  python3 tool/pos.py
  Enter        重新记录原点（模拟 bridge 重启，归零）
  Ctrl+C       退出

输出：`#POS,x,y,yaw$`，与 bridge 发给上位机的字节完全一致。
"""

import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from fun import coords

FRAME = "odom"   # 与 bridge 默认计程帧一致


class PosNode(Node):
    def __init__(self) -> None:
        super().__init__("host_pos_view")
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.origin: tuple | None = None

    def current(self) -> tuple[float, float, float]:
        f = self.buf.lookup_transform(FRAME, "base_link", rclpy.time.Time())
        t, q = f.transform.translation, f.transform.rotation
        return t.x, t.y, coords.quat_to_yaw(q)


def main() -> None:
    rclpy.init()
    node = PosNode()

    print("等待 TF 就绪...", flush=True)
    while True:
        try:
            node.current()
            break
        except Exception:
            time.sleep(0.2)

    print("按 Enter 重新记录原点；下行为发给上位机的帧（与 #POS 完全一致）\n", flush=True)
    try:
        while rclpy.ok():
            try:
                x, y, yaw = node.current()
            except Exception:
                line = "POS,no_tf"
            else:
                if node.origin is None:
                    node.origin = (x, y, yaw)
                x_f, y_f, yaw_f = coords.rel_to_origin(x, y, yaw, node.origin)
                line = f"POS,{x_f:.3f},{y_f:.3f},{yaw_f:.3f}"
            sys.stdout.write(f"\r#{line}$  ")
            sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0.2)[0]:
                sys.stdin.readline()
                node.origin = None
                sys.stdout.write("\r--- 原点已重置 ---             \n")
                sys.stdout.flush()
            else:
                time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()