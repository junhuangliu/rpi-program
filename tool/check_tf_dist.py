#!/usr/bin/env python3
"""TF 两级位移实时排查工具（定位 50cm 显示 45.2cm 的误差来源）。

用法：
  python3 tool/check_tf_dist.py

流程：
  1. 车停在起点，启动脚本。按一次回车 → 把当前位姿记为起点（显示全 0.000）
  2. 沿直线向前推 50cm，边推边看三列数字实时增长
  3. 停稳后，三列稳定值即各级位移；再按回车可重置起点重测

显示格式（单位 m）：起点系下 (前方 x, 左侧 y)：
  odom->laser     matcher 纯激光里程计本级（主排查对象）
  map->laser      含 slam_toolbox 在线修正的总链路
  map->base_link  上位机 #POS 实际读到并显示的值

判断：
  若 odom->laser 已只有 ~0.45 → 误差在 matcher（纯激光 ICP 欠估）
  若 odom->laser ≈0.50 而 map->* 只有 0.45 → 误差在 slam_toolbox 在线修正
  若推车时全列为 0.000 → 推车时段没被 matcher/slam 识别（检查雷达与推车方式）
"""

import math
import select
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

KEYS = ("odom->laser", "map->laser", "map->base_link")


def quat_to_yaw(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TfWatchNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_watch_dist")
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.ref: dict[str, tuple] = {k: None for k in KEYS}   # 起点位姿 (x,y,yaw)
        self.cur: dict[str, tuple] = {k: (0.0, 0.0, 0.0) for k in KEYS}

    def lookup(self, parent: str, child: str):
        f = self.buf.lookup_transform(parent, child, rclpy.time.Time())
        t, q = f.transform.translation, f.transform.rotation
        return t.x, t.y, quat_to_yaw(q)

    def refresh(self) -> None:
        for key in KEYS:
            parent, child = key.split("->")
            try:
                self.cur[key] = self.lookup(parent, child)
            except Exception:
                self.cur[key] = None

    def rel(self, key: str):
        src, dst = self.ref[key], self.cur[key]
        if src is None or dst is None:
            return None
        x0, y0, yaw0 = src
        dx, dy = dst[0] - x0, dst[1] - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        return (dx * c + dy * s, -dx * s + dy * c)

    def reset_ref(self) -> None:
        for key in KEYS:
            v = self.cur[key]
            self.ref[key] = v if v is not None else None


def main() -> None:
    rclpy.init()
    node = TfWatchNode()

    print("等待 TF 就绪...", flush=True)
    while True:
        node.refresh()
        if all(node.cur[k] is not None for k in KEYS):
            break
        time.sleep(0.2)

    node.reset_ref()
    print("按 Enter 重置起点；推车时观察下方位移（单位 m，前,左）\n", flush=True)

    try:
        while rclpy.ok():
            node.refresh()
            fields = []
            for key in KEYS:
                v = node.rel(key)
                if v is None:
                    fields.append(f"{key:14s}   <无TF>")
                else:
                    fields.append(f"{key:14s}  {v[0]:+7.3f},{v[1]:+7.3f}")
            line = "  |  ".join(fields)
            sys.stdout.write(f"\r{line}  ")
            sys.stdout.flush()

            if select.select([sys.stdin], [], [], 0.2)[0]:
                sys.stdin.readline()
                node.reset_ref()
                sys.stdout.write("\r--- 起点已重置 ---             \n")
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