#!/usr/bin/env python3
"""整车初始坐标系换算（bridge #POS 与 tool/pos.py 共用，保证本地看到的=上位机收到的）。

整车初始系：以首次拿到 TF 位姿的时刻为原点/零角度（启动即 0,0,0）。
  - x 正 = 初始时刻车头前方；y 正 = 初始时刻左侧
  - yaw 用导航惯例（0=初始前方、左转为负）：对相对角取负，并归约到 [-π, π]
首次之后的换算：相对位移 (dx,dy) 绕 z 转 -yaw0 到初始系。
"""

import math


def quat_to_yaw(q) -> float:
    """四元数转偏航角（弧度）。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rel_to_origin(x: float, y: float, yaw: float, origin: tuple | None) -> tuple[float, float, float]:
    """把当前位姿 (x,y,yaw) 换算到以 origin 为原点的整车初始系，返回 (x_f, y_f, yaw_f)。

    origin 为 None 时视为首次（返回 0,0,0，由调用方负责记录 origin）。
    """
    if origin is None:
        return 0.0, 0.0, 0.0
    x0, y0, yaw0 = origin
    dx, dy = x - x0, y - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    x_f = dx * c + dy * s
    y_f = -dx * s + dy * c
    yaw_f = -math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))  # [-π, π]
    return x_f, y_f, yaw_f