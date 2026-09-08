#!/usr/bin/env python3
"""打开 RPLIDAR C1 激光雷达。

将以下手工步骤统一封装为一个 Python 脚本：
  1. 加载 ROS 工作空间环境变量
  2. 自动检测雷达串口设备（/dev/ttyUSB* 或 /dev/ttyACM*）
  3. 启动 RPLIDAR C1 显示节点（自动打开 rviz2）

说明：串口权限通过将用户加入 dialout 组解决，无需每次授权。
"""

import glob
import os
import subprocess
import sys

ROS_SETUP = os.path.expanduser("~/ros2_ws/install/setup.bash")
LIDAR_LAUNCH = "view_rplidar_c1_launch.py"
LIDAR_PKG = "rplidar_ros"
SERIAL_PATTERNS = ("/dev/ttyUSB*", "/dev/ttyACM*")


def run(cmd: str, check: bool = True) -> None:
    print(f"\n>>> {cmd}")
    subprocess.run(["bash", "-c", cmd], check=check)


def ros_env_prefix() -> str:
    if not os.path.exists(ROS_SETUP):
        print(f"[警告] 未找到 {ROS_SETUP}，若 ROS 环境已全局生效可忽略")
        return ""
    return f"source {ROS_SETUP} && "


def find_lidar_port() -> str | None:
    ports = sorted(
        set(p for pattern in SERIAL_PATTERNS for p in glob.glob(pattern))
    )
    if not ports:
        print("[错误] 未检测到雷达串口设备，请确认雷达 USB 线已连接")
        return None
    print(f"[OK] 检测到串口设备: {', '.join(ports)}")
    return ports[0]


def main() -> None:
    port = find_lidar_port()
    if port is None:
        sys.exit(1)

    launch = f"{ros_env_prefix()}ros2 launch {LIDAR_PKG} {LIDAR_LAUNCH}"
    run(launch)


if __name__ == "__main__":
    main()