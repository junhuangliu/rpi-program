#!/usr/bin/env python3
"""打开 RPLIDAR C1 激光雷达。

将以下手工步骤统一封装为一个 Python 脚本：
  1. 加载 ROS 工作空间环境变量
  2. 自动检测雷达串口设备（通过 /dev/serial/by-id 稳定识别，区分扫码枪）
  3. 启动 RPLIDAR C1 显示节点（自动打开 rviz2）

说明：串口权限通过将用户加入 dialout 组解决，无需每次授权。
"""

import argparse
import os
import subprocess
import sys

from fun import serial_ports

ROS_SETUP = os.path.expanduser("~/ros2_ws/install/setup.bash")
LIDAR_LAUNCH = "view_rplidar_c1_launch.py"
LIDAR_PKG = "rplidar_ros"


def run(cmd: str, check: bool = True) -> None:
    print(f"\n>>> {cmd}")
    subprocess.run(["bash", "-c", cmd], check=check)


def ros_env_prefix() -> str:
    if not os.path.exists(ROS_SETUP):
        print(f"[警告] 未找到 {ROS_SETUP}，若 ROS 环境已全局生效可忽略")
        return ""
    return f"source {ROS_SETUP} && "


def main() -> None:
    parser = argparse.ArgumentParser(description="打开 RPLIDAR C1 激光雷达")
    parser.add_argument("-p", "--port", help="雷达串口（覆盖自动识别）")
    args, _ = parser.parse_known_args()

    port = args.port or serial_ports.find_lidar_port()
    if port is None:
        sys.exit(1)

    launch = (f"{ros_env_prefix()}ros2 launch {LIDAR_PKG} {LIDAR_LAUNCH} "
              f"serial_port:={port}")
    run(launch)


if __name__ == "__main__":
    main()