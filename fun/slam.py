#!/usr/bin/env python3
"""手持式 2D 激光 SLAM（建图 + 定位）。

启动 slam_launch.py，一次性拉起：
  雷达驱动 -> 激光里程计 -> slam_toolbox -> rviz2

说明：
  - 需要 slam_toolbox（apt 安装）、ros2_laser_scan_matcher（源码编译）
  - 手持缓慢移动雷达即可边建图边定位
"""

import argparse
import os
import subprocess
import sys

from fun import serial_ports

ROS_SETUP = os.path.expanduser("~/ros2_ws/install/setup.bash")
LAUNCH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "slam_launch.py")


def run(cmd: str, check: bool = True) -> None:
    print(f"\n>>> {cmd}")
    subprocess.run(["bash", "-c", cmd], check=check)


def ros_env_prefix() -> str:
    if not os.path.exists(ROS_SETUP):
        print(f"[警告] 未找到 {ROS_SETUP}，若 ROS 环境已全局生效可忽略")
        return ""
    return f"source {ROS_SETUP} && "


def main() -> None:
    parser = argparse.ArgumentParser(description="手持式 2D 激光 SLAM")
    parser.add_argument("-p", "--port", help="雷达串口（覆盖自动识别）")
    args, _ = parser.parse_known_args()

    port = args.port or serial_ports.find_lidar_port()
    if port is None:
        sys.exit(1)

    launch = (f"{ros_env_prefix()}ros2 launch {LAUNCH_FILE} "
              f"serial_port:={port}")
    run(launch)


if __name__ == "__main__":
    main()
