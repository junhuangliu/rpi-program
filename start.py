#!/usr/bin/env python3
"""项目启动入口：集成调用 fun 文件夹中的各个功能模块。"""

import sys

from fun import bridge, camera, scanner, open_lidar, obstacle, slam

FUNCS = {
    "lidar": open_lidar.main,
    "slam": slam.main,
    "scanner": scanner.main,
    "camera": camera.main,
    "bridge": bridge.main,
    "obstacle": obstacle.main,
}


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in FUNCS:
        print("用法: python start.py <功能名>")
        print("可用功能:")
        for name in FUNCS:
            print(f"  {name}")
        sys.exit(1)
    FUNCS[sys.argv[1]]()


if __name__ == "__main__":
    main()