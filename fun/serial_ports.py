#!/usr/bin/env python3
"""USB 串口稳定识别工具（区分雷达与扫码模块）。

内核按插入顺序为 /dev/ttyUSBx、/dev/ttyACMx 编号，插拔顺序一变端口名就
可能互换，不能据此区分设备。本模块优先读 /dev/serial/by-id/ 稳定链接
（名称含 USB 厂商/产品/序列号），无 by-id 时回退 sysfs idVendor:idProduct 匹配。

识别约定：
  - 雷达   RPLIDAR C1：CP210x（Silicon_Labs），PID ea60
  - 扫码枪 BF SCAN：   链接名含 "BF_SCAN"，VID 9901
  - OpenMV 摄像头：    PYBoard 虚拟串口（MicroPython），VID 37c5
"""

import glob
import os

LIDAR_KEYWORDS = ("Silicon_Labs", "CP210")
LIDAR_PID = "ea60"
SCANNER_KEYWORDS = ("BF_SCAN",)
SCANNER_VID = "9901"
OPENMV_KEYWORDS = ("MicroPython",)
OPENMV_VID = "37c5"


def by_id_ports() -> dict[str, str]:
    """返回 {实际串口路径: by-id 链接名}，无 by-id 时返回空 dict。"""
    mapping = {}
    for link in glob.glob("/dev/serial/by-id/usb-*"):
        target = os.readlink(link)
        port = os.path.join("/dev", os.path.basename(target))
        mapping[port] = os.path.basename(link)
    return mapping


def _sysfs_vid_pid(port: str) -> str:
    """读取端口对应的 USB 设备 VID:PID（如 "10c4:ea60"），读取失败返回空串。"""
    dev = os.path.join("/sys/class/tty", os.path.basename(port))
    cur = os.path.realpath(os.path.join(dev, "device")) if os.path.exists(dev) else dev
    while True:
        vid = os.path.join(cur, "idVendor")
        if os.path.exists(vid) and os.path.exists(os.path.join(cur, "idProduct")):
            with open(vid) as f:
                v = f.read().strip()
            with open(os.path.join(cur, "idProduct")) as f:
                p = f.read().strip()
            return f"{v}:{p}"
        parent = os.path.dirname(cur)
        if parent == cur:
            return ""
        cur = parent


def list_ports() -> list[tuple[str, str]]:
    """列出所有串口设备 [(端口, VID:PID)]，用于排查。"""
    out = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for port in glob.glob(pattern):
            out.append((port, _sysfs_vid_pid(port)))
    return out


def find_port(keywords: tuple[str, ...], vid: str | None = None,
              pid: str | None = None, label: str = "设备") -> str | None:
    """按 by-id 关键词优先、sysfs VID/PID 兜底匹配串口，未找到返回 None。"""
    byid = by_id_ports()
    for port, link in byid.items():
        if any(k in link for k in keywords):
            print(f"[OK] 检测到{label}串口: {port}  (by-id: {link})")
            return port
    for port, tag in list_ports():
        if vid and not tag.startswith(vid):
            continue
        if pid and not tag.endswith(pid):
            continue
        if vid or pid:
            print(f"[OK] 检测到{label}串口(by PID): {port}  ({tag})")
            return port
    print(f"[错误] 未检测到{label}（by-id 关键词: {keywords}，PID: {vid}:{pid}）")
    current = list_ports()
    if current:
        print("[提示] 当前串口设备:")
        for port, tag in current:
            print(f"        {port}  ({tag})")
    else:
        print("[提示] 当前无任何串口设备，请检查 USB 连接")
    return None


def find_lidar_port() -> str | None:
    """返回雷达串口；未找到返回 None。"""
    return find_port(LIDAR_KEYWORDS, pid=LIDAR_PID, label="雷达")


def find_scanner_port() -> str | None:
    """返回扫码枪串口；未找到返回 None。"""
    return find_port(SCANNER_KEYWORDS, vid=SCANNER_VID, label="扫码枪")


def find_openmv_port() -> str | None:
    """返回 OpenMV 摄像头串口；未找到返回 None。"""
    return find_port(OPENMV_KEYWORDS, vid=OPENMV_VID, label="OpenMV")


if __name__ == "__main__":
    find_lidar_port()
    find_scanner_port()
    find_openmv_port()