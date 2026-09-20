"""
OpenMV N6 摄像头视觉识别程序 —— 命令交互模式

功能：
  1. 启动时红绿蓝 LED 闪烁（machine.LED）
  2. 初始化摄像头（统一使用 sensor，RGB565 彩色）
  3. 等待接收指令：
     - TASK1       六色矩形检测（引用 test1.task1）
     - TASK2       二维码识别
     - SNAPSHOT    快照（占位）
     - TRACK       跟踪（占位）
     - IRRIGATION  灌溉（占位）
     - exit / quit 退出命令行循环
用法：
  在 OpenMV IDE 里连接摄像头 -> 点运行 -> 在串口终端输入命令回车；
  或由上位机节点 fun/camera.py 经 USB REPL（/r/n 结尾）发送命令并解析 print 回显。
回显协议（print 走 USB，上位机据此解析）：
  TASK1_OK: xxxxxx        / TASK1_TIMEOUT: xxxxxx
  TASK2_OK: <二维码内容>  / TASK2_TIMEOUT: <最近内容>
  SNAPSHOT_OK / TRACK_OK / IRRIGATION_DONE / UNKNOWN
说明：
  send() 的 #<payload>$ 帧走 UART3 物理引脚，USB 收不到，仅供外接串口读取。
"""

import pyb
import sensor
import time
from machine import LED
from test1 import task1

# ================= 配置 =================
LED_DELAY_MS = 200        # 每颗 LED 点亮/熄灭的时长（毫秒）
TASK1_TIMEOUT = 3000      # 六色矩形检测超时（毫秒）
TASK2_TIMEOUT = 3000      # 二维码识别超时（毫秒）

UART_ID = 3               # 使用的 UART 外设编号
UART_BAUDRATE = 115200    # 串口波特率
FRAME_START = b"#"        # 帧起始符
FRAME_END   = b"$"        # 帧结束符
# =======================================

# ================= 串口初始化（只创建一次） =================
uart = pyb.UART(UART_ID, UART_BAUDRATE)

# ================= 统一串口发送函数 =================
def send(payload: str):
    """按统一帧格式发送：#<payload>$。"""
    try:
        data = FRAME_START + payload.encode('utf-8') + FRAME_END
        uart.write(data)
        uart.flush()
    except Exception as e:
        print("SEND_ERR", e)


# ================= 红绿蓝 LED 闪烁 =================
def rgb_blink():
    """依次点亮红色、蓝色、绿色 LED 各一次，用于提示程序启动。"""
    for name in ("LED_RED", "LED_BLUE", "LED_GREEN"):
        led = LED(name)
        led.on()
        time.sleep_ms(LED_DELAY_MS)
        led.off()
        time.sleep_ms(LED_DELAY_MS)


# ================= 任务函数 =================
# 每个命令对应一个任务函数，函数只负责执行动作并 print 结果。

def task_main_1():
    """TASK1：六色矩形检测。循环调 task1()，命中（结果无 '0'）即发成功，超时发最近结果。"""
    start = time.ticks_ms()
    final_result = "000000"

    while time.ticks_diff(time.ticks_ms(), start) < TASK1_TIMEOUT:
        result = task1()
        if result and "0" not in result:
            send(result)
            print(f"TASK1_OK: {result}")
            return
        final_result = result
        time.sleep_ms(10)

    send(f"{final_result}<TIMEOUT>")
    print(f"TASK1_TIMEOUT: {final_result}")


def task_main_2():
    """TASK2：二维码识别。命中即发内容，超时发最近识别内容（无则空）。"""
    start = time.ticks_ms()
    last_payload = ""

    while time.ticks_diff(time.ticks_ms(), start) < TASK2_TIMEOUT:
        img = sensor.snapshot()
        codes = img.find_qrcodes()
        if codes:
            payload = str(codes[0].payload())
            send(payload)
            print(f"TASK2_OK: {payload}")
            return
        last_payload = ""
        time.sleep_ms(10)

    send(f"{last_payload}<TIMEOUT>")
    print(f"TASK2_TIMEOUT: {last_payload}")


def task_snapshot():
    """SNAPSHOT：快照任务（占位）。"""
    print("SNAPSHOT_OK")


def task_track():
    """TRACK：跟踪任务（占位）。"""
    print("TRACK_OK")


def task_irrigation():
    """IRRIGATION：灌溉任务（占位）。"""
    print("IRRIGATION_DONE")


def task_default():
    """UNKNOWN：未知命令兜底处理。"""
    print("UNKNOWN")


# ================= 命令分派 =================
# 分派表：关键词 -> 对应任务函数。
# 收到以关键词开头的行即调用对应函数，未匹配则使用 task_default。
DISPATCH = {
    "TASK1":      task_main_1,
    "TASK2":      task_main_2,
    "SNAPSHOT":   task_snapshot,
    "TRACK":      task_track,
    "IRRIGATION": task_irrigation,
}


def dispatch(line):
    """解析一行命令，按关键词调用任务函数。"""
    line = line.strip()            # 去掉首尾空白和换行
    if not line:                   # 空行忽略
        return
    print("[命令] " + line)
    keyword = line.split()[0]      # 取第一个词作为关键词
    func = DISPATCH.get(keyword, task_default)
    func()


# ================= 主流程 =================
def main():
    """初始化摄像头，进入命令交互循环。"""
    # 启动提示灯
    rgb_blink()

    # 初始化摄像头（统一 sensor，RGB565 彩色）
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(10)

    print("== 命令交互模式 ==")
    print("支持命令：TASK1 / TASK2 / SNAPSHOT / TRACK / IRRIGATION，exit 退出。")

    while True:
        # 从串口终端读取一行命令（不带提示符）
        try:
            cmd = input().strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd.lower() in ("exit", "quit"):
            break

        # 分派命令到对应任务；单个任务异常不退出循环
        try:
            dispatch(cmd)
        except Exception as e:
            print(f"CMD_ERR: {e}")


main()