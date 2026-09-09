# MEMORY.md - 项目长期记忆

本文件存放跨会话的项目关键事实、决策与偏好。此文件为项目内自管理，路径：`<项目根>/MEMORY.md`。

## 项目信息

- 项目：rpi-program（树莓派编程项目，位于 ~/Desktop/rpi-program）
- 技术栈：ROS（机器人操作系统）+ Python
- 身份约定：协助者 wcq，用户 ljy

## 外设硬件

- RPLIDAR_C1M1 激光雷达
- OpenMV 摄像头
- 扫码模块
- USB 外接串口等

## 技术知识

- GitHub 加速：使用 dev-sidecar（sudo dss start），验证命令详情见项目 note.txt
- 日志体系：项目内自管理，daily 日志在 `instruction/daily/YYYY-MM-DD.md`，规则见 `instruction/LOG.md`
- 雷达串口权限：已配置 `/etc/udev/rules.d/rplidar.rules`（匹配 10c4:ea60，`MODE:=0777`）。若 `/dev/ttyUSB0` 权限异常（非 777），重插 USB 或执行 `sudo udevadm control --reload-rules && sudo udevadm trigger` 使其生效
- rviz2 在树莓派上须用软件渲染：树莓派 5 v3d 驱动仅支持 OpenGL 3.1，rviz2 地图 shader 链接失败会崩溃。解决：rviz2 节点设 `additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"}`（llvmpipe，OpenGL 4.5）。已在 `fun/slam_launch.py` 中配置
- rviz 视图缩放：`TopDownOrtho` 视图里**真正控制缩放的是 `Scale` 参数**（值越大显示越大，默认 50），而非 `Distance`（相机距离）。`slam.rviz` 已把 `Scale` 设为 200
- 系统无 `python` 命令，Python 脚本须用 `python3` 运行（如 `python3 start.py slam`）
- USB 串口稳定识别：`/dev/ttyUSBx`/`/dev/ttyACMx` 编号随插拔顺序变化，不可据名区分设备。
  用 `/dev/serial/by-id/` 稳定链接识别（名称含厂商/产品/序列号，不受插拔顺序影响）：
  - 雷达 RPLIDAR：by-id 含 `Silicon_Labs`/`CP210`，PID `10c4:ea60`
  - 扫码枪 BF SCAN：by-id 含 `BF_SCAN`，VID `9901`（USB CDC 模式为 `9901:0303`）
  - 统一工具：`fun/serial_ports.py`（find_lidar_port / find_scanner_port），缺失时回退 sysfs VID:PID 过滤
- 扫码枪串口参数：USB CDC（/dev/ttyACM*）、GBK 编码、115200 波特率；接收节点 `fun/scanner.py`（rclpy + pyserial，发布 `/scanner/barcode`，另提供 `/scanner/query` 服务返回最近条码）
- OpenMV N6 摄像头：USB 枚举为 `MicroPython Pyboard Virtual Comm Port`，VID:PID `37c5:1206`，by-id 名含 `MicroPython`；`fun/serial_ports.py::find_openmv_port()` 识别
- OpenMV 通信协议：命令经 REPL（USB VCP，`/\r\n` 结尾）发送，`print()` 回显（`TASK1_OK: xxxxxx` / `TASK1_TIMEOUT: xxxxxx` / `SNAPSHOT_OK` 等）；`send()` 的 `#<payload>$` 帧走 UART3 物理引脚，USB 收不到
- ROS 任务 vs 事件流约定：短任务/一问一答 = Service（OpenMV 用 `openmv_msgs/srv/Command`）；持续事件流 = Topic（扫码 `/scanner/barcode`）；长流程/进度 = Action
- OpenMV 服务包：`~/ros2_ws/src/openmv_msgs`（`srv/Command.srv`），`colcon build` 后须 source `~/ros2_ws/install/setup.bash`；节点 `fun/camera.py` 提供 `/camera/command` service + `/camera/result` 话题
- 上位机桥接：`fun/bridge.py`（节点 `host_bridge`）经 USB 虚拟串口（115200）与上位机通信：
  - 下行 10Hz `#POS,x,y,yaw$`（TF map->base_link；无 SLAM 时 `#POS,no_tf$`）
  - 上行 `#<命令>$` → 命令表 `self.commands` 调服务 → 回传 `#RES,<内容>$`
  - 命令表占位：TASK1→/camera/command、QR→/scanner/query；帧头尾/分隔符是常量（FRAME_HEAD/TAIL/FIELD_SEP），协议确定后改
  - 端口用 `-p` 指定；需 MultiThreadedExecutor（服务阻塞不挡 10Hz）；rclpy Future 无 `timeout_sec` 参数，用轮询 `done()`+超时

## 约定

- 回答默认使用中文（除非用户明确要求其他语言）
- 定义函数必须在之前注释功能和用法
- 未明确要求时不主动执行 git 提交/推送
- 用户的行为规则写在文件中（AGENTS.md、note.txt 等），遵循这些约定

## 启动与使用

每次先加载 ROS 环境：`source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`，再 `cd ~/Desktop/rpi-program`。

| 功能 | 命令 | 说明 |
|---|---|---|
| 开雷达 | `python3 start.py lidar` | 识别雷达→开 rviz |
| 建图 | `python3 start.py slam` | 5 节点手持建图，串口自动识别 |
| 扫码 | `python3 start.py scanner` | 广播 `/scanner/barcode` + 服务 `/scanner/query` |
| OpenMV 任务 | `python3 start.py camera` | 服务 `/camera/command`(openmv_msgs)，调用例：`ros2 service call /camera/command openmv_msgs/srv/Command "{command: 'TASK1'}"` |
| 桥接上位机 | `python3 start.py bridge -p /dev/ttyACMx` | `-p` 指定上位机 USB 串口 |

服务自测：`ros2 service call /scanner/query std_srvs/srv/Trigger`；`ros2 topic echo /scanner/barcode`。