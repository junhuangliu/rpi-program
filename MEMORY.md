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
  - **条码结束符=Tab**：已把扫码枪"后缀"配置成 Tab(0x09)，scanner 按 `\t` 分帧（`-e` 可改，回车模式用 `-e '\n'`），**整条二维码内容原样保留（不做任何分割/规整）**，一个二维码=一条发布；内容里的 `\r`/换行由消费端自行按需 split
  - **解析服务 `/scanner/query_parsed`**：对最近扫码做结构化解析（编号+4行情况→编号+映射拼接，如 `12462213`）；映射=轻微干旱1/一般干旱2/严重干旱3；段数≠5、编号非数字、未知情况均整体返回 `ERROR:<原因>`（对应上位机 `#QRB$`）
- OpenMV N6 摄像头：USB 枚举为 `MicroPython Pyboard Virtual Comm Port`，VID:PID `37c5:1206`，by-id 名含 `MicroPython`；`fun/serial_ports.py::find_openmv_port()` 识别
- OpenMV 通信协议：命令经 REPL（USB VCP，`/\r\n` 结尾）发送，`print()` 回显（`TASK1_OK: xxxxxx` / `TASK1_TIMEOUT: xxxxxx` / `SNAPSHOT_OK` 等）；`send()` 的 `#<payload>$` 帧走 UART3 物理引脚，USB 收不到
- ROS 任务 vs 事件流约定：短任务/一问一答 = Service（OpenMV 用 `openmv_msgs/srv/Command`）；持续事件流 = Topic（扫码 `/scanner/barcode`）；长流程/进度 = Action
- OpenMV 服务包：`~/ros2_ws/src/openmv_msgs`（`srv/Command.srv`），`colcon build` 后须 source `~/ros2_ws/install/setup.bash`；节点 `fun/camera.py` 提供 `/camera/command` service + `/camera/result` 话题
- 上位机桥接：`fun/bridge.py`（节点 `host_bridge`）经 USB 虚拟串口（115200）与上位机通信：
  - 下行 10Hz `#POS,x,y,yaw$`（TF map->base_link；无 SLAM 时 `#POS,no_tf$`）；`-d/--debug` 开关把每帧发送内容打到 rclpy 日志（后台 `/tmp/bridge.log`），用于在无上位机时查看实际发送数据
  - 上行 `#<命令>$` → 命令表 `self.commands` 调服务 → 回传 `#RES,<内容>$`
  - 命令表：TASK1→/camera/command（OpenMV）；QR/QRC→/scanner/query（原样整条）；QRB→/scanner/query_parsed（解析后：编号+4行情况映射，如 12462213）；帧头尾/分隔符是常量（FRAME_HEAD/TAIL/FIELD_SEP），协议确定后改
  - 端口用 `-p` 指定；需 MultiThreadedExecutor（服务阻塞不挡 10Hz）；rclpy Future 无 `timeout_sec` 参数，用轮询 `done()`+超时
- 串口识别加固：`find_port()` 按 by-id 关键词命中**多个**候选（如同型号多块 CP210）时不再猜测，打印全部并返回 None 提示用 `-p` 显式指定；`slam`/`open_lidar` 均支持 `-p/--port` 覆盖串口

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

## 一键控制脚本 control.sh

- 用法：`./control.sh <start|stop|status|logs|scanq|tf|hz|svc> [功能]`
  - `start/stop` 功能可选 `slam|scanner|camera|lidar|bridge|all`（all=slams+scanner+camera）
  - 后台运行、日志 `/tmp/<fn>.log`、PID 记录 `/tmp/rpi-pids/<fn>.pid`
  - `scanq`=调 /scanner/query；`camera <命令>`=调 /camera/command（默认 TASK1，可接 TRACK/SNAPSHOT/IRRIGATION，无服务时自动提示）；`tf`=map->base_link；`hz`=/scan 频率；`svc`=关键服务在线检查
- 实现要点：脚本免 source 环境（内置 set +u 规避 ROS setup 的未定义变量）；停止用 kill+pgrep 精确匹配（避开 pkill 自匹配坑）

服务自测：`ros2 service call /scanner/query std_srvs/srv/Trigger`；`ros2 topic echo /scanner/barcode`。

## 供电约束（重要，影响所有 USB 外设）

- 树莓派 USB 供电能力不足：RPLIDAR C1 电**机启动瞬时电流**会触发 xHCI 总线级 `over-current`（journald 见 `over-current change`）→ **整条 USB 全部外设断电重枚举**，`/dev/ttyUSB*`/`/dev/ttyACM*` 端口号反复洗牌，已挂旧端口的所有节点失效
- 换任意 USB 口无效，根治需：雷达电机独立 5V 供电 / 带独立电源的 USB hub / 换官方规格电源（5V/5A PD）
- 临时缓解：**少接外设**。当前可靠组合 = 雷达 + 扫码枪；接 OpenMV 会增加过流风险
- 外设集体掉线/端口洗牌已是常态事件 → 节点按 by-id 自动识别可跟随端口，但**节点不会自动重连**，掉线后需重启节点（后续可加自动重连容错）

## 排障经验

- **不要用 `pkill -f '关键词'`**：会匹配到执行命令的 bash 自身（cmdline 含同样字符串）导致自杀/挂死。清场用 `kill -9 <PID>` 或避免自匹配
- 后台节点统一启动法：`source <ROS setup> && setsid python3 start.py <fn> > /tmp/<fn>.log 2>&1 < /dev/null &`
- `ros2 service call` 偶发 `rcl node's context is invalid`、或 openmv_msgs 类型报 `The passed service type is invalid`（缺 `source ~/ros2_ws/install/setup.bash`）；兜底用 python rclpy 客户端验证服务往返