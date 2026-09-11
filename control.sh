#!/usr/bin/env bash
# rpi-program 一键启动/控制脚本
#
# 用法: ./control.sh <命令> [功能]
#   start  <slam|scanner|camera|lidar|bridge|all>  后台启动对应功能（日志 /tmp/<fn>.log）
#   stop   <同上|all>                               停止对应功能（含清除子进程）
#   status                                         查看全部功能运行状态
#   logs   <slam|scanner|camera|lidar|bridge>       tail -f 查看日志
#   scanq                                         调 /scanner/query 服务
#   qrc                                             调 /scanner/query（原样整条，同 scanq）
#   qrb                                             调 /scanner/query_parsed（解析后：编号+4情况映射）
#   camera <命令>                                 调 /camera/command（默认 TASK1，可接 TRACK/SNAPSHOT/IRRIGATION）
#   tf                                            查看 TF map->base_link 坐标
#   hz                                            查看 /scan 发布频率
#   oc                                            调 /obstacle/check 服务（前方矩形障碍检测）
#   svc                                           列出关键服务是否在线
#
# 说明:
#   - all = slam + scanner + camera（当前供电安全组合；OpenMV 未插时 camera 会自行退出）
#   - 障碍检测节点随 slam 一起启停（由 slam_launch.py 拉起）
#   - 用电受限：雷达电机启动可能触发 USB 过流，建议外设少接
#   - 停止使用 kill + 精确进程匹配，避免 pkill 自匹配坑
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_DIR="/tmp/rpi-pids"
mkdir -p "$PID_DIR"
FUNCS_ALL=(slam scanner camera lidar bridge)

# 用法提示
usage() {
    echo "用法: $0 <命令> [功能]"
    echo
    echo "命令:"
    echo "  start  <slam|scanner|camera|lidar|bridge|all>  后台启动功能（all=slam+scanner+camera）"
    echo "  stop   <同上|all>                               停止功能"
    echo "  status                                          查看全部功能状态"
    echo "  logs   <功能>                                    tail -f 日志"
    echo "  scanq                                           调用 /scanner/query"
    echo "  qrc                                             调用 /scanner/query（原样整条）"
    echo "  qrb                                             调用 /scanner/query_parsed（编号+4情况映射，如 12462213）"
    echo "  camera <命令>                                    调用 /camera/command（默认 TASK1）"
    echo "  tf                                               查看 TF map->base_link"
    echo "  hz                                               查看 /scan 频率"
    echo "  oc                                               调用 /obstacle/check（前方矩形障碍检测）"
    echo "  svc                                              列出关键服务"
}

# 加载 ROS 环境（新 bash 必须 source 一次；setup.bash 会引用未定义变量，先关 set -u）
env_pre() {
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
    cd "$ROOT"
    set -u
}

# 取指定功能当前 PID（pid 文件优先，其次按命令行兜底）
get_pid() { # fn
    local fn="$1" pid=""
    if [ -f "$PID_DIR/$fn.pid" ]; then
        pid=$(cat "$PID_DIR/$fn.pid")
    fi
    if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
        pid=$(pgrep -f "python3 start.py $fn" | head -1)
    fi
    echo "$pid"
}

# 启动单个功能
start_one() { # fn
    local fn="$1" pid
    pid=$(get_pid "$fn")
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        echo "[已在运行] $fn  pid=$pid"
        return
    fi
    if [ ! -f "$ROOT/start.py" ]; then
        echo "[错误] 未找到 $ROOT/start.py"
        return 1
    fi
    # 在独立会话中启动并记录 PID
    ( cd "$ROOT"
      set +u
      source /opt/ros/jazzy/setup.bash
      source "$HOME/ros2_ws/install/setup.bash"
      set -u
      setsid python3 start.py "$fn" > "/tmp/$fn.log" 2>&1 < /dev/null &
      echo $! > "$PID_DIR/$fn.pid"
    )
    echo "[已启动] $fn  日志=/tmp/$fn.log  pid=$(cat "$PID_DIR/$fn.pid")"
}

# 清理某功能的子进程（本脚本 cmdline 不含节点名，pkill 安全）
cleanup() { # fn
    case "$1" in
        slam|lidar)
            pkill -9 -f "rplidar_node" 2>/dev/null
            pkill -9 -f "laser_scan_matcher" 2>/dev/null
            pkill -9 -f "async_slam_toolbox_node" 2>/dev/null
            pkill -9 -f "static_transform_publisher" 2>/dev/null
            pkill -9 -f "fun/obstacle.py" 2>/dev/null
            pkill -9 -f "rviz2 -d" 2>/dev/null
            ;;
    esac
}

# 停止单个功能（含子进程清理）
stop_one() { # fn
    local fn="$1" pid
    pid=$(get_pid "$fn")
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null
        echo "[已停止] $fn  pid=$pid"
    else
        echo "[无进程] $fn"
    fi
    rm -f "$PID_DIR/$fn.pid"
    cleanup "$fn"
}

# 状态总览
cmd_status() {
    printf "%-8s %-9s %s\n" "功能" "PID" "状态"
    for fn in "${FUNCS_ALL[@]}"; do
        local pid et
        pid=$(get_pid "$fn")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            et=$(ps -o etimes= -p "$pid" 2>/dev/null | tr -d ' ')
            printf "%-8s %-9s RUNNING  %ss\n" "$fn" "$pid" "${et:-?}"
        else
            printf "%-8s %-9s STOPPED\n" "$fn" "-"
        fi
    done
}

# 查看日志
cmd_logs() { # fn
    tail -f "/tmp/$1.log"
}

# 快捷：查询扫码
cmd_scanq() {
    env_pre
    timeout 8 ros2 service call /scanner/query std_srvs/srv/Trigger 2>&1 | tail -5
}

# 快捷：查询扫码（原样整条）
cmd_qrc() {
    env_pre
    timeout 8 ros2 service call /scanner/query std_srvs/srv/Trigger 2>&1 | tail -5
}

# 快捷：查询扫码（解析后：编号+4情况映射）
cmd_qrb() {
    env_pre
    timeout 8 ros2 service call /scanner/query_parsed std_srvs/srv/Trigger 2>&1 | tail -5
}

# 快捷：调用摄像头命令（TASK1/SNAPSHOT/TRACK/IRRIGATION，默认 TASK1）
cmd_camera() { # [命令名]
    env_pre
    local sub="${1:-TASK1}"
    if ! timeout 5 ros2 service list 2>/dev/null | grep -qx "/camera/command" >/dev/null; then
        echo "[提示] /camera/command 服务未运行：请插上 OpenMV 并启动 ./control.sh start camera"
        return 1
    fi
    echo "调用 /camera/command 命令: $sub"
    timeout 25 ros2 service call /camera/command openmv_msgs/srv/Command "{command: '$sub'}" 2>&1 | tail -6
}

# 快捷：当前坐标
cmd_tf() {
    env_pre
    timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E "Translation|in RPY \(degree\)" | head -4
}

# 快捷：雷达频率
cmd_hz() {
    env_pre
    timeout 8 ros2 topic hz /scan 2>&1 | tail -2
}

# 快捷：前方矩形障碍检测
cmd_oc() {
    env_pre
    timeout 8 ros2 service call /obstacle/check rpi_msgs/srv/ObstacleCheck "{}" 2>&1 | tail -12
}

# 快捷：关键服务在线检查
cmd_svc() {
    env_pre
    echo "关键服务:"
    local s alive
    alive=$(timeout 6 ros2 service list 2>/dev/null)
    for s in /scanner/query /scanner/query_parsed /camera/command /obstacle/check /slam_toolbox/get_state /slam_toolbox/save_map; do
        if echo "$alive" | grep -qx "$s" >/dev/null; then
            echo "  [OK]    $s"
        else
            echo "  [N/A]   $s"
        fi
    done
}

cmd="${1:-}"
shift 2>/dev/null || shift || true

case "$cmd" in
    start)
        if [ "${1:-}" = "all" ]; then
            for fn in slam scanner camera; do start_one "$fn"; done
        elif [ -n "${1:-}" ]; then
            start_one "$1"
        else
            usage
        fi
        ;;
    stop)
        if [ "${1:-}" = "all" ]; then
            for fn in bridge lidar camera scanner slam; do stop_one "$fn"; done
        elif [ -n "${1:-}" ]; then
            stop_one "$1"
        else
            usage
        fi
        ;;
    status) cmd_status ;;
    logs) [ -n "${1:-}" ] && cmd_logs "$1" || usage ;;
    scanq) cmd_scanq ;;
    qrc) cmd_qrc ;;
    qrb) cmd_qrb ;;
    camera) cmd_camera "${1:-}" ;;
    tf) cmd_tf ;;
    hz) cmd_hz ;;
    oc) cmd_oc ;;
    svc) cmd_svc ;;
    *) usage ;;
esac