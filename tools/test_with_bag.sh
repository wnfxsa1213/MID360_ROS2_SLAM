#!/bin/bash

# ROS2 Bag包动态过滤器测试脚本
# 使用录制的bag包测试动态对象过滤器功能

set -e

# 配置参数（可通过环境变量覆盖）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DEFAULT_BAG_DIR="$PROJECT_ROOT/data/rosbags"
BAG_PATH="${BAG_PATH:-}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$PROJECT_ROOT/ws_livox/src/localizer/rviz/localizer.rviz}"

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --bag)
        BAG_PATH="$2"; shift 2 ;;
      --rviz)
        RVIZ_CONFIG="$2"; shift 2 ;;
      --)
        shift; break ;;
      *)
        break ;;
    esac
  done
  REMAINING_ARGS="$@"
}

ensure_bag_path() {
  if [[ -n "$BAG_PATH" ]]; then
    return
  fi
  if [[ -d "$DEFAULT_BAG_DIR" ]]; then
    latest=$(find "$DEFAULT_BAG_DIR" -maxdepth 1 -type d -name "*" ! -path "$DEFAULT_BAG_DIR" -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n1 | cut -d' ' -f2-)
    if [[ -n "$latest" ]]; then
      BAG_PATH="$latest"
      return
    fi
  fi
  log_error "未找到 bag 路径，请使用 --bag <path> 或设置 BAG_PATH 环境变量"
  exit 1
}

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[信息]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

log_error() {
    echo -e "${RED}[错误]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[步骤]${NC} $1"
}

# 检查依赖
check_dependencies() {
    log_step "检查测试环境..."

    ensure_bag_path

    # 检查bag文件
    if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
        log_error "未找到bag包文件: $BAG_PATH"
        exit 1
    fi

    # 检查RViz配置
    if [ ! -f "$RVIZ_CONFIG" ]; then
        log_error "未找到RViz配置文件: $RVIZ_CONFIG"
        exit 1
    fi

    # 设置ROS2环境
    if [ -f "$PROJECT_ROOT/ws_livox/install/setup.bash" ]; then
        source "$PROJECT_ROOT/ws_livox/install/setup.bash"
        log_info "ROS2环境已设置"
    else
        log_error "未找到ROS2环境设置文件"
        exit 1
    fi

    log_info "环境检查通过"
}

# 显示使用说明
show_usage() {
    cat << EOF
动态过滤器Bag包测试脚本

用法: $0 [选项]

选项:
    play        - 播放bag包并启动localizer节点
    rviz        - 启动RViz可视化
    monitor     - 监控过滤器统计信息
    full        - 完整测试(播放+可视化+监控)
    clean       - 清理测试进程
    help        - 显示此帮助信息

测试话题:
    /livox/lidar           - 原始LiDAR数据
    /livox/imu             - IMU数据
    /localizer/filtered_cloud - 过滤后点云
    /localizer/dynamic_points - 动态点云
    /localizer/filter_stats   - 过滤统计信息

EOF
}

# 启动localizer节点
start_localizer() {
    log_step "启动localizer节点..."

    # 检查节点是否已经运行
    if ros2 node list | grep -q "localizer_node"; then
        log_warn "localizer节点已在运行"
        return 0
    fi

    # 启动localizer节点
    ros2 run localizer localizer_node --ros-args \
        -p config_path:="$PROJECT_ROOT/ws_livox/src/localizer/config/localizer.yaml" \
        > /tmp/localizer_test.log 2>&1 &

    local localizer_pid=$!
    sleep 3

    # 检查节点是否成功启动
    if ros2 node list | grep -q "localizer_node"; then
        log_info "localizer节点启动成功 (PID: $localizer_pid)"
        echo $localizer_pid > /tmp/localizer_test.pid
    else
        log_error "localizer节点启动失败"
        cat /tmp/localizer_test.log
        exit 1
    fi
}

# 播放bag包
play_bag() {
    log_step "播放bag包数据..."

    # 重映射话题以匹配localizer期望的输入
    ros2 bag play "$BAG_PATH" \
        --remap /livox/lidar:=/livox/lidar \
        --remap /livox/imu:=/livox/imu \
        --remap /slam/body_cloud:=/fastlio2/body_cloud \
        --remap /slam/lio_odom:=/fastlio2/lio_odom \
        --rate 0.5 \
        > /tmp/bag_play.log 2>&1 &

    local bag_pid=$!
    echo $bag_pid > /tmp/bag_play.pid
    log_info "bag包播放已启动 (PID: $bag_pid, 播放速度: 0.5x)"
}

# 启动RViz
start_rviz() {
    log_step "启动RViz可视化..."

    if pgrep -f "rviz2" > /dev/null; then
        log_warn "RViz已在运行"
        return 0
    fi

    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz_test.log 2>&1 &
    local rviz_pid=$!
    echo $rviz_pid > /tmp/rviz_test.pid
    log_info "RViz启动成功 (PID: $rviz_pid)"
}

# 监控过滤器统计
monitor_stats() {
    log_step "监控过滤器统计信息..."
    log_info "按 Ctrl+C 停止监控"

    echo "时间戳,总点数,动态点数,静态点数,过滤比例,处理时间(ms),内存使用(MB)" > /tmp/filter_stats.csv

    ros2 topic echo /localizer/filter_stats --csv >> /tmp/filter_stats.csv &
    local monitor_pid=$!
    echo $monitor_pid > /tmp/monitor.pid

    # 实时显示统计信息
    while true; do
        if ros2 topic hz /localizer/filter_stats --window 10 > /tmp/stats_hz.tmp 2>&1; then
            clear
            echo "=== 动态过滤器实时统计 ==="
            echo "时间: $(date '+%H:%M:%S')"
            echo ""

            # 显示话题频率
            echo "--- 话题频率 ---"
            cat /tmp/stats_hz.tmp | tail -n 3
            echo ""

            # 显示最新统计数据
            echo "--- 最新过滤统计 ---"
            ros2 topic echo /localizer/filter_stats --once --no-arr 2>/dev/null | \
                grep -E "(total_points|dynamic_points|static_points|filter_ratio|processing_time_ms)" | \
                head -5
            echo ""

            # 显示节点状态
            echo "--- 节点状态 ---"
            if ros2 node list | grep -q "localizer_node"; then
                echo "✓ localizer节点: 运行中"
            else
                echo "✗ localizer节点: 未运行"
            fi

            echo ""
            echo "按 Ctrl+C 停止监控..."

        else
            log_warn "等待过滤器统计数据..."
        fi

        sleep 2
    done
}

# 清理测试进程
cleanup() {
    log_step "清理测试进程..."

    # 清理各种进程
    for pid_file in /tmp/localizer_test.pid /tmp/bag_play.pid /tmp/rviz_test.pid /tmp/monitor.pid; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                kill "$pid"
                log_info "已终止进程 PID: $pid"
            fi
            rm -f "$pid_file"
        fi
    done

    # 清理临时文件
    rm -f /tmp/localizer_test.log /tmp/bag_play.log /tmp/rviz_test.log /tmp/stats_hz.tmp

    log_info "清理完成"
}

# 完整测试流程
full_test() {
    log_step "开始完整测试流程..."

    # 清理之前的进程
    cleanup

    # 启动各个组件
    start_localizer
    sleep 2

    start_rviz
    sleep 3

    play_bag
    sleep 2

    log_info "所有组件已启动，开始监控..."
    log_info "测试指南:"
    log_info "1. 在RViz中检查以下话题的可视化效果:"
    log_info "   - /fastlio2/body_cloud (原始点云 - 白色)"
    log_info "   - /localizer/filtered_cloud (过滤后点云 - 绿色)"
    log_info "   - /localizer/dynamic_points (动态点云 - 橙色)"
    log_info "2. 观察过滤统计数据的变化"
    log_info "3. 按 Ctrl+C 结束测试"

    # 开始监控
    monitor_stats
}

# 信号处理
trap cleanup EXIT INT TERM

# 主程序
main() {
    parse_args "$@"
    set -- $REMAINING_ARGS
    case "${1:-help}" in
        "play")
            check_dependencies
            start_localizer
            play_bag
            log_info "Bag包播放已启动，使用 '$0 clean' 清理进程"
            ;;
        "rviz")
            check_dependencies
            start_rviz
            ;;
        "monitor")
            check_dependencies
            monitor_stats
            ;;
        "full")
            check_dependencies
            full_test
            ;;
        "clean")
            cleanup
            ;;
        "help"|*)
            show_usage
            ;;
    esac
}

main "$@"
