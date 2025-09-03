#!/bin/bash

# FASTLIO2_ROS2 SLAM系统启动脚本
# 作者: Claude Code Assistant
# 日期: 2025-09-03
# 描述: 启动MID360 LiDAR + FASTLIO2 SLAM系统

echo "========================================="
echo "  MID360 FASTLIO2_ROS2 SLAM启动脚本"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE_DIR="/home/tianyu/codes/Mid360map/ws_livox"

# 检查工作目录是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}错误: 工作目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

cd "$WORKSPACE_DIR"

# 检查是否已编译
if [ ! -d "install" ]; then
    echo -e "${RED}错误: 工作空间未编译，请先运行 colcon build${NC}"
    exit 1
fi

echo -e "${BLUE}当前工作目录: $PWD${NC}"

# 加载ROS2环境
echo -e "${YELLOW}正在加载ROS2环境...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查网络配置
echo -e "${YELLOW}检查网络配置...${NC}"
EXPECTED_IP="192.168.1.50"
CURRENT_IP=$(ip route get 192.168.1.74 2>/dev/null | grep -Po '(?<=src )(\S+)' || echo "未配置")

if [ "$CURRENT_IP" != "$EXPECTED_IP" ]; then
    echo -e "${YELLOW}警告: 当前IP ($CURRENT_IP) 与预期IP ($EXPECTED_IP) 不匹配${NC}"
    echo -e "${YELLOW}请确保网络已正确配置${NC}"
fi

# 检查雷达连接
echo -e "${YELLOW}检查雷达连接...${NC}"
if ping -c 1 -W 2 192.168.1.3 >/dev/null 2>&1; then
    echo -e "${GREEN}✓ MID360雷达连接正常 (192.168.1.3)${NC}"
else
    echo -e "${YELLOW}⚠ 无法连接到MID360雷达 (192.168.1.3)${NC}"
    echo -e "${YELLOW}  系统仍将启动，但可能无法接收数据${NC}"
fi

# 创建日志目录
LOG_DIR="$HOME/.ros/log/slam_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}启动SLAM系统...${NC}"
echo -e "${BLUE}=========================================${NC}"

# 启动函数
start_node() {
    local node_name="$1"
    local launch_command="$2"
    local log_file="$3"
    
    echo -e "${YELLOW}启动 $node_name...${NC}"
    eval "$launch_command" > "$log_file" 2>&1 &
    local pid=$!
    echo "$pid" > "/tmp/slam_${node_name}.pid"
    sleep 2
    
    if kill -0 $pid 2>/dev/null; then
        echo -e "${GREEN}✓ $node_name 启动成功 (PID: $pid)${NC}"
        return 0
    else
        echo -e "${RED}✗ $node_name 启动失败${NC}"
        return 1
    fi
}

# 1. 启动Livox驱动
start_node "livox_driver" \
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py" \
    "$LOG_DIR/livox_driver.log"

if [ $? -ne 0 ]; then
    echo -e "${RED}Livox驱动启动失败，退出${NC}"
    exit 1
fi

# 等待驱动初始化
echo -e "${YELLOW}等待驱动初始化...${NC}"
sleep 3

# 2. 启动FASTLIO2 SLAM
start_node "fastlio2_slam" \
    "ros2 launch fastlio2 lio_launch.py" \
    "$LOG_DIR/fastlio2_slam.log"

if [ $? -ne 0 ]; then
    echo -e "${RED}FASTLIO2 SLAM启动失败${NC}"
    # 清理已启动的进程
    if [ -f "/tmp/slam_livox_driver.pid" ]; then
        kill $(cat /tmp/slam_livox_driver.pid) 2>/dev/null
        rm -f /tmp/slam_livox_driver.pid
    fi
    exit 1
fi

# 等待系统稳定
echo -e "${YELLOW}等待系统稳定...${NC}"
sleep 5

echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}✓ SLAM系统启动完成！${NC}"
echo -e "${BLUE}=========================================${NC}"

# 显示系统信息
echo -e "${YELLOW}系统信息:${NC}"
echo -e "  工作目录: $WORKSPACE_DIR"
echo -e "  日志目录: $LOG_DIR"
echo -e "  雷达IP:   192.168.1.3"
echo -e "  电脑IP:   192.168.1.50"

echo ""
echo -e "${YELLOW}运行中的节点:${NC}"
echo -e "  • Livox Driver  (PID: $(cat /tmp/slam_livox_driver.pid 2>/dev/null || echo '未知'))"
echo -e "  • FASTLIO2 SLAM (PID: $(cat /tmp/slam_fastlio2_slam.pid 2>/dev/null || echo '未知'))"
echo -e "  • RViz2 可视化"

echo ""
echo -e "${YELLOW}主要话题:${NC}"
echo -e "  输入: /livox/lidar, /livox/imu"
echo -e "  输出: /fastlio2/lio_odom, /fastlio2/lio_path, /fastlio2/world_cloud"

echo ""
echo -e "${YELLOW}常用命令:${NC}"
echo -e "  查看话题: ${BLUE}ros2 topic list${NC}"
echo -e "  监控里程计: ${BLUE}ros2 topic echo /fastlio2/lio_odom${NC}"
echo -e "  停止系统: ${BLUE}./stop_slam.sh${NC}"

echo ""
echo -e "${GREEN}系统已就绪，可以开始SLAM建图！${NC}"
echo -e "${YELLOW}按 Ctrl+C 或运行 ./stop_slam.sh 来停止系统${NC}"

# 保持脚本运行并监控
trap 'echo -e "\n${YELLOW}正在停止SLAM系统...${NC}"; /home/tianyu/codes/Mid360map/stop_slam.sh; exit' INT TERM

# 监控循环
while true; do
    sleep 5
    # 检查进程是否还在运行
    for pid_file in /tmp/slam_*.pid; do
        if [ -f "$pid_file" ]; then
            pid=$(cat "$pid_file")
            if ! kill -0 "$pid" 2>/dev/null; then
                node_name=$(basename "$pid_file" .pid | sed 's/slam_//')
                echo -e "${RED}警告: $node_name 进程已停止${NC}"
                rm -f "$pid_file"
            fi
        fi
    done
done