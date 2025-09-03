#!/bin/bash

# FASTLIO2_ROS2 SLAM系统状态检查脚本
# 作者: Claude Code Assistant
# 日期: 2025-09-03
# 描述: 检查MID360 FASTLIO2_ROS2 SLAM系统运行状态

echo "========================================="
echo "  MID360 FASTLIO2_ROS2 SLAM状态检查"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE_DIR="/home/tianyu/codes/Mid360map/ws_livox"

cd "$WORKSPACE_DIR" 2>/dev/null || {
    echo -e "${RED}错误: 无法进入工作目录 $WORKSPACE_DIR${NC}"
    exit 1
}

# 加载环境
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

echo -e "${BLUE}1. 网络连接状态${NC}"
echo "================================="
echo -n "电脑IP地址: "
CURRENT_IP=$(ip route get 192.168.1.74 2>/dev/null | grep -Po '(?<=src )(\S+)' || echo "未获取")
if [ "$CURRENT_IP" = "192.168.1.50" ]; then
    echo -e "${GREEN}$CURRENT_IP ✓${NC}"
else
    echo -e "${YELLOW}$CURRENT_IP (预期: 192.168.1.50)${NC}"
fi

echo -n "雷达连接: "
if ping -c 1 -W 2 192.168.1.3 >/dev/null 2>&1; then
    echo -e "${GREEN}192.168.1.3 在线 ✓${NC}"
else
    echo -e "${RED}192.168.1.3 离线 ✗${NC}"
fi

echo ""
echo -e "${BLUE}2. ROS2进程状态${NC}"
echo "================================="

# 检查ROS进程
check_process() {
    local process_name="$1"
    local display_name="$2"
    local pids=$(pgrep -f "$process_name")
    
    echo -n "$display_name: "
    if [ ! -z "$pids" ]; then
        echo -e "${GREEN}运行中 (PID: $pids) ✓${NC}"
        return 0
    else
        echo -e "${RED}未运行 ✗${NC}"
        return 1
    fi
}

check_process "livox_ros_driver2_node" "Livox驱动"
check_process "lio_node" "FASTLIO2 SLAM"
check_process "rviz2" "RViz2可视化"

echo ""
echo -e "${BLUE}3. ROS话题状态${NC}"
echo "================================="

# 检查话题
check_topic() {
    local topic_name="$1"
    local display_name="$2"
    
    echo -n "$display_name: "
    if ros2 topic info "$topic_name" >/dev/null 2>&1; then
        local pub_count=$(ros2 topic info "$topic_name" 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
        local sub_count=$(ros2 topic info "$topic_name" 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
        echo -e "${GREEN}活跃 (发布者:$pub_count, 订阅者:$sub_count) ✓${NC}"
        return 0
    else
        echo -e "${RED}不存在 ✗${NC}"
        return 1
    fi
}

check_topic "/livox/lidar" "点云输入"
check_topic "/livox/imu" "IMU输入"
check_topic "/fastlio2/lio_odom" "SLAM里程计"
check_topic "/fastlio2/lio_path" "机器人轨迹"
check_topic "/fastlio2/world_cloud" "世界地图点云"

echo ""
echo -e "${BLUE}4. 数据流检查${NC}"
echo "================================="

# 检查数据频率
check_data_rate() {
    local topic_name="$1"
    local display_name="$2"
    local timeout="$3"
    
    echo -n "$display_name 数据率: "
    
    # 使用timeout命令避免无限等待
    local hz_output=$(timeout ${timeout}s ros2 topic hz "$topic_name" 2>/dev/null | grep "average rate:" | tail -1)
    
    if [ ! -z "$hz_output" ]; then
        local rate=$(echo "$hz_output" | awk '{print $3}')
        echo -e "${GREEN}${rate} Hz ✓${NC}"
        return 0
    else
        echo -e "${YELLOW}无数据或超时${NC}"
        return 1
    fi
}

echo "检查数据流 (最多等待5秒)..."
check_data_rate "/livox/imu" "IMU" 5
check_data_rate "/fastlio2/lio_odom" "里程计" 5

echo ""
echo -e "${BLUE}5. 系统资源使用${NC}"
echo "================================="

# CPU和内存使用
echo "ROS进程资源使用:"
ps aux | grep -E "(livox|lio_node|rviz)" | grep -v grep | while read line; do
    echo "  $(echo "$line" | awk '{printf "%-20s CPU:%s%% MEM:%s%%\n", $11, $3, $4}')"
done

echo ""
echo -e "${BLUE}6. 配置文件状态${NC}"
echo "================================="

CONFIG_FILE="$WORKSPACE_DIR/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
echo -n "配置文件: "
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}存在 ✓${NC}"
    echo "  路径: $CONFIG_FILE"
    
    # 检查IP配置
    if grep -q "192.168.1.50" "$CONFIG_FILE" && grep -q "192.168.1.74" "$CONFIG_FILE"; then
        echo -e "  IP配置: ${GREEN}正确 ✓${NC}"
    else
        echo -e "  IP配置: ${YELLOW}需要检查${NC}"
    fi
else
    echo -e "${RED}不存在 ✗${NC}"
fi

echo ""
echo -e "${BLUE}7. 快速诊断${NC}"
echo "================================="

# 综合诊断
all_good=true

# 检查关键进程
if ! pgrep -f "livox_ros_driver2_node" >/dev/null; then
    echo -e "${RED}⚠ Livox驱动未运行${NC}"
    all_good=false
fi

if ! pgrep -f "lio_node" >/dev/null; then
    echo -e "${RED}⚠ FASTLIO2 SLAM未运行${NC}"
    all_good=false
fi

# 检查关键话题
if ! ros2 topic info "/fastlio2/lio_odom" >/dev/null 2>&1; then
    echo -e "${RED}⚠ SLAM里程计话题不可用${NC}"
    all_good=false
fi

if $all_good; then
    echo -e "${GREEN}✓ 系统运行正常，可以进行SLAM建图${NC}"
else
    echo -e "${YELLOW}⚠ 系统存在问题，建议检查上述状态${NC}"
fi

echo ""
echo -e "${YELLOW}使用建议:${NC}"
echo "  • 启动系统: ${BLUE}./start_slam.sh${NC}"
echo "  • 停止系统: ${BLUE}./stop_slam.sh${NC}"
echo "  • 查看话题: ${BLUE}ros2 topic list${NC}"
echo "  • 监控里程计: ${BLUE}ros2 topic echo /fastlio2/lio_odom${NC}"