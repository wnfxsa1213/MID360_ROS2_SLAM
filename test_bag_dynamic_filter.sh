#!/bin/bash

# 使用真实bag文件测试动态过滤器
# 基于真实Livox雷达数据验证过滤效果

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 项目根目录
PROJECT_ROOT="/home/tianyu/codes/Mid360map"
WS_ROOT="$PROJECT_ROOT/ws_livox"
BAG_PATH="$PROJECT_ROOT/data/rosbags/slam_data_20250918_104927"

echo -e "${CYAN}==============================================${NC}"
echo -e "${CYAN}    真实数据动态过滤器测试${NC}"
echo -e "${CYAN}==============================================${NC}"

# 加载ROS环境
load_ros_env() {
    echo -e "${YELLOW}加载ROS环境...${NC}"
    source /opt/ros/humble/setup.bash
    if [ -f "$WS_ROOT/install/setup.bash" ]; then
        source "$WS_ROOT/install/setup.bash"
        echo -e "${GREEN}✅ ROS环境加载完成${NC}"
    else
        echo -e "${RED}❌ 工作空间未编译，请先运行编译${NC}"
        exit 1
    fi
}

# 停止所有SLAM进程
stop_all_slam() {
    echo -e "${YELLOW}停止所有SLAM进程...${NC}"
    pkill -f "lio_node" 2>/dev/null || true
    pkill -f "localizer_node" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "ros2 bag" 2>/dev/null || true
    sleep 3
    echo -e "${GREEN}✅ 所有进程已停止${NC}"
}

# 检查bag文件
check_bag_file() {
    echo -e "${YELLOW}检查bag文件...${NC}"

    if [ ! -d "$BAG_PATH" ]; then
        echo -e "${RED}❌ Bag文件目录不存在: $BAG_PATH${NC}"
        exit 1
    fi

    if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
        echo -e "${RED}❌ Bag元数据文件不存在${NC}"
        exit 1
    fi

    echo -e "${GREEN}✅ Bag文件检查完成${NC}"
    echo -e "${BLUE}Bag信息:${NC}"
    echo -e "  路径: $BAG_PATH"
    echo -e "  大小: $(du -sh $BAG_PATH | cut -f1)"

    # 显示bag信息
    cd "$WS_ROOT"
    timeout 10s ros2 bag info "$BAG_PATH" | head -20 || echo "  (部分信息显示)"
}

# 启动SLAM系统
start_slam_system() {
    echo -e "${YELLOW}启动SLAM系统...${NC}"

    cd "$WS_ROOT"

    # 启动FastLIO2 + Localizer系统
    echo -e "${BLUE}启动FastLIO2和Localizer...${NC}"
    gnome-terminal --tab --title="SLAM System" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '启动SLAM系统...'
        ros2 launch fastlio2 cooperative_slam_system.launch.py
        exec bash
    " &

    sleep 8

    # 启动RViz
    echo -e "${BLUE}启动RViz可视化...${NC}"
    gnome-terminal --tab --title="RViz" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '启动RViz...'
        rviz2 -d src/fastlio2/rviz/enhanced_fastlio2.rviz
        exec bash
    " &

    sleep 5

    echo -e "${GREEN}✅ SLAM系统启动完成${NC}"
}

# 启动bag回放
start_bag_playback() {
    echo -e "${YELLOW}启动bag文件回放...${NC}"

    cd "$WS_ROOT"

    gnome-terminal --tab --title="Bag Playback" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '开始回放bag文件...'
        echo 'Bag路径: $BAG_PATH'
        ros2 bag play '$BAG_PATH' --clock 200
        echo 'Bag回放完成'
        exec bash
    " &

    sleep 3
    echo -e "${GREEN}✅ Bag回放已启动${NC}"
}

# 启动Python可视化工具
start_visualization_tool() {
    echo -e "${YELLOW}启动动态过滤器可视化工具...${NC}"

    gnome-terminal --tab --title="Filter Visualization" -- bash -c "
        cd '$PROJECT_ROOT'
        source /opt/ros/humble/setup.bash
        echo '启动可视化工具...'
        python3 tools/visualize_dynamic_filter.py
        exec bash
    " &

    sleep 2
    echo -e "${GREEN}✅ 可视化工具已启动${NC}"
}

# 监控系统状态
monitor_system() {
    echo -e "${CYAN}==============================================${NC}"
    echo -e "${CYAN}         系统运行状态监控${NC}"
    echo -e "${CYAN}==============================================${NC}"

    load_ros_env

    echo -e "${YELLOW}等待系统稳定...${NC}"
    sleep 15

    echo -e "${BLUE}检查节点状态:${NC}"
    timeout 5s ros2 node list | grep -E "(lio|localizer|bag)" || echo "  节点检查超时"

    echo -e "\n${BLUE}检查话题状态:${NC}"
    timeout 5s ros2 topic list | grep -E "(livox|slam|localizer)" | sort || echo "  话题检查超时"

    echo -e "\n${BLUE}检查数据流频率:${NC}"
    echo -e "输入数据:"
    timeout 5s ros2 topic hz /livox/lidar || echo "  /livox/lidar: 无数据"

    echo -e "\nSLAM输出:"
    timeout 5s ros2 topic hz /slam/body_cloud || echo "  /slam/body_cloud: 无数据"

    echo -e "\n过滤器输出:"
    timeout 5s ros2 topic hz /localizer/filtered_cloud || echo "  /localizer/filtered_cloud: 无数据"

    echo -e "\n${BLUE}尝试获取过滤统计数据:${NC}"
    timeout 8s ros2 topic echo /localizer/filter_stats --once || echo "  暂无过滤统计数据"
}

# 显示使用说明
show_instructions() {
    echo -e "\n${GREEN}==============================================${NC}"
    echo -e "${GREEN}  动态过滤器测试环境启动完成！${NC}"
    echo -e "${GREEN}==============================================${NC}"
    echo -e "${YELLOW}测试说明：${NC}"
    echo -e "  1. SLAM System - FastLIO2和Localizer处理真实数据"
    echo -e "  2. RViz - 3D可视化界面，查看过滤效果"
    echo -e "  3. Bag Playback - 真实Livox数据回放"
    echo -e "  4. Filter Visualization - 过滤器统计可视化(中文标签)"
    echo ""
    echo -e "${CYAN}观察要点：${NC}"
    echo -e "  - RViz中的点云颜色变化"
    echo -e "  - Python GUI中的过滤统计图表"
    echo -e "  - 过滤比例和处理时间"
    echo -e "  - 动态对象检测效果"
    echo ""
    echo -e "${CYAN}使用 './test_bag_dynamic_filter.sh stop' 停止所有进程${NC}"
}

# 主函数
main() {
    case "${1:-full}" in
        "stop")
            stop_all_slam
            ;;
        "check")
            load_ros_env
            check_bag_file
            ;;
        "slam")
            load_ros_env
            start_slam_system
            ;;
        "bag")
            load_ros_env
            start_bag_playback
            ;;
        "viz")
            start_visualization_tool
            ;;
        "monitor")
            monitor_system
            ;;
        "full")
            stop_all_slam
            load_ros_env
            check_bag_file
            start_slam_system
            start_bag_playback
            start_visualization_tool
            monitor_system
            show_instructions
            ;;
        *)
            echo "用法: $0 [stop|check|slam|bag|viz|monitor|full]"
            echo "  stop    - 停止所有SLAM进程"
            echo "  check   - 检查bag文件信息"
            echo "  slam    - 启动SLAM系统"
            echo "  bag     - 启动bag回放"
            echo "  viz     - 启动可视化工具"
            echo "  monitor - 监控系统状态"
            echo "  full    - 完整测试流程 (默认)"
            ;;
    esac
}

main "$@"