#!/bin/bash

# 动态过滤器测试脚本
# 用于验证动态对象过滤器的功能和性能

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

echo -e "${CYAN}==============================================${NC}"
echo -e "${CYAN}     动态对象过滤器功能验证测试${NC}"
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
    sleep 3
    echo -e "${GREEN}✅ 所有进程已停止${NC}"
}

# 检查配置文件
check_configs() {
    echo -e "${YELLOW}检查配置文件...${NC}"

    # 检查主配置文件
    if [ ! -f "$PROJECT_ROOT/config/master_config.yaml" ]; then
        echo -e "${RED}❌ 主配置文件不存在${NC}"
        exit 1
    fi

    # 检查localizer配置
    if [ ! -f "$WS_ROOT/src/localizer/config/localizer.yaml" ]; then
        echo -e "${RED}❌ Localizer配置文件不存在${NC}"
        exit 1
    fi

    # 检查动态过滤器是否启用
    if grep -q "enable: true" "$WS_ROOT/src/localizer/config/localizer.yaml"; then
        echo -e "${GREEN}✅ 动态过滤器已启用${NC}"
    else
        echo -e "${RED}❌ 动态过滤器未启用${NC}"
        exit 1
    fi

    echo -e "${GREEN}✅ 配置文件检查完成${NC}"
}

# 生成模拟点云数据
generate_test_data() {
    echo -e "${YELLOW}生成模拟测试数据...${NC}"

    # 使用修复后的测试数据发布器
    cp /tmp/test_pointcloud_fixed.py /tmp/test_pointcloud.py
    chmod +x /tmp/test_pointcloud.py
    echo -e "${GREEN}✅ 测试数据生成器已准备${NC}"
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
        ros2 launch fastlio2 cooperative_slam_system.launch.py
        exec bash
    " &

    sleep 5

    # 启动RViz
    echo -e "${BLUE}启动RViz可视化...${NC}"
    gnome-terminal --tab --title="RViz" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        rviz2 -d src/fastlio2/rviz/enhanced_fastlio2.rviz
        exec bash
    " &

    sleep 3

    echo -e "${GREEN}✅ SLAM系统启动完成${NC}"
}

# 启动测试数据发布器
start_test_publisher() {
    echo -e "${YELLOW}启动测试数据发布器...${NC}"

    gnome-terminal --tab --title="Test Data" -- bash -c "
        source /opt/ros/humble/setup.bash
        python3 /tmp/test_pointcloud.py
        exec bash
    " &

    sleep 2
    echo -e "${GREEN}✅ 测试数据发布器已启动${NC}"
}

# 启动Python可视化工具
start_visualization_tool() {
    echo -e "${YELLOW}启动动态过滤器可视化工具...${NC}"

    gnome-terminal --tab --title="Filter Visualization" -- bash -c "
        cd '$PROJECT_ROOT'
        source /opt/ros/humble/setup.bash
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
    sleep 10

    echo -e "${BLUE}检查话题状态:${NC}"
    echo "输入话题："
    timeout 3s ros2 topic list | grep -E "(livox|slam)" | head -5 || echo "  话题检查超时"

    echo -e "\n${BLUE}检查节点状态:${NC}"
    timeout 3s ros2 node list | grep -E "(lio|localizer)" || echo "  节点检查超时"

    echo -e "\n${BLUE}检查动态过滤器数据流:${NC}"
    echo "尝试获取过滤统计数据..."
    timeout 5s ros2 topic echo /localizer/filter_stats --once 2>/dev/null || echo "  暂无过滤统计数据"
}

# 主函数
main() {
    case "${1:-full}" in
        "stop")
            stop_all_slam
            ;;
        "config")
            load_ros_env
            check_configs
            ;;
        "data")
            generate_test_data
            ;;
        "slam")
            load_ros_env
            start_slam_system
            ;;
        "test")
            load_ros_env
            start_test_publisher
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
            check_configs
            generate_test_data
            start_slam_system
            start_test_publisher
            start_visualization_tool
            monitor_system

            echo -e "\n${GREEN}==============================================${NC}"
            echo -e "${GREEN}  动态过滤器测试环境启动完成！${NC}"
            echo -e "${GREEN}==============================================${NC}"
            echo -e "${YELLOW}请检查以下窗口：${NC}"
            echo -e "  1. SLAM System - FastLIO2和Localizer"
            echo -e "  2. RViz - 3D可视化界面"
            echo -e "  3. Test Data - 模拟数据发布器"
            echo -e "  4. Filter Visualization - 过滤器可视化工具"
            echo ""
            echo -e "${CYAN}使用 './test_dynamic_filter.sh stop' 停止所有进程${NC}"
            ;;
        *)
            echo "用法: $0 [stop|config|data|slam|test|viz|monitor|full]"
            echo "  stop    - 停止所有SLAM进程"
            echo "  config  - 检查配置文件"
            echo "  data    - 生成测试数据"
            echo "  slam    - 启动SLAM系统"
            echo "  test    - 启动测试数据发布器"
            echo "  viz     - 启动可视化工具"
            echo "  monitor - 监控系统状态"
            echo "  full    - 完整测试流程 (默认)"
            ;;
    esac
}

main "$@"