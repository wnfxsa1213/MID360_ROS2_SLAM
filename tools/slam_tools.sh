#!/bin/bash

# FASTLIO2_ROS2 SLAM工具集合脚本
# 作者: Claude Code Assistant
# 日期: 2025-09-03
# 描述: SLAM系统管理和调试工具集合

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

show_help() {
    echo "========================================="
    echo "  MID360 FASTLIO2_ROS2 SLAM工具集"
    echo "========================================="
    echo ""
    echo -e "${YELLOW}使用方法:${NC}"
    echo "  $0 [命令]"
    echo ""
    echo -e "${YELLOW}可用命令:${NC}"
    echo -e "  ${CYAN}start${NC}     - 启动SLAM系统"
    echo -e "  ${CYAN}stop${NC}      - 停止SLAM系统"
    echo -e "  ${CYAN}status${NC}    - 检查系统状态"
    echo -e "  ${CYAN}restart${NC}   - 重启SLAM系统"
    echo -e "  ${CYAN}monitor${NC}   - 实时监控数据流"
    echo -e "  ${CYAN}topics${NC}    - 显示所有话题"
    echo -e "  ${CYAN}nodes${NC}     - 显示所有节点"
    echo -e "  ${CYAN}build${NC}     - 重新编译系统"
    echo -e "  ${CYAN}clean${NC}     - 清理编译文件"
    echo -e "  ${CYAN}config${NC}    - 显示配置信息"
    echo -e "  ${CYAN}network${NC}   - 检查网络配置"
    echo -e "  ${CYAN}log${NC}       - 查看系统日志"
    echo -e "  ${CYAN}rviz${NC}      - 启动RViz可视化"
    echo -e "  ${CYAN}help${NC}      - 显示此帮助信息"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 start    # 启动SLAM系统"
    echo "  $0 status   # 检查运行状态"
    echo "  $0 monitor  # 监控数据流"
}

# 确保在正确的目录
cd "/home/tianyu/codes/Mid360map" 2>/dev/null || {
    echo -e "${RED}错误: 无法进入SLAM系统目录${NC}"
    exit 1
}

# 加载ROS2环境
load_ros_env() {
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ws_livox/install/setup.bash 2>/dev/null
}

case "$1" in
    "start")
        echo -e "${GREEN}启动SLAM系统...${NC}"
        ./start_slam.sh
        ;;
    
    "stop")
        echo -e "${YELLOW}停止SLAM系统...${NC}"
        ./stop_slam.sh
        ;;
    
    "status")
        ./check_slam.sh
        ;;
    
    "restart")
        echo -e "${YELLOW}重启SLAM系统...${NC}"
        ./stop_slam.sh
        sleep 3
        ./start_slam.sh
        ;;
    
    "monitor")
        echo -e "${CYAN}实时监控数据流 (Ctrl+C退出)...${NC}"
        echo ""
        load_ros_env
        
        echo -e "${YELLOW}监控里程计数据:${NC}"
        timeout 5s ros2 topic echo /fastlio2/lio_odom --once 2>/dev/null || echo "无数据"
        echo ""
        
        echo -e "${YELLOW}监控话题频率:${NC}"
        echo "IMU: $(timeout 3s ros2 topic hz /livox/imu 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        echo "点云: $(timeout 3s ros2 topic hz /livox/lidar 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        echo "里程计: $(timeout 3s ros2 topic hz /fastlio2/lio_odom 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        ;;
    
    "topics")
        echo -e "${CYAN}当前活跃话题:${NC}"
        load_ros_env
        ros2 topic list | sort
        ;;
    
    "nodes")
        echo -e "${CYAN}当前运行节点:${NC}"
        load_ros_env
        ros2 node list | sort
        ;;
    
    "build")
        echo -e "${YELLOW}重新编译SLAM系统...${NC}"
        cd ws_livox
        echo "编译核心包..."
        colcon build --packages-select livox_ros_driver2 fastlio2 interface --cmake-args -DCMAKE_BUILD_TYPE=Release
        ;;
    
    "clean")
        echo -e "${YELLOW}清理编译文件...${NC}"
        cd ws_livox
        rm -rf build install log
        echo "编译文件已清理"
        ;;
    
    "config")
        echo -e "${CYAN}系统配置信息:${NC}"
        echo "================================="
        echo "工作目录: $(pwd)"
        echo "ROS版本: $(ros2 --version 2>/dev/null || echo '未加载')"
        
        CONFIG_FILE="ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
        if [ -f "$CONFIG_FILE" ]; then
            echo ""
            echo "MID360配置:"
            echo "  电脑IP: $(grep -o '192\.168\.1\.[0-9]*' "$CONFIG_FILE" | head -1)"
            echo "  雷达IP: $(grep -o '192\.168\.1\.[0-9]*' "$CONFIG_FILE" | tail -1)"
        fi
        
        FASTLIO_CONFIG="ws_livox/install/fastlio2/share/fastlio2/config/lio.yaml"
        if [ -f "$FASTLIO_CONFIG" ]; then
            echo ""
            echo "FASTLIO2配置:"
            echo "  IMU话题: $(grep 'imu_topic:' "$FASTLIO_CONFIG" | awk '{print $2}')"
            echo "  点云话题: $(grep 'lidar_topic:' "$FASTLIO_CONFIG" | awk '{print $2}')"
            echo "  扫描分辨率: $(grep 'scan_resolution:' "$FASTLIO_CONFIG" | awk '{print $2}')"
        fi
        ;;
    
    "network")
        echo -e "${CYAN}网络配置检查:${NC}"
        echo "================================="
        
        echo "本机网络接口:"
        ip addr show | grep -A 2 "state UP" | grep inet
        
        echo ""
        echo "路由表:"
        ip route | head -5
        
        echo ""
        echo "雷达连接测试:"
        if ping -c 2 -W 2 192.168.1.3 >/dev/null 2>&1; then
            echo -e "192.168.1.3: ${GREEN}在线${NC}"
        else
            echo -e "192.168.1.3: ${RED}离线${NC}"
        fi
        ;;
    
    "log")
        echo -e "${CYAN}查看系统日志:${NC}"
        LOG_DIR="$HOME/.ros/log"
        if [ -d "$LOG_DIR" ]; then
            echo "最新日志目录:"
            ls -lt "$LOG_DIR" | head -3
            
            echo ""
            echo "最新错误信息:"
            find "$LOG_DIR" -name "*.log" -mmin -10 -exec grep -l "ERROR\|FATAL\|Failed" {} \; | head -3 | while read logfile; do
                echo "=== $logfile ==="
                tail -10 "$logfile" | grep -E "ERROR|FATAL|Failed" || echo "无错误信息"
                echo ""
            done
        else
            echo "未找到ROS日志目录"
        fi
        ;;
    
    "rviz")
        echo -e "${CYAN}启动RViz可视化...${NC}"
        load_ros_env
        
        RVIZ_CONFIG="ws_livox/install/fastlio2/share/fastlio2/rviz/fastlio2.rviz"
        if [ -f "$RVIZ_CONFIG" ]; then
            echo "使用FASTLIO2配置文件启动RViz..."
            rviz2 -d "$RVIZ_CONFIG" &
        else
            echo "启动默认RViz..."
            rviz2 &
        fi
        
        echo "RViz已在后台启动"
        ;;
    
    "help"|"--help"|"-h"|"")
        show_help
        ;;
    
    *)
        echo -e "${RED}错误: 未知命令 '$1'${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac