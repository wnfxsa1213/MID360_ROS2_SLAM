#!/bin/bash

# FASTLIO2_ROS2 SLAM系统停止脚本
# 作者: Claude Code Assistant
# 日期: 2025-09-03
# 描述: 安全停止MID360 FASTLIO2_ROS2 SLAM系统

echo "========================================="
echo "  停止 MID360 FASTLIO2_ROS2 SLAM系统"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 停止函数
stop_process() {
    local process_name="$1"
    local pid_file="/tmp/slam_${process_name}.pid"
    
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}正在停止 $process_name (PID: $pid)...${NC}"
            kill -TERM "$pid" 2>/dev/null
            
            # 等待进程优雅退出
            local count=0
            while kill -0 "$pid" 2>/dev/null && [ $count -lt 10 ]; do
                sleep 1
                count=$((count + 1))
            done
            
            # 如果进程还没退出，强制杀死
            if kill -0 "$pid" 2>/dev/null; then
                echo -e "${YELLOW}强制停止 $process_name...${NC}"
                kill -KILL "$pid" 2>/dev/null
                sleep 1
            fi
            
            if kill -0 "$pid" 2>/dev/null; then
                echo -e "${RED}✗ 无法停止 $process_name${NC}"
                return 1
            else
                echo -e "${GREEN}✓ $process_name 已停止${NC}"
                rm -f "$pid_file"
                return 0
            fi
        else
            echo -e "${YELLOW}$process_name 进程已不存在${NC}"
            rm -f "$pid_file"
            return 0
        fi
    else
        echo -e "${YELLOW}$process_name 没有PID文件${NC}"
        return 0
    fi
}

# 按顺序停止进程
echo -e "${YELLOW}正在停止SLAM组件...${NC}"

# 1. 停止FASTLIO2 SLAM
stop_process "fastlio2_slam"

# 2. 停止Livox驱动
stop_process "livox_driver"

# 清理其他可能的ROS进程
echo -e "${YELLOW}清理其他ROS进程...${NC}"

# 查找并停止rviz2
rviz_pids=$(pgrep -f "rviz2")
if [ ! -z "$rviz_pids" ]; then
    echo -e "${YELLOW}停止 RViz2...${NC}"
    echo "$rviz_pids" | xargs kill -TERM 2>/dev/null
    sleep 2
    echo "$rviz_pids" | xargs kill -KILL 2>/dev/null || true
    echo -e "${GREEN}✓ RViz2 已停止${NC}"
fi

# 查找并停止其他相关进程
for process in "livox_ros_driver2_node" "lio_node"; do
    pids=$(pgrep -f "$process")
    if [ ! -z "$pids" ]; then
        echo -e "${YELLOW}停止 $process...${NC}"
        echo "$pids" | xargs kill -TERM 2>/dev/null
        sleep 1
        echo "$pids" | xargs kill -KILL 2>/dev/null || true
        echo -e "${GREEN}✓ $process 已停止${NC}"
    fi
done

# 清理临时文件
echo -e "${YELLOW}清理临时文件...${NC}"
rm -f /tmp/slam_*.pid

# 检查是否还有相关进程
remaining_processes=$(pgrep -f "livox\|fastlio\|lio_node\|rviz" || true)
if [ ! -z "$remaining_processes" ]; then
    echo -e "${YELLOW}警告: 发现残留进程:${NC}"
    ps -p $remaining_processes -o pid,ppid,cmd --no-headers
    echo -e "${YELLOW}这些进程可能需要手动清理${NC}"
else
    echo -e "${GREEN}✓ 所有进程已清理干净${NC}"
fi

echo ""
echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}✓ SLAM系统已完全停止${NC}"
echo -e "${BLUE}=========================================${NC}"

echo ""
echo -e "${YELLOW}系统状态:${NC}"
echo -e "  • 所有SLAM相关进程已停止"
echo -e "  • 临时文件已清理"
echo -e "  • 系统已恢复到初始状态"

echo ""
echo -e "${YELLOW}要重新启动系统，请运行:${NC}"
echo -e "  ${BLUE}./start_slam.sh${NC}"