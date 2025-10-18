#!/bin/bash
# SLAM 系统完整状态检查脚本

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo "=========================================="
echo "SLAM 系统状态检查"
echo "=========================================="
echo ""

# 1. 检查节点
echo -e "${BLUE}=== 1. 节点状态 ===${NC}"
echo ""

nodes=$(ros2 node list 2>/dev/null | sort | uniq)

check_node() {
    local node_name=$1
    if echo "$nodes" | grep -q "$node_name"; then
        echo -e "${GREEN}✓${NC} $node_name"
        return 0
    else
        echo -e "${RED}✗${NC} $node_name"
        return 1
    fi
}

check_node "/fastlio2_node"
check_node "/pgo_node"
check_node "/hba_node"
check_node "/localizer_node"
check_node "/optimization_coordinator"

# 2. 检查关键话题
echo ""
echo -e "${BLUE}=== 2. 话题数据流 ===${NC}"
echo ""

check_topic() {
    local topic=$1
    local timeout=${2:-2}

    echo -n "检查 $topic ... "

    if timeout $timeout ros2 topic hz $topic --once --window 5 > /dev/null 2>&1; then
        local hz=$(timeout 2 ros2 topic hz $topic --once --window 5 2>&1 | grep "average rate" | awk '{print $3}')
        echo -e "${GREEN}✓${NC} (~${hz} Hz)"
    else
        echo -e "${YELLOW}⚠${NC} 无数据"
    fi
}

check_topic "/slam/lio_odom"
check_topic "/livox/lidar"
check_topic "/livox/imu"

# 3. 检查服务
echo ""
echo -e "${BLUE}=== 3. 优化服务 ===${NC}"
echo ""

services=$(ros2 service list 2>/dev/null)

check_service() {
    local service=$1
    if echo "$services" | grep -q "$service"; then
        echo -e "${GREEN}✓${NC} $service"
    else
        echo -e "${RED}✗${NC} $service"
    fi
}

check_service "/coordinator/trigger_optimization"
check_service "/pgo/save_maps"
check_service "/hba/refine_map"
check_service "/fastlio2/update_pose"

# 4. 检查协同状态
echo ""
echo -e "${BLUE}=== 4. 协同优化状态 ===${NC}"
echo ""

echo -n "读取协同状态... "
coop_status=$(timeout 3 ros2 topic echo /fastlio2/cooperation_status std_msgs/msg/Float64MultiArray --once 2>&1 || echo "timeout")

if echo "$coop_status" | grep -q "data:"; then
    echo -e "${GREEN}✓${NC}"

    # 提取关键数据
    feedback_count=$(echo "$coop_status" | grep -A 1 "data:" | tail -n 1 | grep -oP '\- \K[0-9.]+' | head -n 1 | cut -d'.' -f1)

    if [ -n "$feedback_count" ]; then
        echo "  - 反馈计数: $feedback_count"
        if [ "$feedback_count" -gt 0 ]; then
            echo -e "    ${GREEN}✓ PGO 反馈正在工作${NC}"
        else
            echo -e "    ${YELLOW}⚠ 尚未检测到反馈（可能还未出现回环）${NC}"
        fi
    fi
else
    echo -e "${YELLOW}⚠${NC} 无法读取"
fi

# 5. 检查回环检测
echo ""
echo -e "${BLUE}=== 5. 回环检测 ===${NC}"
echo ""

echo -n "检查回环标记... "
loop_markers=$(timeout 3 ros2 topic echo /slam/loop_markers --once 2>&1 || echo "timeout")

if echo "$loop_markers" | grep -q "markers:"; then
    marker_count=$(echo "$loop_markers" | grep -c "id:" || echo "0")
    echo -e "${GREEN}✓${NC} 检测到 $marker_count 个回环"
else
    echo -e "${YELLOW}⚠${NC} 尚未检测到回环（需要更多数据或绕圈场景）"
fi

# 6. 系统性能
echo ""
echo -e "${BLUE}=== 6. 系统性能 ===${NC}"
echo ""

perf=$(timeout 3 ros2 topic echo /slam/performance_metrics --once 2>&1 || echo "timeout")

if echo "$perf" | grep -q "total_time"; then
    echo -e "${GREEN}✓${NC} 性能指标正在发布"

    # 提取关键指标
    total_time=$(echo "$perf" | grep "total_time:" | awk '{print $2}')
    if [ -n "$total_time" ]; then
        echo "  - 总处理时间: ${total_time}s"
    fi
else
    echo -e "${YELLOW}⚠${NC} 性能指标不可用"
fi

# 7. 协调器日志
echo ""
echo -e "${BLUE}=== 7. 协调器最新日志 ===${NC}"
echo ""

echo "最近 5 条协调器日志:"
ros2 topic echo /rosout --once 2>/dev/null | grep -A 5 "optimization_coordinator" | head -10 || echo "无可用日志"

# 总结
echo ""
echo "=========================================="
echo -e "${CYAN}状态总结${NC}"
echo "=========================================="

node_count=$(echo "$nodes" | wc -l)
key_nodes=$(echo "$nodes" | grep -E "(fastlio2|pgo|hba|optimization_coordinator)" | wc -l)

echo "节点统计: $key_nodes/5 核心节点运行中"

if [ "$key_nodes" -eq 5 ]; then
    echo -e "${GREEN}✓ 所有核心节点正常运行${NC}"
    echo ""
    echo "系统状态: ${GREEN}健康${NC}"
    echo ""
    echo "PGO 和 HBA 正在后台工作："
    echo "  - PGO: 持续监听 /slam/lio_odom，寻找回环"
    echo "  - HBA: 等待服务调用进行地图精细化"
    echo "  - 协调器: 监控漂移，自动触发优化"
    echo ""
    echo "如何触发优化："
    echo "  1. 等待系统检测到回环（需要绕圈场景）"
    echo "  2. 手动触发: ros2 service call /coordinator/trigger_optimization interface/srv/TriggerOptimization \"{optimization_type: 'pgo'}\""
    echo "  3. 保存地图: ./tools/slam_tools.sh save pgo"
else
    echo -e "${YELLOW}⚠ 部分节点未运行${NC}"
    echo ""
    echo "缺失的节点："
    for node in "/fastlio2_node" "/pgo_node" "/hba_node" "/localizer_node" "/optimization_coordinator"; do
        if ! echo "$nodes" | grep -q "$node"; then
            echo "  - $node"
        fi
    done
fi

echo ""
echo "=========================================="
