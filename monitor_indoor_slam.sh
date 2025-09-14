#!/bin/bash

echo "=== MID360 室内SLAM监控面板 ==="
echo "启动时间: $(date)"
echo

# 检查节点状态
echo "📡 系统节点状态:"
ros2 node list 2>/dev/null | grep -E "(fastlio2|pgo|hba|localizer|livox)" | while read node; do
    echo "  ✅ $node"
done
echo

# 检查话题频率
echo "📈 关键话题频率:"
echo "  FastLIO2里程计: $(timeout 3 ros2 topic hz /fastlio2/lio_odom --window 10 2>/dev/null | head -1 || echo '未运行')"
echo "  点云数据: $(timeout 3 ros2 topic hz /fastlio2/body_cloud --window 5 2>/dev/null | head -1 || echo '未运行')"
echo "  PGO优化: $(timeout 3 ros2 topic hz /pgo/optimized_poses --window 5 2>/dev/null | head -1 || echo '未运行')"
echo

# 检查服务状态
echo "🔧 服务可用性:"
services=("/fastlio2/save_maps" "/pgo/save_maps" "/hba/refine_map" "/localizer/relocalize")
for service in "${services[@]}"; do
    if ros2 service list 2>/dev/null | grep -q "$service"; then
        echo "  ✅ $service"
    else
        echo "  ❌ $service (不可用)"
    fi
done
echo

# 检查数据流状态
echo "🌊 数据流状态:"
if ros2 topic list 2>/dev/null | grep -q "/livox/lidar"; then
    echo "  ✅ 雷达数据流"
else
    echo "  ❌ 雷达数据流 (检查网络连接)"
fi

if ros2 topic list 2>/dev/null | grep -q "/fastlio2/lio_odom"; then
    echo "  ✅ SLAM定位输出"
else
    echo "  ❌ SLAM定位输出"
fi

if ros2 topic list 2>/dev/null | grep -q "/pgo/loop_markers"; then
    echo "  ✅ PGO回环检测"
else
    echo "  ❌ PGO回环检测"
fi
echo

echo "💡 使用 'watch -n 2 ./monitor_indoor_slam.sh' 实现实时监控"
echo "💡 使用 'Ctrl+C' 退出监控"