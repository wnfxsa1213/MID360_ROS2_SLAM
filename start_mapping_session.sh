#!/bin/bash

SESSION_ID=$(date +%Y%m%d_%H%M%S)
LOG_DIR="mapping_logs/$SESSION_ID"
mkdir -p "$LOG_DIR"

echo "🗺️ 开始室内建图会话: $SESSION_ID"
echo "📁 日志目录: $LOG_DIR"

# 记录系统状态
echo "📋 记录系统状态..."
ros2 node list > "$LOG_DIR/nodes.txt" 2>/dev/null
ros2 topic list > "$LOG_DIR/topics.txt" 2>/dev/null
ros2 service list > "$LOG_DIR/services.txt" 2>/dev/null

# 记录配置
echo "📝 备份配置文件..."
cp config/slam_master_config.yaml "$LOG_DIR/" 2>/dev/null
mkdir -p "$LOG_DIR/configs"
find ws_livox/src -name "*.yaml" -path "*/config/*" -exec cp {} "$LOG_DIR/configs/" \; 2>/dev/null

# 记录网络状态
echo "🌐 检查网络连接..."
ping -c 3 192.168.1.3 > "$LOG_DIR/network_test.txt" 2>&1

# 开始录制关键话题的rosbag
echo "📹 开始录制关键话题数据..."
ros2 bag record \
    /fastlio2/lio_odom \
    /fastlio2/body_cloud \
    /fastlio2/lio_path \
    /pgo/loop_markers \
    /pgo/optimized_poses \
    /livox/lidar \
    /livox/imu \
    -o "$LOG_DIR/rosbag" &

BAG_PID=$!

# 开始监控系统状态
echo "📊 开始系统监控..."
(
    while true; do
        echo "$(date): $(ros2 topic hz /fastlio2/lio_odom --window 5 2>/dev/null | head -1 || echo 'SLAM停止')" >> "$LOG_DIR/monitoring.log"
        sleep 10
    done
) &
MONITOR_PID=$!

echo "════════════════════════════════════════"
echo "🚀 建图会话已启动！"
echo "📹 数据录制 PID: $BAG_PID"
echo "📊 监控进程 PID: $MONITOR_PID"
echo "📁 所有数据保存在: $LOG_DIR"
echo "════════════════════════════════════════"
echo
echo "💡 建图提示："
echo "  1. 在起始位置保持静止2-3秒"
echo "  2. 沿房间边缘慢速移动 (0.3 m/s)"
echo "  3. 完成一圈后回到起始位置形成回环"
echo "  4. 观察RViz中的轨迹和地图质量"
echo
echo "🛑 按任意键停止建图会话..."
read -n 1

echo
echo "🛑 正在停止建图会话..."

# 停止录制和监控
kill $BAG_PID 2>/dev/null
kill $MONITOR_PID 2>/dev/null
sleep 2

# 保存最终地图
echo "💾 保存地图数据..."
mkdir -p "$LOG_DIR/maps"

# 调用地图保存服务
ros2 service call /fastlio2/save_maps interface/srv/SaveMaps "{file_path: '$LOG_DIR/maps'}" > "$LOG_DIR/fastlio2_save.log" 2>&1
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$LOG_DIR/maps'}" > "$LOG_DIR/pgo_save.log" 2>&1

# 生成会话报告
echo "📄 生成会话报告..."
cat > "$LOG_DIR/session_report.txt" << EOF
MID360 SLAM 建图会话报告
========================

会话ID: $SESSION_ID
开始时间: $(head -1 "$LOG_DIR/monitoring.log" 2>/dev/null || echo "未记录")
结束时间: $(date)

系统状态:
- 运行节点数: $(wc -l < "$LOG_DIR/nodes.txt" 2>/dev/null || echo "0")
- 可用话题数: $(wc -l < "$LOG_DIR/topics.txt" 2>/dev/null || echo "0") 
- 可用服务数: $(wc -l < "$LOG_DIR/services.txt" 2>/dev/null || echo "0")

网络连接:
$(cat "$LOG_DIR/network_test.txt" 2>/dev/null | grep "packet loss" || echo "未测试")

保存的文件:
$(ls -la "$LOG_DIR/maps/" 2>/dev/null || echo "无地图文件保存")

建议后续操作:
1. 使用 python3 tools/advanced_point_cloud_processor.py 处理点云
2. 使用 python3 tools/rviz_map_viewer.py 查看地图质量
3. 分析 rosbag 数据进行算法优化

EOF

echo "🎉 建图会话完成！"
echo "📊 会话报告: $LOG_DIR/session_report.txt"
echo "📁 完整数据: $LOG_DIR/"
echo
echo "📈 后续处理建议："
echo "  查看报告: cat $LOG_DIR/session_report.txt"
echo "  处理地图: python3 tools/advanced_point_cloud_processor.py $LOG_DIR/maps/map.pcd"
echo "  可视化: python3 tools/rviz_map_viewer.py $LOG_DIR/maps/optimized_map.pcd"