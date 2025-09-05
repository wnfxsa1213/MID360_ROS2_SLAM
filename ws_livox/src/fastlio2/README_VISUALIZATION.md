# FAST-LIO2 增强可视化功能

本文档说明了为FAST-LIO2系统添加的增强可视化功能，提供更丰富的实时监控和调试信息。

## 新增功能

### 1. IMU姿态可视化
- **话题**: `/fastlio2/imu_pose_marker`
- **类型**: `visualization_msgs/Marker`
- **功能**: 实时显示IMU的三维姿态，使用红色箭头表示当前朝向
- **配置**: 箭头长度0.5m，更新频率与SLAM同步

### 2. 实时性能监控
- **话题**: `/fastlio2/performance_metrics`
- **类型**: `std_msgs/Float64MultiArray`
- **数据结构**:
  - `data[0]`: 处理时间 (毫秒)
  - `data[1]`: 点云大小 (点数)
  - `data[2]`: IMU缓冲区大小
  - `data[3]`: 激光雷达缓冲区大小

### 3. 系统诊断信息
- **话题**: `/fastlio2/diagnostics`
- **类型**: `diagnostic_msgs/DiagnosticArray`
- **包含信息**:
  - 系统整体健康状态
  - 处理延迟评估
  - 点云质量评估
  - 缓冲区状态监控

### 4. 增强的RViz配置
- **文件**: `rviz/enhanced_fastlio2.rviz`
- **改进**:
  - 更清晰的点云显示设置
  - IMU姿态箭头显示
  - 优化的颜色方案
  - 更好的轨迹可视化

## 使用方法

### 启动增强可视化
```bash
# 使用新的启动文件
ros2 launch fastlio2 enhanced_visualization.launch.py

# 或者手动启动各个组件
ros2 run fastlio2 lio_node --ros-args -p config_path:=path/to/config.yaml
ros2 run rviz2 rviz2 -d src/fastlio2/rviz/enhanced_fastlio2.rviz
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### 监控系统状态

#### 1. RViz可视化
- 启动RViz查看实时建图效果
- 红色箭头显示IMU姿态
- 绿色轨迹显示SLAM路径
- 点云颜色表示高度信息

#### 2. 性能监控
```bash
# 查看实时性能数据
ros2 topic echo /fastlio2/performance_metrics

# 使用rqt_plot绘制性能曲线
ros2 run rqt_plot rqt_plot /fastlio2/performance_metrics/data[0]
```

#### 3. 系统诊断
```bash
# 查看诊断信息
ros2 topic echo /fastlio2/diagnostics

# 或使用图形化界面
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## 状态指示器

### 系统状态等级
- **OK (绿色)**: 处理时间 < 20ms，系统运行正常
- **WARN (黄色)**: 处理时间 20-50ms，性能有所下降
- **ERROR (红色)**: 处理时间 > 50ms，性能严重下降

### 点云质量评级
- **Excellent**: > 15000 点/帧
- **Good**: 10000-15000 点/帧
- **Fair**: 5000-10000 点/帧  
- **Poor**: < 5000 点/帧

## 故障排除

### 常见问题

1. **IMU姿态箭头不显示**
   - 检查话题是否发布：`ros2 topic list | grep imu_pose_marker`
   - 确认RViz中Marker显示已启用

2. **性能数据异常**
   - 检查系统负载：`htop`
   - 确认网络延迟：`ping 雷达IP地址`
   - 检查磁盘IO：`iotop`

3. **诊断信息显示错误**
   - 重启节点：`ros2 node kill /fastlio2_node`
   - 检查配置文件：确认参数设置正确
   - 查看日志：`ros2 log info`

### 性能优化建议

1. **处理延迟过高时**：
   - 降低点云发布频率
   - 增加滤波参数
   - 检查CPU使用率

2. **点云质量不佳时**：
   - 检查雷达安装位置
   - 确认环境遮挡情况  
   - 调整雷达扫描参数

3. **内存使用过高时**：
   - 减小地图范围参数
   - 增加点云下采样率
   - 限制历史数据缓存

## 自定义配置

### 修改可视化参数
编辑 `enhanced_fastlio2.rviz` 文件：
- 调整点云大小和颜色
- 修改IMU箭头尺寸
- 设置轨迹显示长度

### 添加新的监控指标
在 `publishPerformanceMetrics()` 函数中添加：
```cpp
metrics.data[4] = custom_metric_value;
```

### 自定义诊断阈值
在 `publishDiagnostics()` 函数中修改：
```cpp
if (processing_time_ms < YOUR_THRESHOLD) {
    // 自定义阈值逻辑
}
```

## 话题列表

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|------|
| `/fastlio2/body_cloud` | PointCloud2 | ~50Hz | 机体坐标系点云 |
| `/fastlio2/world_cloud` | PointCloud2 | ~50Hz | 世界坐标系点云 |
| `/fastlio2/lio_path` | Path | ~50Hz | SLAM轨迹 |
| `/fastlio2/lio_odom` | Odometry | ~50Hz | 里程计信息 |
| `/fastlio2/imu_pose_marker` | Marker | ~50Hz | IMU姿态可视化 |
| `/fastlio2/performance_metrics` | Float64MultiArray | ~50Hz | 性能监控数据 |
| `/fastlio2/diagnostics` | DiagnosticArray | ~50Hz | 系统诊断信息 |

通过这些增强的可视化功能，用户可以更好地监控和调试FAST-LIO2系统，及时发现和解决潜在问题。