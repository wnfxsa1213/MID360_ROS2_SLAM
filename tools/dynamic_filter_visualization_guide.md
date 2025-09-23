# 动态对象过滤器可视化工具使用指南

## 概述

本工具套件为动态对象过滤器提供完整的可视化功能，包括实时点云对比、性能监控、参数调优和数据录制回放等功能。

## 功能特性

### 核心功能
- ✅ **实时点云对比**: 同时显示原始点云与过滤后点云
- ✅ **统计图表**: 实时显示过滤统计信息和性能指标
- ✅ **交互式参数调优**: 实时调整过滤器参数并观察效果
- ✅ **3D可视化**: 基于Open3D的三维点云显示
- ✅ **性能监控**: 处理时间、内存使用等性能指标
- ✅ **数据录制**: 记录过滤过程数据用于离线分析
- ✅ **数据回放**: 回放已录制的数据进行分析

### 支持的ROS2话题
- `/localizer/raw_cloud` - 原始点云
- `/localizer/filtered_cloud` - 过滤后点云
- `/localizer/dynamic_cloud` - 检测到的动态点云
- `/localizer/filter_stats` - 过滤器统计信息

## 安装依赖

### 必需依赖
```bash
# Python基础依赖
pip install numpy matplotlib tkinter

# ROS2依赖
sudo apt install ros-humble-sensor-msgs ros-humble-rclpy

# 可选依赖（3D可视化）
pip install open3d
```

### 构建interface包
```bash
cd /home/tianyu/codes/Mid360map/ws_livox
colcon build --packages-select interface
source install/setup.bash
```

## 使用方法

### 1. 启动可视化工具

#### 方法1: 独立运行
```bash
cd /home/tianyu/codes/Mid360map/tools
python3 visualize_dynamic_filter.py
```

#### 方法2: 与SLAM系统一起启动
```bash
# 首先启动SLAM系统
cd /home/tianyu/codes/Mid360map
./tools/slam_tools.sh start_full_system

# 然后在新终端启动可视化工具
cd /home/tianyu/codes/Mid360map/tools
python3 visualize_dynamic_filter.py
```

### 2. RViz可视化

启动RViz查看点云分层显示：

```bash
cd /home/tianyu/codes/Mid360map/ws_livox
source install/setup.bash
rviz2 -d src/localizer/rviz/localizer.rviz
```

在RViz中你将看到：
- **PointCloud2 (白色)**: 原始SLAM点云
- **Map Cloud (彩色)**: 地图点云
- **Raw_PointCloud (红色)**: 动态过滤器输入点云
- **Filtered_PointCloud (绿色)**: 过滤后静态点云
- **Dynamic_PointCloud (黄色)**: 检测到的动态点云

### 3. 界面说明

#### 主界面布局
```
┌─────────────────┬─────────────────────────────────┐
│   控制面板      │         图表显示区域           │
│                 │                                │
│ • 连接状态      │ • 实时统计图表                │
│ • 录制控制      │ • 性能监控图表                │
│ • 回放控制      │ • 点云对比图表                │
│ • 参数调优      │                                │
│ • 实时统计      │                                │
│ • 3D可视化      │                                │
└─────────────────┴─────────────────────────────────┘
```

#### 控制面板功能

1. **连接状态**
   - 显示ROS2连接状态
   - 绿色: 已连接，红色: 未连接

2. **数据录制**
   - `开始录制/停止录制`: 切换录制状态
   - `加载数据`: 加载已录制的.pkl文件
   - `导出数据`: 导出为JSON格式

3. **数据回放**
   - `开始回放/停止回放`: 控制数据回放
   - 进度条: 显示回放进度
   - 速度滑条: 调整回放速度(0.1x - 5.0x)

4. **参数调优**
   - 运动阈值: 检测运动的最小位移
   - 历史帧数: 用于时间一致性分析的帧数
   - 稳定性阈值: 判断点稳定的阈值
   - 搜索半径: 邻域搜索半径
   - 其他几何特征参数

5. **实时统计**
   - 总点数、动态点数、静态点数
   - 过滤比例
   - 处理时间和内存使用

#### 图表显示区域

1. **实时统计标签页**
   - 点云统计趋势图
   - 过滤比例变化图
   - 处理时间趋势图
   - 内存使用趋势图

2. **性能监控标签页**
   - 处理时间趋势(带平均值线)
   - 内存使用趋势(带最大值线)

3. **点云对比标签页**
   - 原始点云X-Y投影图(红色)
   - 过滤后点云X-Y投影图(绿色)

### 4. 3D可视化

点击"打开3D视图"按钮启动Open3D可视化窗口:

- **红色点云**: 原始点云
- **绿色点云**: 过滤后点云(略微偏移便于区分)
- 支持鼠标交互: 旋转、缩放、平移

### 5. 数据分析工作流

#### 参数调优工作流
1. 启动系统并开始可视化
2. 调整参数滑条观察实时效果
3. 监控过滤比例和处理时间
4. 记录最优参数组合

#### 数据录制与分析
1. 点击"开始录制"收集数据
2. 运行测试场景(有动态对象)
3. 停止录制并保存数据
4. 使用回放功能分析录制数据
5. 导出JSON数据进行进一步分析

## 故障排除

### 常见问题

1. **ROS2连接失败**
   ```bash
   # 检查ROS2环境
   echo $ROS_DOMAIN_ID
   ros2 topic list

   # 重新source环境
   source /opt/ros/humble/setup.bash
   source /home/tianyu/codes/Mid360map/ws_livox/install/setup.bash
   ```

2. **缺少Python依赖**
   ```bash
   # 安装缺失的包
   pip install matplotlib numpy tkinter
   pip install open3d  # 可选，用于3D可视化
   ```

3. **话题无数据**
   - 检查localizer节点是否正常运行
   - 验证话题名称是否正确
   - 检查QoS设置是否兼容

4. **性能问题**
   - 降低点云显示采样率
   - 关闭不需要的图表更新
   - 减少历史数据缓存大小

### 调试模式

如果遇到问题，可以在脚本中启用调试输出:

```python
# 在visualize_dynamic_filter.py中找到
debug_mode = True  # 设置为True启用调试
```

## 配置文件

### ROS2话题配置

在localizer节点中需要发布以下话题:

```cpp
// 在localizer_node.cpp中添加发布器
raw_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/localizer/raw_cloud", qos_profile);

filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/localizer/filtered_cloud", qos_profile);

dynamic_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/localizer/dynamic_cloud", qos_profile);

filter_stats_pub_ = this->create_publisher<interface::msg::FilterStats>(
    "/localizer/filter_stats", qos_profile);
```

### 可视化工具配置

可在脚本开头修改默认配置:

```python
@dataclass
class FilterConfig:
    motion_threshold: float = 0.1      # 运动阈值
    history_size: int = 5              # 历史帧数
    stability_threshold: float = 0.8   # 稳定性阈值
    # ... 其他参数
```

## 性能优化建议

1. **降低更新频率**: 减少图表更新间隔
2. **点云采样**: 在可视化时对大点云进行采样
3. **内存管理**: 限制历史数据缓存大小
4. **QoS设置**: 使用BEST_EFFORT提高性能

## 扩展功能

### 添加新的可视化指标

1. 在`VisualizationData`类中添加新字段
2. 在ROS2回调中更新新字段
3. 在图表更新函数中添加新图表

### 自定义参数

1. 在`FilterConfig`中添加新参数
2. 在参数调优面板中添加对应控件
3. 实现参数更新回调

## 技术细节

### 架构设计
- **数据层**: ROS2订阅器接收实时数据
- **处理层**: 数据预处理和性能监控
- **显示层**: Matplotlib图表和Open3D 3D可视化
- **控制层**: Tkinter GUI交互界面

### 线程安全
- 使用线程锁保护共享数据
- ROS2在独立线程中运行
- 3D可视化在独立线程中更新

### 数据格式
- 录制数据: Pickle格式(.pkl)
- 导出数据: JSON格式(.json)
- 点云数据: NumPy数组格式

## 版本信息

- **版本**: 1.0.0
- **作者**: SLAM系统开发团队
- **最后更新**: 2024年
- **兼容性**: ROS2 Humble, Python 3.8+

## 相关文档

- [动态对象过滤器算法文档](../docs/dynamic_object_filter.md)
- [SLAM系统整体架构](../docs/system_architecture.md)
- [性能调优指南](../docs/performance_tuning.md)