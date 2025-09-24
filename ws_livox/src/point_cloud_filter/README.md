# Point Cloud Filter Bridge

## 概述

点云过滤桥接节点（Point Cloud Filter Bridge）是一个ROS2 C++节点，用于解决动态过滤器数据流问题。该节点将现有的DynamicObjectFilter算法集成到激光雷达数据流中，确保FastLIO2接收到的是经过动态对象过滤的静态点云数据。

## 功能特性

- **实时动态对象过滤**: 应用时间一致性和几何特征分析来识别和过滤动态对象
- **格式兼容**: 保持livox_ros_driver2的CustomMsg格式，确保与现有系统完全兼容  
- **高性能**: 优化的PCL点云转换和处理流程，支持实时10Hz激光雷达数据
- **可配置参数**: 丰富的过滤器参数配置，适应不同场景需求
- **调试支持**: 提供调试点云发布和过滤统计信息
- **频率控制**: 支持处理频率限制，避免系统过载

## 系统架构

```
/livox/lidar (CustomMsg) 
    ↓
[Point Cloud Filter Bridge]
    ↓ 
/livox/lidar_filtered (CustomMsg)
    ↓
[FastLIO2 SLAM]
```

## 文件结构

```
point_cloud_filter/
├── CMakeLists.txt                 # 构建配置
├── package.xml                    # 包依赖描述  
├── config/
│   └── point_cloud_filter.yaml   # 配置文件
├── launch/
│   └── point_cloud_filter_launch.py  # 启动文件
├── include/
│   └── point_cloud_filter_bridge.h   # 节点头文件
└── src/
    ├── point_cloud_filter_bridge.cpp     # 节点实现
    └── point_cloud_filter_bridge_node.cpp # 主函数
```

## 编译安装

### 依赖项

- ROS2 (Humble 或更新版本)
- PCL (Point Cloud Library)
- Eigen3
- yaml-cpp
- livox_ros_driver2
- localizer (包含DynamicObjectFilter)
- interface (包含FilterStats消息)

### 编译

```bash
cd /path/to/your/workspace
colcon build --packages-select point_cloud_filter --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 使用方法

### 基本启动

```bash
# 启动点云过滤桥接节点
ros2 launch point_cloud_filter point_cloud_filter_launch.py
```

### 自定义参数启动

```bash  
# 启用调试模式
ros2 launch point_cloud_filter point_cloud_filter_launch.py debug_enabled:=true

# 自定义话题名称
ros2 launch point_cloud_filter point_cloud_filter_launch.py \
    input_topic:=/custom/lidar \
    output_topic:=/custom/lidar_filtered

# 自定义处理频率
ros2 launch point_cloud_filter point_cloud_filter_launch.py \
    max_processing_hz:=5.0
```

### 与现有系统集成

要将该节点集成到现有的SLAM系统中：

1. **启动过滤桥接节点**:
   ```bash
   ros2 launch point_cloud_filter point_cloud_filter_launch.py
   ```

2. **修改FastLIO2配置**，将其订阅话题从`/livox/lidar`改为`/livox/lidar_filtered`

3. **启动完整系统**:
   ```bash
   # 启动livox驱动
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   
   # 启动点云过滤器
   ros2 launch point_cloud_filter point_cloud_filter_launch.py
   
   # 启动FastLIO2 (修改后的配置)
   ros2 launch fastlio2 lio_launch.py
   ```

## 配置参数

### ROS话题配置

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `input_topic` | `/livox/lidar` | 输入话题：原始livox数据 |
| `output_topic` | `/livox/lidar_filtered` | 输出话题：过滤后数据 |
| `debug_topic` | `/point_cloud_filter/debug_cloud` | 调试话题：PCL格式点云 |
| `stats_topic` | `/point_cloud_filter/filter_stats` | 统计话题：过滤统计信息 |

### 节点控制参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `debug_enabled` | `false` | 是否启用调试模式 |
| `publish_stats` | `true` | 是否发布过滤统计信息 |
| `max_processing_hz` | `10.0` | 最大处理频率限制（Hz） |

### 动态过滤器参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `motion_threshold` | `0.05` | 运动阈值(m) |
| `history_size` | `3` | 历史帧数 |
| `stability_threshold` | `0.8` | 稳定性阈值 [0,1] |
| `max_time_diff` | `1.0` | 最大时间差(s) |
| `search_radius` | `0.2` | 邻域搜索半径(m) |
| `min_neighbors` | `8` | 最小邻居点数 |
| `normal_consistency_thresh` | `0.8` | 法向量一致性阈值 [0,1] |
| `density_ratio_thresh` | `0.5` | 密度比值阈值 [0,1] |
| `downsample_ratio` | `2` | 降采样比例 |
| `max_points_per_frame` | `50000` | 每帧最大点数 |
| `voxel_size_base` | `0.01` | 基础体素大小(m) |

## 监控和调试

### 查看统计信息

```bash
# 监听过滤统计信息  
ros2 topic echo /point_cloud_filter/filter_stats
```

### 可视化调试点云

```bash
# 启用调试模式
ros2 launch point_cloud_filter point_cloud_filter_launch.py debug_enabled:=true

# 在RViz中查看调试点云
ros2 run rviz2 rviz2
# 添加PointCloud2显示，话题设置为 /point_cloud_filter/debug_cloud
```

### 性能监控

节点会定期输出性能统计信息，包括：
- 处理时间
- 输入/输出点数
- 过滤比例
- 平均帧率

## 常见问题

### Q: 编译时找不到localizer包
A: 确保已经编译了localizer包，并且在同一个工作空间中。

### Q: 运行时报告无法找到DynamicObjectFilter
A: 检查CMakeLists.txt中的包含路径设置是否正确。

### Q: 过滤效果不理想
A: 调整动态过滤器参数，特别是`motion_threshold`和`stability_threshold`。

### Q: 处理延迟过高
A: 降低`max_processing_hz`参数或增加`downsample_ratio`来减少计算量。

## 技术细节

### 消息转换

该节点实现了livox CustomMsg与PCL点云格式之间的高效转换：

- **CustomMsg → PCL**: 提取x,y,z坐标和反射强度
- **PCL → CustomMsg**: 保持原始消息的时间戳和其他元数据

### 过滤算法

复用localizer包中的DynamicObjectFilter，该算法结合：
- 多帧时间一致性分析
- 局部几何特征检查  
- 运动模式识别
- 密度变化检测

### 性能优化

- 频率控制避免过载
- 自适应降采样处理大点云
- 内存预分配减少动态分配
- 并行处理支持（未来版本）

## 许可证

MIT License

## 维护者

tianyu@todo.todo

---

**注意**: 这是修复动态过滤器数据流问题的关键组件，确保FastLIO2接收到经过动态对象过滤的静态点云数据，从而生成更高质量的地图。