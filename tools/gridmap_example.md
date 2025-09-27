# 栅格地图转换使用示例

## 概述

本文档演示如何将SLAM系统保存的PCD点云地图转换为ROS兼容的栅格地图，用于导航和路径规划。

## 快速开始

### 1. 转换最新的PCD地图（推荐）

```bash
# 使用slam_tools.sh的集成命令（最简单）
./tools/slam_tools.sh gridmap
```

### 2. 转换指定的PCD地图

```bash
# 转换特定的PCD文件
./tools/slam_tools.sh gridmap mid360_map_20250918_151632.pcd
```

### 3. 查看转换结果

```bash
# 分析栅格地图并保存可视化图像
python3 tools/view_gridmap.py saved_maps/mid360_map_20250918_151632_gridmap.yaml --save-png
```

## 高级用法

### 自定义转换参数

```bash
# 使用更高精度 + 车辆参数（底盘高0.3m，整车高0.8m，外宽0.55m）
python3 tools/pcd_to_gridmap.py saved_maps/input.pcd \
    -o saved_maps/high_res_map \
    --resolution 0.03 \
    --ground-min -0.3 \
    --ground-max 0.3 \
    --ceiling-min 1.8 \
    --ceiling-max 3.0 \
    --robot-height 0.8 \
    --obstacle-thresh 6 \
    --vertical-thresh 0.8 \
    --inflate-radius 0.35
```

### 参数说明

- `--resolution`: 栅格分辨率（米/像素），默认0.05。更小的值提供更高精度但文件更大
- `--ground-min/max`: 地面附近高度范围，用于可通行性判断
- `--ceiling-min/max`: 天花板高度范围；室外转换可保持默认或调高上限
- `--robot-height`: 车辆高度（米），用于垂直结构与通行性推断
- `--obstacle-thresh`: 障碍物点云密度阈值（机器人层）
- `--vertical-thresh`: 垂直连续性阈值，用于墙柱判定
- `--inflate-radius`: 障碍物膨胀半径（米），建议≈半车宽0.275m + 0.05~0.10m冗余；示例用0.35

## 输出文件说明

转换完成后会生成三个文件：

### 1. PGM文件（栅格地图图像）
- **文件名**: `*_gridmap.pgm`
- **用途**: 栅格地图的实际图像数据
- **格式**: PGM图像格式
- **颜色映射**:
  - 黑色(0): 障碍物
  - 白色(255): 自由空间
  - 灰色(127): 未知区域

### 2. YAML文件（地图元数据）
- **文件名**: `*_gridmap.yaml`
- **用途**: ROS地图服务器使用的元数据
- **包含信息**: 分辨率、原点坐标、阈值等
- **ROS兼容**: 可直接用于`map_server`

### 3. INFO文件（转换信息）
- **文件名**: `*_gridmap.info`
- **用途**: 转换过程的详细信息和统计
- **内容**: 转换参数、像素统计、地图尺寸等

## 在ROS中使用栅格地图

### 1. 加载地图到ROS

```bash
# 使用map_server加载栅格地图
ros2 run map_server map_server --ros-args \
    -p yaml_filename:=saved_maps/mid360_map_20250918_151632_gridmap.yaml
```

### 2. 在导航栈中使用

```xml
<!-- 在launch文件中使用 -->
<node pkg="map_server" exec="map_server" name="map_server">
    <param name="yaml_filename" value="/path/to/your_gridmap.yaml"/>
</node>
```

### 3. 与导航栈集成

栅格地图可以直接用于：
- **AMCL定位**: 粒子滤波定位
- **全局路径规划**: A*、Dijkstra等算法
- **局部路径规划**: DWA、TEB等算法
- **障碍物检测**: 静态障碍物映射

## 质量评估

### 地图质量指标

转换完成后，可以通过以下指标评估地图质量：

1. **覆盖率**: 自由空间像素占比应该合理（通常70-95%）
2. **障碍物密度**: 障碍物像素占比（通常1-10%）
3. **未知区域**: 应该尽可能少（<5%）
4. **分辨率适中**: 平衡精度和文件大小

### 优化建议

**如果障碍物太多**:
- 增大`--obstacle-thresh`参数
- 调整高度过滤范围
- 检查SLAM数据质量

**如果地图稀疏**:
- 减小`--obstacle-thresh`
- 使用更精细的分辨率（0.03–0.05）

**如果文件太大**:
- 增大分辨率值（如0.1而不是0.05）
- 裁剪地图边界
- 压缩PGM文件

## 常见问题

### Q: 转换后的地图看起来很稀疏？
A: 尝试减小`--obstacle-thresh`和`--free-thresh`参数值，或使用更精细的分辨率。

### Q: 地图文件很大怎么办？
A: 增大分辨率参数（如0.1），或者在保存PCD时减少点云密度。

### Q: 如何处理室外→室内过渡？
A: 保持`--robot-height`准确，室外的高空点（树冠/雨棚）不会标为障碍；仅当机器人高度层出现密集点时才判为障碍。必要时可增大`--ceiling-min`，或启用`--disable-panoramic`做纯2D密度图。

### Q: 多层建筑怎么做？
A: 分别针对每层设置`--ground-min/max`与`--ceiling-min/max`并各自生成一张地图。

### Q: 转换的地图在RViz中显示异常？
A: 检查YAML文件中的`negate`参数，确保原点坐标正确。

## 自动化脚本示例

```bash
#!/bin/bash
# 批量转换所有PCD地图为栅格地图

MAPS_DIR="saved_maps"

for pcd_file in "$MAPS_DIR"/*.pcd; do
    if [ -f "$pcd_file" ]; then
        echo "转换: $pcd_file"
        ./tools/slam_tools.sh gridmap "$(basename "$pcd_file")"
    fi
done

echo "所有地图转换完成！"
```

## 总结

栅格地图转换功能为SLAM系统提供了完整的2D地图生成能力，支持：
- ✅ 多种分辨率和参数配置
- ✅ ROS标准格式输出
- ✅ 可视化和质量分析
- ✅ 自动化批量处理
- ✅ 与导航栈无缝集成

通过合理的参数调整，可以为不同应用场景生成高质量的栅格地图。
