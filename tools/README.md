# MID360 SLAM 工具集

本目录包含MID360 LiDAR SLAM系统的各种工具和实用程序。

## 🔧 核心工具

### slam_tools.sh - SLAM系统管理工具集 ⭐
**使用方法：** `./tools/slam_tools.sh [命令]`

完整的SLAM系统管理和调试工具，包含以下功能：
- `build` - 按依赖顺序编译系统
- `start` - 启动SLAM系统  
- `stop` - 停止SLAM系统
- `status` - 检查系统状态
- `monitor` - 实时监控数据流
- `save` - 保存地图和轨迹
- `clean` - 清理编译文件
- `fix` - 修复已知编译问题

## 🎨 点云处理工具

### advanced_point_cloud_processor.py - 高级点云处理器 ⭐
**功能：** 专业的点云优化和表面重建工具
- 强度感知过滤和几何增强
- 支持4种表面重建算法（Poisson、Alpha Shape、Ball Pivoting、Delaunay 2D）
- 自动生成彩色可视化文件

**使用：** 
```bash
# 基本处理
python3 tools/advanced_point_cloud_processor.py input.pcd

# 表面重建
python3 tools/advanced_point_cloud_processor.py input.pcd \
    --surface-reconstruction \
    --reconstruction-method alpha_shape
```

### point_cloud_processor_gui.py - 点云处理GUI界面 ⭐
**功能：** PyQt5图形化点云处理工具
- 直观的参数设置界面
- 实时进度显示和结果统计
- 多线程后台处理

**启动：** `./tools/start_gui.sh` 或 `python3 tools/point_cloud_processor_gui.py`

### rviz_map_viewer.py - RViz地图查看器  
**功能：** 读取保存的地图，发布ROS2话题供RViz显示
**使用：** `python3 tools/rviz_map_viewer.py saved_map.pcd`

## ⚙️ 系统工具

### config_generator.py - 配置文件生成器
**功能：** 从主配置文件生成各组件配置，确保参数一致性
**使用：** `python3 tools/config_generator.py`

### install_gtsam.sh - GTSAM库安装脚本
**功能：** 自动化安装和配置GTSAM依赖库
**使用：** `./tools/install_gtsam.sh`

## 📚 文档文件

- **GUI_README.md** - GUI界面详细使用说明
- **INSTALL_GUI.md** - GUI安装和配置指南  
- **README.md** - 本文档

## 📁 目录结构

```
tools/
├── slam_tools.sh                      # SLAM系统管理脚本
├── advanced_point_cloud_processor.py  # 点云处理器（命令行）
├── point_cloud_processor_gui.py       # 点云处理器（GUI版本）  
├── start_gui.sh                       # GUI启动脚本
├── rviz_map_viewer.py                 # RViz地图查看器
├── config_generator.py                # 配置文件生成器
├── install_gtsam.sh                   # GTSAM安装脚本
├── rviz/                              # RViz配置文件
│   └── saved_map_viewer.rviz         # 地图查看器配置
├── archive/                           # 归档文件（过时工具）
│   └── README.md                     # 归档说明
└── *.md                              # 各种文档文件
```

## 🚀 快速开始

### 1. 系统构建和启动
```bash
./tools/slam_tools.sh fix     # 修复已知问题
./tools/slam_tools.sh build   # 编译系统
./tools/slam_tools.sh start   # 启动SLAM
```

### 2. 地图处理
```bash
# 保存地图
./tools/slam_tools.sh save

# 处理点云（命令行）
python3 tools/advanced_point_cloud_processor.py saved_maps/map.pcd

# 处理点云（GUI界面）
./tools/start_gui.sh
```

### 3. 结果查看
```bash
# RViz查看
python3 tools/rviz_map_viewer.py saved_maps/enhanced_map.pcd

# 系统监控
./tools/slam_tools.sh monitor
```

## 🎯 推荐工作流

1. **系统启动**: `./tools/slam_tools.sh start`
2. **地图保存**: `./tools/slam_tools.sh save` 
3. **点云处理**: `./tools/start_gui.sh` (GUI) 或命令行处理
4. **结果可视化**: `python3 tools/rviz_map_viewer.py` 

## 📦 归档文件

过时或重复的工具已移至 `tools/archive/` 目录，包括：
- `check_pcd_intensity.py` - 功能已集成到高级处理器
- `save_map_simple.py` - 功能已集成到slam_tools.sh
- `map_saver_node.py` - 功能已集成到slam_tools.sh  
- `cleanup_project.py` - 一次性清理脚本（已完成）
- `test_*` 脚本 - 开发测试脚本（功能已验证）

详见 `tools/archive/README.md` 了解归档原因和用途。