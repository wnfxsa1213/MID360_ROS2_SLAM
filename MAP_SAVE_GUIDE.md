# MID360 SLAM稠密点云地图保存使用指南

## 概述

本指南介绍如何使用新增的地图保存功能来保存和查看MID360 SLAM系统生成的稠密点云地图。

## 快速开始

### 1. 保存地图
```bash
# 启动SLAM系统后，使用以下命令保存地图
./tools/slam_tools.sh save
```

### 2. 查看已保存的地图
```bash
# 查看所有已保存的地图
./tools/slam_tools.sh maps

# 可视化特定地图文件 (需要安装Open3D: pip install open3d)  
./tools/slam_tools.sh view saved_maps/mid360_map_20250904_143000.pcd
```

## 功能特性

✅ **一键保存**: 通过简单命令保存稠密点云地图和轨迹  
✅ **多种格式**: 支持PCD、PLY点云格式，TXT轨迹格式  
✅ **可视化工具**: 提供3D点云查看器  
✅ **集成工具**: 完全集成到slam_tools.sh工具集  
✅ **自动管理**: 自动创建目录，时间戳命名  

## 使用场景

- **团队协作**: 保存地图供其他成员查看和分析
- **结果展示**: 生成可视化材料用于报告和演示  
- **数据备份**: 保存重要的建图成果
- **离线分析**: 导出数据进行后续处理

## 输出文件说明

保存后会在`saved_maps/`目录下生成:
- `mid360_map_YYYYMMDD_HHMMSS.pcd`: 稠密点云地图
- `mid360_trajectory_YYYYMMDD_HHMMSS.txt`: SLAM轨迹数据

## 依赖要求

- **必需**: ROS2 Humble, PCL库
- **可选**: Open3D (用于3D可视化): `pip install open3d`

## 故障排除

如果保存失败:
1. 确保SLAM系统正在运行: `./tools/slam_tools.sh status`
2. 测试服务可用性: `python3 tools/test_map_save_simple.py`  
3. 检查磁盘空间是否充足

## 技术支持

更多详细信息请查看项目中的相关文档和代码注释。