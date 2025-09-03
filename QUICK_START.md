# MID360 SLAM建图系统 - 快速开始

## 🚀 一键启动

```bash
cd /home/tianyu/codes/Mid360map
./tools/start_slam.sh
```

## 📋 系统要求

- **硬件**: Livox MID360 激光雷达
- **系统**: Ubuntu 22.04 + ROS2 Humble  
- **网络**: MID360 IP=192.168.1.3, 电脑 IP=192.168.1.50

## 🛠️ 常用命令

```bash
# 启动系统
./tools/start_slam.sh

# 检查状态  
./tools/check_slam.sh

# 停止系统
./tools/stop_slam.sh

# 工具集
./tools/slam_tools.sh help
```

## 📖 详细文档

- [完整使用文档](README_SLAM.md)
- [项目总结](PROJECT_SUMMARY.md)
- [工具说明](tools/README.md)

## ⚡ 核心功能

✅ **实时SLAM建图** - FAST-LIO2算法  
✅ **3D可视化** - RViz2显示  
✅ **自动化管理** - 一键启停  
✅ **智能监控** - 状态检查  

---
*建图效果立即可见！* 🗺️