# MID360 FASTLIO2_ROS2 SLAM系统使用指南

## 系统概述

本系统基于Livox MID360激光雷达和FASTLIO2算法实现实时SLAM建图，支持ROS2 Humble环境。

### 核心组件
- **硬件**: Livox MID360 激光雷达
- **算法**: FASTLIO2_ROS2 (来源: liangheming/FASTLIO2_ROS2)
- **驱动**: livox_ros_driver2
- **环境**: Ubuntu 22.04 + ROS2 Humble

## 网络配置

| 设备 | IP地址 | 用途 |
|------|--------|------|
| 计算机 | 192.168.1.50 | 数据接收端 |
| MID360雷达 | 192.168.1.3 | 数据发送端 |

## 快速启动

### 1. 启动SLAM系统
```bash
cd /home/tianyu/codes/Mid360map
./start_slam.sh
```

### 2. 检查系统状态
```bash
./check_slam.sh
```

### 3. 停止系统
```bash
./stop_slam.sh
```

## 系统架构

### 数据流
```
MID360雷达 → livox_ros_driver2 → FASTLIO2算法 → RViz可视化
    ↓                ↓              ↓           ↓
  原始数据        ROS2话题        SLAM输出      实时显示
```

### 主要话题

**输入话题:**
- `/livox/lidar` - 点云数据 (sensor_msgs/PointCloud2)
- `/livox/imu` - IMU数据 (sensor_msgs/Imu)

**输出话题:**
- `/fastlio2/lio_odom` - SLAM里程计 (nav_msgs/Odometry)
- `/fastlio2/lio_path` - 机器人轨迹 (nav_msgs/Path)
- `/fastlio2/world_cloud` - 世界坐标系点云地图 (sensor_msgs/PointCloud2)
- `/fastlio2/body_cloud` - 机身坐标系点云 (sensor_msgs/PointCloud2)

## 配置文件

### 主要配置
1. **MID360驱动配置**: `ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
2. **FASTLIO2参数**: `ws_livox/install/fastlio2/share/fastlio2/config/lio.yaml`

### 关键参数
```yaml
# FASTLIO2配置 (lio.yaml)
imu_topic: /livox/imu
lidar_topic: /livox/lidar
lidar_min_range: 0.5
lidar_max_range: 30.0
scan_resolution: 0.15
map_resolution: 0.3
```

## 常用操作命令

### 基础监控
```bash
# 查看所有话题
ros2 topic list

# 监控里程计数据
ros2 topic echo /fastlio2/lio_odom

# 查看话题频率
ros2 topic hz /fastlio2/lio_odom

# 检查话题信息
ros2 topic info /fastlio2/world_cloud
```

### 系统调试
```bash
# 查看节点状态
ros2 node list

# 检查节点信息
ros2 node info /fastlio2/lio_node

# 查看参数
ros2 param list /fastlio2/lio_node

# 查看计算图
rqt_graph
```

## 故障排除

### 常见问题

1. **网络连接问题**
   ```bash
   # 检查雷达连接
   ping 192.168.1.3
   
   # 检查本机IP
   ip addr show
   ```

2. **驱动启动失败**
   - 确认网络配置正确
   - 检查配置文件IP地址
   - 确认端口未被占用

3. **SLAM无输出**
   - 检查输入数据流: `ros2 topic hz /livox/lidar`
   - 确认IMU数据: `ros2 topic hz /livox/imu`
   - 查看节点日志

4. **RViz无法显示**
   - 手动启动RViz: `rviz2`
   - 加载配置文件: `ws_livox/install/fastlio2/share/fastlio2/rviz/fastlio2.rviz`

### 日志位置
- ROS日志: `~/.ros/log/`
- 启动脚本日志: 运行时显示的日志目录

## 高级功能

### 地图保存
FASTLIO2_ROS2还支持其他高级功能包（需要额外编译GTSAM）:

1. **回环检测**: `pgo` 包
2. **重定位**: `localizer` 包  
3. **地图优化**: `hba` 包

安装GTSAM后可启用这些功能:
```bash
# 安装GTSAM (需要sudo权限)
sudo apt install libgtsam-dev

# 编译完整系统
colcon build
```

## 性能优化

### 系统要求
- CPU: Intel i5以上或同等AMD处理器
- 内存: 8GB以上
- 存储: SSD推荐
- 网络: 千兆以太网

### 调优参数
根据硬件性能调整 `lio.yaml` 中的参数:
- `scan_resolution`: 扫描分辨率 (0.1-0.2)
- `map_resolution`: 地图分辨率 (0.2-0.5)
- `cube_len`: 地图立方体大小 (200-500)

## 开发说明

### 工作空间结构
```
ws_livox/
├── src/
│   ├── livox_ros_driver2/    # Livox驱动
│   ├── fastlio2/             # FASTLIO2核心
│   ├── interface/            # 服务接口定义
│   ├── pgo/                  # 回环检测(可选)
│   ├── localizer/            # 重定位(可选)
│   └── hba/                  # 地图优化(可选)
├── build/                    # 编译临时文件
├── install/                  # 安装文件
└── log/                      # 编译日志
```

### 重新编译
```bash
cd /home/tianyu/codes/Mid360map/ws_livox

# 清理编译
rm -rf build install log

# 编译核心包
colcon build --packages-select livox_ros_driver2 fastlio2 interface

# 编译全部(需要GTSAM)
colcon build
```

## 技术支持

### 相关资源
- **FASTLIO2原版**: https://github.com/hku-mars/FAST_LIO
- **FASTLIO2_ROS2**: https://github.com/liangheming/FASTLIO2_ROS2
- **Livox驱动**: https://github.com/Livox-SDK/livox_ros_driver2
- **MID360官网**: https://www.livoxtech.com/mid-360

### 维护记录
- 2025-09-03: 初始部署完成
- 系统版本: ROS2 Humble + Ubuntu 22.04
- 部署环境: Legion R9000P ARX8