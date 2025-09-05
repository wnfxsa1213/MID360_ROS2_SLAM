# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于Livox MID360激光雷达的穿越机实时SLAM建图系统，采用光纤传输技术实现超低延迟数据传输，结合FAST-LIO2算法进行实时建图。

## 开发环境要求

- Ubuntu 20.04/22.04
- ROS2 Humble
- C++17编译器
- CMake 3.10+

## 构建和开发命令

### 使用便捷脚本（推荐）
```bash
# 使用SLAM工具集脚本（推荐）
./tools/slam_tools.sh help     # 查看所有可用命令
./tools/slam_tools.sh fix      # 修复已知编译问题
./tools/slam_tools.sh build    # 按依赖顺序编译系统
./tools/slam_tools.sh start    # 启动SLAM系统
./tools/slam_tools.sh status   # 检查系统状态
./tools/slam_tools.sh monitor  # 实时监控数据流
```

### 手动编译项目
```bash
# 进入ROS2工作空间
cd ws_livox

# 按依赖顺序编译（重要）
colcon build --packages-select interface --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select fastlio2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置环境变量（必须）
source install/setup.bash
```

### 启动系统
```bash
# 方法1：使用便捷脚本（推荐）
./tools/slam_tools.sh start

# 方法2：使用launch文件
cd ws_livox
source install/setup.bash
ros2 launch fastlio2 enhanced_visualization.launch.py

# 方法3：使用启动脚本
./tools/start_slam.sh
```

### 系统监控和调试
```bash
# 实时监控数据流
./tools/slam_tools.sh monitor

# 查看系统状态
./tools/slam_tools.sh status

# 查看话题和节点
./tools/slam_tools.sh topics
./tools/slam_tools.sh nodes

# 网络配置检查
./tools/slam_tools.sh network

# 查看系统日志
./tools/slam_tools.sh log
```

### 故障排除和修复
```bash
# 修复已知编译问题
./tools/slam_tools.sh fix

# 清理并重新编译
./tools/slam_tools.sh clean
./tools/slam_tools.sh build

# 重启系统
./tools/slam_tools.sh restart
```

## 代码架构

### 核心组件

项目基于ROS2架构，当前实现的主要组件：

1. **Livox MID360驱动节点** (`livox_ros_driver2`)
   - 位置：`ws_livox/src/livox_ros_driver2/`
   - 功能：MID360激光雷达数据采集和发布
   - 配置：`ws_livox/src/livox_ros_driver2/config/MID360_config.json`
   - 话题：发布点云数据到`/livox/lidar`，IMU数据到`/livox/imu`

2. **FAST-LIO2 SLAM节点** (`fastlio2`)
   - 位置：`ws_livox/src/fastlio2/`
   - 主要文件：`src/lio_node.cpp`（SLAM核心实现）
   - 配置文件：`config/lio.yaml`
   - 功能：实时SLAM定位建图，输出里程计和地图点云
   - 话题：订阅雷达和IMU数据，发布里程计和地图

3. **增强可视化系统**
   - Launch文件：`ws_livox/src/fastlio2/launch/enhanced_visualization.launch.py`
   - RViz配置：`ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz`
   - 功能：实时显示建图过程、轨迹和点云数据

4. **接口包** (`interface`)
   - 位置：`ws_livox/src/interface/`
   - 功能：定义自定义消息和服务接口

5. **系统管理工具**
   - SLAM工具集：`tools/slam_tools.sh`（完整的系统管理和调试工具）
   - 启动脚本：`tools/start_slam.sh`
   - 配置管理：`config/launch_config.yaml`

### 数据流架构

```
MID360雷达 -> UDP数据接收 -> 点云/IMU数据 -> FAST-LIO2算法 -> 地图/里程计输出
                |                                        |
                v                                        v
        光纤传输优化                            RViz可视化显示
```

### 关键配置文件

**系统级配置：**
- `config/launch_config.yaml`: 系统启动配置，包含网络设置(主机IP: 192.168.1.50, 雷达IP: 192.168.1.3)、端口配置、调试选项
- `tools/slam_tools.sh`: SLAM系统管理工具集，提供编译、启动、监控等功能

**Livox驱动配置：**
- `ws_livox/src/livox_ros_driver2/config/MID360_config.json`: MID360雷达硬件配置
- `ws_livox/src/livox_ros_driver2/CMakeLists.txt`: 驱动编译配置（已修复ROS版本检测问题）

**FAST-LIO2配置：**
- `ws_livox/src/fastlio2/config/lio.yaml`: SLAM算法核心配置，包含IMU参数、建图参数、外参标定等
- `ws_livox/src/fastlio2/package.xml`: ROS2包依赖定义
- `ws_livox/src/fastlio2/CMakeLists.txt`: SLAM核心编译配置

**可视化配置：**
- `ws_livox/src/fastlio2/launch/enhanced_visualization.launch.py`: 增强可视化启动文件
- `ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz`: RViz显示配置

### ROS2话题结构

**Livox驱动输出话题:**
- `/livox/lidar` (sensor_msgs/PointCloud2): MID360点云数据
- `/livox/imu` (sensor_msgs/Imu): IMU数据  
- `/livox/status` (std_msgs/UInt8): 设备状态信息

**FAST-LIO2 SLAM输出话题:**
- `/fastlio2/lio_odom` (nav_msgs/Odometry): SLAM里程计输出
- `/fastlio2/body` (sensor_msgs/PointCloud2): 机体坐标系点云
- `/fastlio2/path` (nav_msgs/Path): SLAM轨迹路径
- `/fastlio2/residual_point` (sensor_msgs/PointCloud2): 残差点云

**TF变换发布:**
- `odom -> base_link`: SLAM位姿变换
- `base_link -> livox_frame`: 雷达外参变换

**系统监控话题:**
- 可通过`./tools/slam_tools.sh monitor`实时查看话题状态和频率

### 开发注意事项

1. **编译顺序**: 必须按依赖顺序编译：interface → livox_ros_driver2 → fastlio2
2. **已知问题修复**: livox_ros_driver2存在ROS版本检测问题，使用`./tools/slam_tools.sh fix`自动修复
3. **网络配置**: 系统配置为主机IP 192.168.1.50，雷达IP 192.168.1.3，需要相应网络设置
4. **环境依赖**: 需要ROS2 Humble、C++17编译器、PCL、Eigen3等库
5. **启动方式**: 推荐使用`./tools/slam_tools.sh start`启动系统，支持自动环境配置
6. **坐标系**: 使用标准ROS坐标系(odom, base_link, livox_frame)
7. **性能监控**: 使用`./tools/slam_tools.sh monitor`实时监控数据流和系统状态
8. **调试工具**: slam_tools.sh提供完整的调试工具集，包括日志查看、网络检查等
9. **可视化**: 支持增强RViz配置，自动显示点云、轨迹和TF变换
10. **配置管理**: 统一使用`config/launch_config.yaml`管理系统参数

### 当前开发状态

**已完成组件：**
- ✅ Livox MID360驱动集成
- ✅ FAST-LIO2 SLAM核心实现
- ✅ 增强可视化系统（RViz配置）
- ✅ 系统管理工具集（slam_tools.sh）
- ✅ 编译问题修复机制
- ✅ 配置文件统一管理
- ✅ 自动化启动脚本

**测试和验证：**
- 🔄 系统集成测试（进行中）
- 🔄 实时性能验证（进行中）
- ⏳ 长时间稳定性测试（待进行）

### 故障排除指南

**编译问题：**
1. 运行`./tools/slam_tools.sh fix`修复已知问题
2. 按顺序重新编译：`./tools/slam_tools.sh build`
3. 清理后重新编译：`./tools/slam_tools.sh clean && ./tools/slam_tools.sh build`

**网络连接问题：**
1. 检查网络配置：`./tools/slam_tools.sh network`
2. 验证雷达连接：ping 192.168.1.3
3. 检查接口配置：修改`config/launch_config.yaml`中的网络设置

**SLAM性能问题：**
1. 实时监控数据流：`./tools/slam_tools.sh monitor`
2. 查看系统状态：`./tools/slam_tools.sh status`
3. 检查系统日志：`./tools/slam_tools.sh log`
4. 调整FAST-LIO2参数：修改`ws_livox/src/fastlio2/config/lio.yaml`

**可视化问题：**
1. 启动RViz：`./tools/slam_tools.sh rviz`
2. 检查话题发布：`./tools/slam_tools.sh topics`
3. 验证TF变换：`ros2 run tf2_tools view_frames`