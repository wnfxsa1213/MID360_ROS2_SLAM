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

### 编译项目
```bash
# 标准ROS2编译（推荐）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 调试版本编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 仅编译本包
colcon build --packages-select mid360_mapping

# 并行编译（加速）
colcon build --parallel-workers 4
```

### 运行和测试
```bash
# 设置环境变量（必须）
source install/setup.bash

# 运行单独的节点
ros2 run mid360_mapping mid360_receiver
ros2 run mid360_mapping fast_lio2_node
ros2 run mid360_mapping visualization_node
ros2 run mid360_mapping system_monitor

# 运行测试
colcon test --packages-select mid360_mapping
colcon test-result --verbose

# 运行特定测试
colcon test --packages-select mid360_mapping --ctest-args -R test_data_converter
colcon test --packages-select mid360_mapping --ctest-args -R test_fast_lio2
```

### 启动系统
```bash
# 启动系统前先安装依赖和配置网络
./scripts/install_deps.sh
sudo ./scripts/setup_network.sh

# 启动完整系统（当launch文件存在时）
ros2 launch mid360_mapping full_system.launch.py

# 或使用脚本启动
./scripts/start_system.sh
```

### 开发工具命令
```bash
# 代码质量检查
ament_cpplint src/
ament_cppcheck src/
ament_uncrustify --reformat src/

# 依赖管理
rosdep check --from-paths src --ignore-src -r
rosdep install --from-paths src --ignore-src -r -y

# 构建清理
rm -rf build install log
colcon build --cmake-clean-cache
```

## 代码架构

### 核心组件

项目基于ROS2架构，主要包含以下可执行节点：

1. **MID360数据接收节点** (`mid360_receiver`)
   - 源文件：`src/mid360_driver/mid360_receiver.cpp`
   - 处理MID360激光雷达的UDP数据接收和光纤传输
   - 依赖：`src/utils/data_converter.cpp`

2. **FAST-LIO2 SLAM节点** (`fast_lio2_node`)
   - 主文件：`src/fast_lio2/fast_lio2_node.cpp`
   - ikd-Tree实现：`src/fast_lio2/ikd_Tree.cpp`
   - IMU处理：`src/fast_lio2/IMU_Processing.cpp`
   - 点云预处理：`src/fast_lio2/preprocess.cpp`
   - 通用库：`src/utils/common_lib.cpp`

3. **可视化节点** (`visualization_node`)
   - 源文件：`src/visualization/visualization_node.cpp`
   - 标记工具：`src/utils/marker_utils.cpp`
   - 实时RViz显示建图结果和机器人轨迹

4. **系统监控节点** (`system_monitor`)
   - 源文件：`src/utils/system_monitor.cpp`
   - 监控系统性能和资源使用

5. **Livox驱动** (子模块)
   - 位置：`src/livox_ros_driver2/`
   - 提供MID360硬件驱动支持

### 数据流架构

```
MID360雷达 -> UDP数据接收 -> 点云/IMU数据 -> FAST-LIO2算法 -> 地图/里程计输出
                |                                        |
                v                                        v
        光纤传输优化                            RViz可视化显示
```

### 关键配置文件

- `config/mid360_config.yaml`: MID360雷达配置，包括网络设置(IP: 192.168.1.100/192.168.1.50)、扫描参数、光纤传输优化
- `config/fast_lio2.yaml`: FAST-LIO2算法完整配置，包含预处理、IMU参数、建图参数、外参标定等
- `src/livox_ros_driver2/config/`: Livox驱动原生配置文件
- `package.xml`: ROS2包依赖定义
- `CMakeLists.txt`: 构建配置，定义4个主要可执行文件

### ROS2话题结构

**输入话题:**
- `/livox/lidar` (sensor_msgs/PointCloud2): MID360点云数据
- `/livox/imu` (sensor_msgs/Imu): IMU数据

**输出话题:**
- `/Odometry` (nav_msgs/Odometry): SLAM里程计输出
- `/trajectory` (nav_msgs/Path): 机器人轨迹
- `/map_cloud` (sensor_msgs/PointCloud2): 建图结果

### 开发注意事项

1. **编译要求**: 需要C++17标准，使用OpenMP并行化，依赖PCL、Eigen3等库
2. **网络配置**: MID360默认IP为192.168.1.1XX (XX为SN后两位)，地面端配置为192.168.1.50
3. **时间同步**: 支持软件时间同步，光纤传输延迟<1ms，可配置时间偏移
4. **性能优化**: 使用OpenMP并行化，支持100Hz实时处理，ikd-Tree高效点云管理
5. **坐标系**: 使用ROS2标准坐标系(base_link, livox_frame, camera_init)，支持TF发布
6. **数据格式**: 支持PointCloud2格式，可配置降采样和滤波参数
7. **依赖管理**: 项目依赖livox_ros_driver2子模块，需要正确初始化
8. **测试覆盖**: 包含数据转换和FAST-LIO2核心模块的单元测试

### 测试策略

- 单元测试覆盖数据转换和核心算法模块
- 集成测试验证完整SLAM流程
- 性能测试确保实时性要求

### 故障排除

1. **网络连接问题**: 检查IP配置和光纤连接
2. **数据延迟**: 监控网络带宽和系统负载
3. **建图精度**: 调整FAST-LIO2参数，特别是体素大小和迭代次数
4. **内存使用**: 监控ikd-Tree大小和点云数量限制