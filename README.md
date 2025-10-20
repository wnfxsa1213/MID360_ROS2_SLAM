# MID360 ROS2 SLAM 系统

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange.svg)](https://ubuntu.com/)

基于 Livox MID360 激光雷达的高精度实时 SLAM 建图系统

[功能特性](#功能特性) • [系统架构](#系统架构) • [快速开始](#快速开始) • [使用指南](#使用指南) • [配置说明](#配置管理)

</div>

---

## 📋 目录

- [系统概述](#系统概述)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [软件依赖](#软件依赖)
- [安装指南](#安装指南)
- [快速开始](#快速开始)
- [使用指南](#使用指南)
- [配置管理](#配置管理)
- [工具脚本](#工具脚本)
- [图形界面工具](#图形界面工具)
- [常见问题](#常见问题)
- [性能优化](#性能优化)
- [贡献指南](#贡献指南)

---

## 🎯 系统概述

这是一个基于 **Livox MID360** 激光雷达和 **ROS2 Humble** 的完整 SLAM（同步定位与建图）解决方案。系统集成了多种先进的 SLAM 技术，提供从数据采集、实时建图、后端优化到地图导出的完整工具链。

**适用场景**：
- 移动机器人自主导航
- 室内外环境三维建图
- 无人车/AGV定位导航
- 工业场景数字孪生

---

## 🆕 最近更新

- **2025-02-15**：修复 PGO 节点的锁使用错误并引入优化分数评估，避免回环处理时的竞态和虚假分数。
- **2025-02-15**：协同优化协调器的所有跨节点服务调用改为真正异步流程，消除 `future.get()` 阻塞造成的节点假死。
- **2025-02-15**：补齐重定位服务返回值的使用并加上姿态归一化，保证 FAST-LIO2 能收到经过校验的位姿。
- **2025-02-16**：新增紧急优化抢占状态标记，确保漂移告警时 PGO 能在当前任务完成后立即接管。

---

## ✨ 功能特性

### 核心功能

- **🚀 实时 SLAM 建图**
  - 基于 FAST-LIO2 的紧耦合激光-惯性里程计
  - 支持 10Hz 实时点云处理
  - 低漂移、高精度位姿估计

- **🔄 后端优化**
  - PGO（位姿图优化）回环检测与全局优化
  - HBA（分层束调整）精细化地图处理
  - 协同优化协调器自动触发优化

- **🎯 重定位与定位**
  - 基于已知地图的快速重定位
  - 粗定位 + 精定位两阶段匹配
  - 支持动态对象过滤

- **🛠️ 完整工具链**
  - 统一的命令行工具 `slam_tools.sh`
  - **图形界面管理工具**（PyQt5）
    - SLAM 系统控制面板
    - 配置文件可视化编辑
    - 实时性能监控
    - 栅格地图生成 GUI
  - 配置文件自动生成与验证
  - 数据录制与回放验证
  - PCD 到栅格地图转换
  - 地图精细化处理工具

### 高级特性

- ✅ **模块化设计** - 各组件独立可配置
- ✅ **动态对象过滤** - 可选的运动物体滤除
- ✅ **双模式运行** - 实机 / 数据回放灵活切换
- ✅ **协同优化** - 自动漂移检测与优化触发
- ✅ **配置管理** - 主配置文件 + 自动生成机制
- ✅ **性能监控** - 实时性能指标与诊断信息

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     硬件层 - Livox MID360                    │
│                  (激光雷达 + 集成IMU)                        │
└────────────────────────┬────────────────────────────────────┘
                         │ UDP 通信
┌────────────────────────▼────────────────────────────────────┐
│                 驱动层 - livox_ros_driver2                   │
│              /livox/lidar + /livox/imu                       │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│            预处理层 - point_cloud_filter                     │
│        动态对象检测与过滤 (可选)                            │
│              /livox/lidar_filtered                           │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                核心SLAM - FAST-LIO2                          │
│    紧耦合激光-惯性里程计，实时位姿估计与建图                │
│    /slam/lio_odom + /slam/world_cloud + /slam/body_cloud    │
└──────────┬─────────────┬────────────────┬───────────────────┘
           │             │                │
    ┌──────▼─────┐  ┌───▼────┐     ┌────▼─────┐
    │    PGO     │  │  HBA   │     │Localizer │
    │  回环检测  │  │ 精细化 │     │  重定位  │
    │  全局优化  │  │ 束调整 │     │          │
    └──────┬─────┘  └───┬────┘     └────┬─────┘
           │            │                │
           └────────────┼────────────────┘
                        │
           ┌────────────▼────────────┐
           │ Optimization Coordinator │
           │     优化协调器           │
           │   (漂移监控 + 自动优化)  │
           └─────────────────────────┘
```

### 数据流程

```
实时模式:
  Livox MID360 → Driver → Filter → FAST-LIO2 → PGO/HBA → 地图输出

回放模式:
  ROS2 Bag → Filter → FAST-LIO2 → PGO/HBA → 地图输出
```

---

## 💻 硬件要求

### 必需硬件

| 组件 | 规格 | 说明 |
|------|------|------|
| **激光雷达** | Livox MID360 | 集成IMU，FOV 360°×59° |
| **计算平台** | x86_64 CPU | 推荐 8核以上 |
| **内存** | ≥ 16GB RAM | 建议 32GB |
| **存储** | ≥ 100GB SSD | 用于地图存储 |
| **网络** | 千兆以太网 | 与雷达通信 |

### 网络配置

- **主机IP**: `192.168.1.50`
- **雷达IP**: `192.168.1.3`（出厂默认）
- **网段**: `192.168.1.0/24`

---

## 📦 软件依赖

### 系统环境

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble Hawksbill
- **编译器**: GCC 11+ / Clang 14+
- **CMake**: ≥ 3.16

### 核心依赖库

```bash
# ROS2 包
ros-humble-desktop
ros-humble-tf2
ros-humble-tf2-ros
ros-humble-pcl-ros
ros-humble-eigen3-cmake-module

# 第三方库
libpcl-dev          # 点云处理
libeigen3-dev       # 线性代数
libopencv-dev       # 视觉处理
libyaml-cpp-dev     # 配置解析
libgtest-dev        # 单元测试

# GTSAM（用于PGO和HBA）
libgtsam-dev        # 版本 4.0+
```

---

## 🔧 安装指南

### 1. 安装 ROS2 Humble

```bash
# 添加ROS2软件源
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. 安装系统依赖

```bash
# 安装基础依赖
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgtest-dev \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros \
    ros-humble-eigen3-cmake-module
```

### 3. 安装 GTSAM

```bash
# 使用系统工具自动安装
./tools/slam_tools.sh gtsam

# 或手动安装
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2a8
mkdir build && cd build
cmake .. -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
make -j$(nproc)
sudo make install
```

### 4. 克隆项目

```bash
git clone https://github.com/wnfxsa1213/MID360_ROS2_SLAM.git
cd MID360_ROS2_SLAM
```

### 5. 编译系统

```bash
# 使用工具脚本编译（推荐）
./tools/slam_tools.sh build

# 或手动编译
cd ws_livox
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 6. 配置网络

```bash
# 编辑主配置文件
vim config/master_config.yaml

# 修改网络配置部分
network:
  host:
    ip: 192.168.1.50      # 根据实际情况修改
    interface: enp7s0      # 根据实际网卡名称修改
  lidar:
    ip: 192.168.1.3        # 雷达IP

# 应用网络配置（需要root权限）
sudo ./tools/slam_tools.sh network
```

---

## 🚀 快速开始

### 实机运行

```bash
# 1. 启动完整SLAM系统
./tools/slam_tools.sh start

# 或启用动态过滤（推荐）
./tools/slam_tools.sh start realtime

# 或禁用动态过滤（直接使用原始点云）
./tools/slam_tools.sh start nofilter

# 2. 查看系统状态
./tools/slam_tools.sh status

# 3. 保存地图
./tools/slam_tools.sh save lio          # 保存LIO地图
./tools/slam_tools.sh save pgo          # 保存PGO优化后地图
./tools/slam_tools.sh save hba --refine # 保存HBA精细化地图

# 4. 停止系统
./tools/slam_tools.sh stop
```

### 数据回放

```bash
# 1. 录制数据
./tools/slam_tools.sh record

# 2. 回放数据
./tools/slam_tools.sh start replay --bag /path/to/bag

# 或禁用动态过滤回放（提高稳定性）
./tools/slam_tools.sh start replay --bag /path/to/bag --no-dynamic-filter

# 3. 调整回放速度
./tools/slam_tools.sh start replay --bag /path/to/bag --rate 0.5  # 半速
./tools/slam_tools.sh start replay --bag /path/to/bag --rate 2.0  # 2倍速

# 4. 循环回放
./tools/slam_tools.sh start replay --bag /path/to/bag --loop
```

---

## 📖 使用指南

### 主要功能

#### 1. 配置管理

```bash
# 生成所有配置文件
./tools/slam_tools.sh config generate

# 验证配置文件
./tools/slam_tools.sh config validate

# 查看配置状态
./tools/slam_tools.sh config status

# 应用预设配置
./tools/slam_tools.sh preset realtime   # 实机预设
./tools/slam_tools.sh preset replay     # 回放预设
```

#### 2. 地图管理

```bash
# 列出已保存的地图
./tools/slam_tools.sh maps

# 查看特定地图
./tools/slam_tools.sh view /path/to/map.pcd

# 在RViz中查看
./tools/slam_tools.sh view /path/to/map.pcd --rviz

# 转换为栅格地图
./tools/slam_tools.sh gridmap /path/to/map.pcd
```

#### 3. 地图精细化

```bash
# 基础精细化
./tools/slam_tools.sh refine /path/to/map.pcd

# 指定去噪模式
./tools/slam_tools.sh refine /path/to/map.pcd --denoising-mode aggressive
# 模式: conservative | balanced | aggressive

# 性能基准测试
./tools/slam_tools.sh benchmark
```

#### 4. 系统监控

```bash
# 实时监控数据流
./tools/slam_tools.sh monitor

# 查看话题列表
./tools/slam_tools.sh topics

# 查看节点列表
./tools/slam_tools.sh nodes

# 查看系统日志
./tools/slam_tools.sh log
```

#### 5. 动态过滤器

```bash
# 测试动态过滤器
./tools/slam_tools.sh filter test

# 可视化调试
./tools/slam_tools.sh filter viz

# 参数调优
./tools/slam_tools.sh filter tune

# 查看状态
./tools/slam_tools.sh filter status

# 启用/禁用
./tools/slam_tools.sh filter enable
./tools/slam_tools.sh filter disable
```

---

## ⚙️ 配置管理

### 配置文件结构

```
config/
├── master_config.yaml          # 主配置文件（唯一需要手动编辑）
└── launch_config.yaml          # 启动配置（自动生成）

ws_livox/src/
├── fastlio2/config/
│   └── lio.yaml               # FAST-LIO2配置（自动生成）
├── pgo/config/
│   └── pgo.yaml               # PGO配置（自动生成）
├── hba/config/
│   └── hba.yaml               # HBA配置（自动生成）
├── localizer/config/
│   └── localizer.yaml         # 定位器配置（自动生成）
└── point_cloud_filter/config/
    └── point_cloud_filter.yaml # 过滤器配置（自动生成）
```

> ⚠️ **生成器覆盖范围提示**  
> 当前 `tools/config_generator.py` 仅同步上述文件中的“核心字段”。如下参数仍需手动维护：
>
> - `fastlio2` 里的 `point_cloud.lidar_cov_scale`、`point_filter_num`、`converge_thresh` 等高级项；
> - `livox_driver` 的话题格式/多话题开关（在 launch 中仍是硬编码）；
> - `localizer.dynamic_filter.enable` 与过滤桥的启停开关、`max_processing_hz` 等；
> - `cooperation`、`monitoring` 模块的阈值与频率设置；
> - RViz 显示布尔开关（`visualization.display.*`）；
>
> 在调整上述参数时，请直接编辑对应子配置或 launch 文件，并将改动同步回 `master_config.yaml` 以免后续生成覆盖。

### 修改配置的标准流程

```bash
# 1. 编辑主配置文件
vim config/master_config.yaml

# 2. 生成子配置文件
./tools/slam_tools.sh config generate

# 3. 验证配置
./tools/slam_tools.sh config validate

# 4. 重启系统应用新配置
./tools/slam_tools.sh restart
```

### 重要参数说明

#### FAST-LIO2 参数

```yaml
fastlio2:
  lidar_filter_num: 1           # 激光滤波器数量
  lidar_min_range: 0.3          # 最小有效距离(m)
  lidar_max_range: 30.0         # 最大有效距离(m)
  scan_resolution: 0.2          # 扫描分辨率(m)
  map_resolution: 0.25          # 地图分辨率(m)
  ieskf_max_iter: 6             # 迭代EKF最大迭代次数
```

#### PGO 参数

```yaml
pgo:
  key_pose_delta_deg: 10        # 关键帧角度阈值(度)
  key_pose_delta_trans: 0.3     # 关键帧平移阈值(m)
  loop_search_radius: 5.0       # 回环搜索半径(m)
  loop_score_tresh: 0.3         # 回环匹配分数阈值
```

#### 动态过滤器参数

```yaml
localizer:
  dynamic_filter:
    enable: false                # 是否启用
    motion_threshold: 0.1        # 运动阈值(m)
    history_size: 15             # 历史帧数量
    stability_threshold: 0.8     # 稳定性阈值
```

---

## 🛠️ 工具脚本

### slam_tools.sh 命令总览

```bash
# 系统管理
start       # 启动SLAM系统
stop        # 停止SLAM系统
restart     # 重启SLAM系统
status      # 查看系统状态
build       # 编译系统
clean       # 清理编译文件

# 配置管理
config      # 配置管理（generate|validate|status）
preset      # 应用配置预设（realtime|replay）
network     # 检查网络配置

# 数据管理
record      # 开始录制数据
record-stop # 停止录制
bags        # 管理数据包
save        # 保存地图
maps        # 管理地图文件
view        # 查看地图

# 地图处理
gridmap     # 转换为栅格地图
refine      # 地图精细化处理
reconstruct # 从patches重建整图

# 监控与调试
monitor     # 实时监控
topics      # 显示话题
nodes       # 显示节点
log         # 查看日志
rviz        # 启动RViz

# 优化与测试
optimize    # 手动触发优化
filter      # 动态过滤器功能
test        # 运行测试
benchmark   # 性能基准测试

# 依赖管理
gtsam       # 安装GTSAM库
deps        # 检查依赖状态
fix         # 修复已知问题
```

> ℹ️ **Legacy 脚本**  
> 旧版网络诊断等脚本已移动到 `tools/legacy/`，默认不会被主流程调用，使用前请手动检查并更新参数。

### 使用示例

```bash
# 完整工作流示例
./tools/slam_tools.sh build                    # 编译
./tools/slam_tools.sh network                  # 配置网络
./tools/slam_tools.sh start                    # 启动
./tools/slam_tools.sh record                   # 录制数据
# ... 采集数据 ...
./tools/slam_tools.sh record-stop              # 停止录制
./tools/slam_tools.sh save pgo                 # 保存优化地图
./tools/slam_tools.sh gridmap saved_maps/*.pcd # 转换栅格地图
./tools/slam_tools.sh stop                     # 停止系统
```

---

## 🖥️ 图形界面工具

系统提供两个专业的 PyQt5 图形界面工具，适合非命令行用户使用。

### 🎛️ SLAM 管理面板 (slam_manager_gui.py)

**功能概览：** 集成化的 SLAM 系统控制与监控面板，提供 4 个功能标签页。

#### 启动方式

```bash
# 安装依赖
pip install PyQt5

# 启动 GUI
python3 tools/slam_manager_gui.py
```

#### 核心功能

**1️⃣ 系统控制标签页**
- ✅ 一键启动/停止/重启 SLAM 系统
- ✅ 实时模式/回放模式预设切换
- ✅ 可视化选择 bag 文件进行回放
- ✅ 过滤桥开关控制

**2️⃣ 配置管理标签页**
- ✅ 浏览 `master_config.yaml` 的所有配置模块
- ✅ 可视化编辑核心参数（带类型检查）
- ✅ 智能参数范围限制（基于参数语义）
- ✅ 150+ 参数提示工具提示（悬停显示说明）
- ✅ 保存配置并触发子配置生成
- ✅ 配置验证与状态查询

**3️⃣ 性能监控标签页**
- ✅ 实时订阅 `/fastlio2/performance_metrics`
- ✅ 实时订阅 `/coordinator/metrics`
- ✅ 自动文本缓冲管理（防止内存泄漏）
- ✅ 一键启动/停止监听

**4️⃣ 工具联动标签页**
- ✅ 启动 Gridmap GUI 工具
- ✅ 后续可扩展更多工具集成

#### 性能特性

- 📊 **内存优化**：文本框自动限制 1000 行，长时间运行稳定
- 🚀 **响应优化**：表单缓存机制，配置切换 <10ms
- 🔒 **线程安全**：使用 Qt 信号/槽机制，跨线程 UI 更新无风险
- ⚡ **异步清理**：窗口关闭时使用定时器异步等待线程，无阻塞

#### 使用示例

```bash
# 1. 启动 GUI
python3 tools/slam_manager_gui.py

# 2. 在"系统控制"标签页点击"启动(默认)"

# 3. 在"配置管理"标签页：
#    - 左侧选择模块（如 fastlio2）
#    - 右侧编辑参数（悬停查看说明）
#    - 点击"保存配置"
#    - 点击"生成配置"同步子配置文件

# 4. 在"性能监控"标签页：
#    - 点击"启动监听"查看实时指标
#    - 观察 FAST-LIO2 性能数据
#    - 观察协同优化器状态

# 5. 在"工具联动"标签页：
#    - 点击"打开 Gridmap GUI"处理地图
```

---

### 🗺️ 栅格地图生成工具 (gridmap_gui.py)

**功能概览：** 专业的 PCD 点云到占用栅格地图转换工具，带质量评估与可视化。

#### 启动方式

```bash
# 安装依赖
pip install PyQt5 opencv-python numpy open3d

# 启动 GUI
python3 tools/gridmap_gui.py
```

#### 核心功能

**输入处理**
- ✅ 支持拖拽 PCD 文件或点击选择
- ✅ 自动加载 `saved_maps/.pcd_index.json` 历史记录
- ✅ 实时显示点云统计信息（点数、边界）

**参数配置**
- ✅ 3 种预设：快速/平衡/高质量
- ✅ 可调节分辨率、高度范围、机器人参数
- ✅ 障碍物阈值、膨胀半径自定义
- ✅ 地面优势模式、高度跨度过滤

**地图生成**
- ✅ 三值栅格地图（空闲/占用/未知）
- ✅ 实时进度条显示处理进度
- ✅ 自动保存 PNG + YAML（ROS Nav2 兼容）

**质量评估**
- ✅ 占用率、连通性分析
- ✅ 障碍物密度统计
- ✅ 骨架提取与路径可行性分析
- ✅ 可视化骨架叠加显示

**结果预览**
- ✅ 原始地图 + 骨架叠加双视图
- ✅ 支持鼠标缩放/平移查看细节
- ✅ 一键查看质量指标报告

#### 使用示例

```bash
# 1. 启动 GUI
python3 tools/gridmap_gui.py

# 2. 加载点云：
#    - 拖拽 .pcd 文件到窗口
#    - 或点击"选择 PCD 文件"浏览

# 3. 选择预设：
#    - 快速预览：0.1m分辨率，快速生成
#    - 平衡模式：0.05m分辨率（推荐）
#    - 高质量：0.02m分辨率，细节丰富

# 4. 调整参数（可选）：
#    - 机器人半径：影响通行能力判断
#    - 障碍物阈值：点密度判定为障碍的阈值
#    - 膨胀半径：安全距离扩展

# 5. 生成地图：
#    - 点击"生成栅格地图"
#    - 等待进度条完成
#    - 自动保存并显示质量报告

# 6. 评估质量：
#    - 查看占用率、连通性
#    - 点击"查看骨架"检查拓扑结构
#    - 根据质量指标调整参数重新生成
```

#### 输出文件

```bash
saved_maps/
├── map_YYYYMMDD_HHMMSS.png          # 栅格地图图像
├── map_YYYYMMDD_HHMMSS.yaml         # ROS 地图元数据
└── map_YYYYMMDD_HHMMSS_skeleton.png # 骨架叠加图（可选）
```

---

### 📋 GUI 工具依赖安装

```bash
# 基础依赖
pip install PyQt5

# Gridmap GUI 额外依赖
pip install opencv-python numpy open3d

# 性能监控功能（可选）
sudo apt install ros-humble-rclpy
```

---

### 💡 GUI 工具提示

1. **SLAM 管理面板**适合：
   - 系统调试与参数调优
   - 实时性能监控
   - 非命令行用户操作

2. **Gridmap GUI** 适合：
   - 生成导航地图
   - 地图质量评估
   - 快速参数实验

3. **命令行工具** 适合：
   - 自动化脚本集成
   - 批量处理任务
   - 服务器远程操作

---

## ❓ 常见问题

### Q1: 编译失败，提示找不到GTSAM

```bash
# 解决方案：安装GTSAM
./tools/slam_tools.sh gtsam

# 或检查依赖状态
./tools/slam_tools.sh deps
```

### Q2: 无法连接到雷达

```bash
# 检查网络配置
./tools/slam_tools.sh network

# 确认雷达IP可达
ping 192.168.1.3

# 检查防火墙
sudo ufw status
```

### Q3: 点云数据无法显示

```bash
# 检查话题是否发布
ros2 topic list
ros2 topic echo /livox/lidar --once

# 检查RViz配置
./tools/slam_tools.sh rviz
```

### Q4: SLAM漂移严重

```bash
# 1. 检查IMU标定
# 编辑 config/master_config.yaml 中的 calibration 部分

# 2. 调整FAST-LIO2参数
# 增大 imu_noise.accelerometer 和 gyroscope

# 3. 手动触发优化
./tools/slam_tools.sh optimize
```

### Q5: 地图保存在哪里？

```bash
# 默认保存目录
saved_maps/

# 查看已保存地图
./tools/slam_tools.sh maps

# 地图文件命名格式
mid360_YYYYMMDD_HHMMSS_[lio|pgo|hba].pcd
```

### Q6: 如何禁用动态过滤器？

```bash
# 方法1：启动时禁用
./tools/slam_tools.sh start nofilter

# 方法2：修改配置
vim config/master_config.yaml
# 设置 localizer.dynamic_filter.enable: false

./tools/slam_tools.sh config generate
./tools/slam_tools.sh restart
```

---

## 🎯 性能优化

### 1. 实时性优化

```yaml
# 调整缓冲区大小（config/master_config.yaml）
fastlio2:
  buffer_management:
    max_imu_buffer_size: 8000
    max_lidar_buffer_size: 100

# 降低地图分辨率
fastlio2:
  scan_resolution: 0.3    # 增大值
  map_resolution: 0.4     # 增大值
```

### 2. 精度优化

```yaml
# 增加IESKF迭代次数
fastlio2:
  algorithm:
    ieskf_max_iter: 8     # 默认6

# 增加回环检测敏感度
pgo:
  loop_score_tresh: 0.2   # 降低阈值
```

### 3. 内存优化

```yaml
# 限制点云数量
point_cloud_filter:
  max_points_per_frame: 50000  # 降低值

# 减小地图范围
fastlio2:
  cube_len: 50            # 降低值
  det_range: 20           # 降低值
```

---

## 🤝 贡献指南

我们欢迎各种形式的贡献！

### 如何贡献

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

### 代码规范

- 遵循 ROS2 编码规范
- 使用有意义的变量和函数命名
- 添加必要的注释和文档
- 确保代码通过编译和测试

### 报告问题

请使用 [GitHub Issues](https://github.com/wnfxsa1213/MID360_ROS2_SLAM/issues) 报告问题，包括：
- 详细的问题描述
- 复现步骤
- 系统环境信息
- 相关日志输出

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 🙏 致谢

本项目基于以下优秀的开源项目：

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) - 快速激光惯性里程计
- [Livox SDK](https://github.com/Livox-SDK/livox_ros_driver2) - Livox雷达ROS2驱动
- [GTSAM](https://github.com/borglab/gtsam) - 因子图优化库
- [PCL](https://pointclouds.org/) - 点云处理库

---

## 📮 联系方式

- **项目主页**: https://github.com/wnfxsa1213/MID360_ROS2_SLAM
- **问题反馈**: https://github.com/wnfxsa1213/MID360_ROS2_SLAM/issues

---

<div align="center">

**⭐ 如果这个项目对你有帮助，请给我们一个 Star！⭐**

Made with ❤️ for Robotics Community

</div>
