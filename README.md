# MID360 激光雷达实时SLAM建图系统

## 🚀 项目简介

这是一个基于Livox MID360激光雷达的高精度实时SLAM（同时定位与建图）系统，专为穿越机和无人车等移动平台设计。系统采用光纤传输技术实现超低延迟数据传输，结合先进的FAST-LIO2算法进行实时建图。

### ✨ 核心特点

- 🎯 **实时性能**: 基于FAST-LIO2算法，实现毫秒级定位更新
- 🌐 **大范围建图**: 支持2km+半径的超大范围地图构建  
- 🔄 **回环检测**: 集成PGO位姿图优化，消除累积误差
- 📡 **光纤传输**: 超低延迟数据传输，适合高速移动平台
- ⚙️ **统一配置**: v2.0配置管理系统，参数一致性保证
- 🔧 **模块化**: 支持FastLIO2、PGO、HBA、Localizer独立启用

## 📋 系统要求

- **操作系统**: Ubuntu 20.04/22.04 LTS
- **ROS版本**: ROS2 Humble
- **编译器**: C++17 支持 (GCC 7.0+)
- **硬件要求**: 
  - 内存: 8GB+ (推荐16GB)
  - CPU: 4核心+ (推荐8核心)
  - 存储: 50GB+ 可用空间

### 🔗 依赖库

| 组件 | 版本要求 | 用途 |
|------|----------|------|
| PCL | 1.10+ | 点云处理 |
| Eigen3 | 3.3+ | 矩阵运算 |
| GTSAM | 4.1+ | PGO/HBA优化 (可选) |
| OpenMP | - | 并行计算加速 |

## 🛠️ 快速开始

### 1. 一键安装和编译

```bash
# 克隆项目
git clone <项目地址>
cd Mid360map

# 修复已知问题并编译
./tools/slam_tools.sh fix
./tools/slam_tools.sh build

# 生成统一配置文件
./tools/slam_tools.sh config-gen
```

### 2. 系统启动

```bash
# 启动完整SLAM系统 (推荐)
./tools/slam_tools.sh start

# 或仅启动基础SLAM
./tools/slam_tools.sh start-basic
```

### 3. 系统监控

```bash
# 查看系统状态
./tools/slam_tools.sh status

# 实时监控数据流
./tools/slam_tools.sh monitor

# 查看配置状态  
./tools/slam_tools.sh config
```

## 🏗️ 系统架构

### 核心组件

```
MID360雷达 ──→ UDP数据接收 ──→ FastLIO2 ──→ PGO/HBA/Localizer ──→ RViz可视化
    ↓              ↓            ↓              ↓              ↓
  原始点云      光纤传输优化    实时SLAM      后处理优化      建图结果展示
```

### 模块详情

| 组件 | 功能 | 状态 | 依赖 |
|------|------|------|------|
| **FastLIO2** | 核心SLAM算法 | ✅ 已实现 | 必需 |
| **PGO** | 位姿图优化 | ✅ 已实现 | GTSAM |
| **HBA** | 分层束调整 | ✅ 已实现 | GTSAM |
| **Localizer** | ICP定位 | ✅ 已实现 | PCL |
| **可视化系统** | RViz增强界面 | ✅ 已实现 | - |

## ⚙️ 配置管理

### v2.0 统一配置系统

系统采用全新的统一配置管理架构：

```
config/slam_master_config.yaml (主配置)
    ↓
tools/config_generator.py (自动生成器)
    ↓
各组件配置文件 (自动同步)
```

### 配置操作

```bash
# 查看配置状态
./tools/slam_tools.sh config

# 生成组件配置
./tools/slam_tools.sh config-gen  

# 验证配置一致性
./tools/slam_tools.sh config-val
```

### 核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_resolution` | 0.05m | 统一扫描分辨率 |
| `map_resolution` | 0.08m | 地图分辨率 |
| `cube_len` | 2000m | 局部地图范围 |
| `topic_prefix` | `/fastlio2` | 统一话题前缀 |

## 📊 数据流结构

### ROS2话题

**输入话题:**
- `/livox/lidar` - MID360点云数据
- `/livox/imu` - IMU惯性数据

**输出话题:**
- `/fastlio2/lio_odom` - 实时里程计
- `/fastlio2/body_cloud` - 处理后点云
- `/fastlio2/lio_path` - SLAM轨迹
- `/fastlio2/world_cloud` - 全局地图

### 坐标系

```
map (世界坐标系)
 ↓
lidar (本地坐标系)  
 ↓
body (机体坐标系)
 ↓
livox_frame (雷达坐标系)
```

## 🔧 故障排除

### 常见问题

**编译错误:**
```bash
./tools/slam_tools.sh fix    # 修复已知问题
./tools/slam_tools.sh clean  # 清理重编译
```

**网络连接:**
```bash
./tools/slam_tools.sh network  # 检查网络配置
ping 192.168.1.3              # 测试雷达连接
```

**配置不一致:**
```bash
./tools/slam_tools.sh config-gen  # 重新生成配置
./tools/slam_tools.sh config-val  # 验证一致性
```

### 日志和调试

```bash
./tools/slam_tools.sh log      # 查看系统日志
./tools/slam_tools.sh topics   # 查看活跃话题
./tools/slam_tools.sh nodes    # 查看运行节点
```

## 💾 地图保存

### 保存地图

```bash
# 保存当前地图和轨迹
./tools/slam_tools.sh save

# 查看保存的地图
./tools/slam_tools.sh view <地图文件.pcd>

# 管理地图文件
./tools/slam_tools.sh maps
```

### 地图格式

- **点云地图**: `.pcd` 格式 (PCL标准)
- **轨迹文件**: `.txt` 格式 (xyz坐标序列)
- **优化地图**: PGO/HBA优化后的结果

## 📈 性能优化

### 系统调优

```bash
# 检查依赖状态
./tools/slam_tools.sh deps

# 安装性能库
./tools/slam_tools.sh gtsam  # 安装GTSAM优化库
```

### 参数调优

主要性能参数 (在 `config/slam_master_config.yaml` 中):

- `scan_resolution`: 降低值提高精度但增加计算量
- `cube_len`: 控制内存使用和建图范围
- `ieskf_max_iter`: 平衡精度和实时性

## 📚 开发指南

### 添加新组件

1. 在 `ws_livox/src/` 下创建组件包
2. 在 `slam_master_config.yaml` 中添加配置段
3. 更新 `config_generator.py` 支持新组件
4. 修改统一启动文件 `unified_slam_launch.py`

### 配置管理

所有配置修改都应该在主配置文件中进行：

```bash
# 编辑主配置
nano config/slam_master_config.yaml

# 重新生成组件配置
./tools/slam_tools.sh config-gen
```

## 📄 许可证

MIT License - 详见 `LICENSE` 文件

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

### 报告问题

1. 运行 `./tools/slam_tools.sh status` 获取系统状态
2. 收集相关日志: `./tools/slam_tools.sh log`
3. 在 GitHub Issues 中描述问题

### 开发流程

1. Fork 项目
2. 创建特性分支
3. 编写代码和测试
4. 提交 Pull Request

## 📞 技术支持

- 📖 文档: 查看 `docs/` 目录
- 🐛 Bug报告: GitHub Issues
- 💡 功能建议: GitHub Discussions

---

**⚡ 快速命令参考:**
```bash
./tools/slam_tools.sh help     # 显示所有可用命令
./tools/slam_tools.sh fix      # 修复编译问题
./tools/slam_tools.sh build    # 编译系统  
./tools/slam_tools.sh start    # 启动SLAM
./tools/slam_tools.sh status   # 检查状态
./tools/slam_tools.sh save     # 保存地图
```