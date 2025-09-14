# MID360 SLAM系统完整使用说明

## 🏗️ 系统架构概览

### 多层级协同优化架构

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   MID360雷达    │───▶│  Livox ROS驱动   │───▶│  FAST-LIO2核心  │
│  (硬件传感器)    │    │   (数据采集)     │    │   (实时SLAM)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                        │
                       ┌─────────────────────────────────┼─────────────────────────────────┐
                       │                                 ▼                                 │
              ┌─────────────────┐                ┌─────────────────┐                ┌─────────────────┐
              │   PGO优化节点   │◄───────────────│   反馈优化      │───────────────▶│  增强可视化系统  │
              │  (位姿图优化)    │                │  (闭环机制)     │                │   (RViz显示)    │
              └─────────────────┘                └─────────────────┘                └─────────────────┘
                       │                                 │                                 
                       ▼                                 ▼                                 
              ┌─────────────────┐                ┌─────────────────┐                
              │  HBA精细化节点  │                │  LOCALIZER节点  │                
              │  (分层BA优化)   │                │   (重定位)      │                
              └─────────────────┘                └─────────────────┘                
```

## 📋 组件功能详解

### 1. FastLIO2 (核心SLAM节点)

**主要功能:**
- ✅ 实时激光雷达-IMU融合SLAM
- ✅ 高速点云处理和地图构建  
- ✅ 毫秒级位姿估计
- ✅ 接收PGO/Localizer位姿反馈优化

**发布的话题:**
```bash
/fastlio2/lio_odom          # 实时里程计输出
/fastlio2/body_cloud        # 机体坐标系点云
/fastlio2/world_cloud       # 世界坐标系累积地图
/fastlio2/lio_path          # SLAM轨迹路径
/fastlio2/imu_pose_marker   # IMU姿态标记
/fastlio2/performance_metrics # 性能指标
/fastlio2/diagnostics       # 系统诊断信息
```

**提供的服务:**
```bash
/fastlio2/save_maps         # 地图保存服务
/fastlio2/save_poses        # 轨迹保存服务
/fastlio2/update_pose       # 位姿更新服务 (接收优化反馈)
```

### 2. PGO (位姿图优化节点)

**主要功能:**
- ✅ 回环检测和闭合
- ✅ 位姿图全局优化
- ✅ 向FastLIO2反馈优化位姿
- ✅ 为HBA提供优化轨迹数据

**订阅的话题:**
```bash
/fastlio2/body_cloud        # 点云数据用于回环检测
/fastlio2/lio_odom          # 里程计数据获取位姿
```

**发布的话题:**
```bash
/pgo/loop_markers           # 回环检测可视化标记
/pgo/optimized_poses        # 优化后的关键帧位姿 (给HBA)
```

**提供的服务:**
```bash
/pgo/save_maps             # 优化地图保存服务
```

**配置参数:**
```yaml
key_pose_delta_trans: 0.5   # 关键帧平移阈值
key_pose_delta_deg: 10      # 关键帧旋转阈值
loop_search_radius: 1.0     # 回环搜索半径
loop_score_thresh: 0.15     # 回环匹配阈值
enable_pose_feedback: true  # 启用位姿反馈
```

### 3. HBA (分层束调整节点)

**主要功能:**
- ✅ 地图精细化优化
- ✅ 分层束调整算法
- ✅ 智能自动触发机制
- ✅ 监控PGO输出自动优化

**订阅的话题:**
```bash
/pgo/optimized_poses        # 监控PGO优化位姿，触发精细化
```

**发布的话题:**
```bash
/hba/map_points            # HBA优化后的地图点云
```

**提供的服务:**
```bash
/hba/refine_map            # 地图精细化服务
/hba/save_poses            # 优化位姿保存服务
```

**自动触发条件:**
- 关键帧数量达到100个
- 时间间隔超过300秒
- PGO发布新的优化位姿

### 4. Localizer (重定位节点)

**主要功能:**
- ✅ 基于预建地图的ICP重定位
- ✅ 自动地图加载机制
- ✅ 两阶段ICP匹配(粗糙+精细)
- ✅ 向FastLIO2反馈重定位结果

**订阅的话题:**
```bash
/fastlio2/body_cloud        # 实时点云用于ICP匹配
/fastlio2/lio_odom          # 里程计提供位姿初值
```

**发布的话题:**
```bash
/localizer/map_cloud        # 加载的地图点云(可视化)
```

**提供的服务:**
```bash
/localizer/relocalize        # 重定位服务
/localizer/relocalize_check  # 重定位状态检查
/localizer/auto_load_map     # 自动地图重载服务
```

**自动搜索地图路径:**
- `./saved_maps/optimized_map.pcd`
- `./saved_maps/map.pcd`
- `./saved_maps/world_cloud.pcd`
- `../saved_maps/optimized_map.pcd`
- `../saved_maps/map.pcd`

### 5. 可视化系统 (RViz增强)

**显示内容:**
- Body Point Cloud - 实时点云
- Persistent World Map - 持久化世界地图  
- SLAM Trajectory - SLAM轨迹路径
- TF坐标系 - 完整坐标系树
- IMU Pose Marker - IMU姿态标记

## 🔍 系统监控命令

### 基础监控命令

```bash
# 查看所有ROS2节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list
```

### 话题数据监控

```bash
# 监控FastLIO2里程计输出
ros2 topic echo /fastlio2/lio_odom

# 监控FastLIO2轨迹
ros2 topic echo /fastlio2/lio_path

# 监控点云数据
ros2 topic echo /fastlio2/body_cloud --once

# 监控PGO回环标记
ros2 topic echo /pgo/loop_markers --once

# 监控PGO优化位姿(给HBA的)
ros2 topic echo /pgo/optimized_poses --once

# 监控HBA优化点云
ros2 topic echo /hba/map_points --once
```

### 话题频率和带宽监控

```bash
# 查看话题发布频率
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /pgo/optimized_poses

# 查看话题数据带宽
ros2 topic bw /fastlio2/body_cloud
ros2 topic bw /fastlio2/world_cloud
```

### 服务测试命令

```bash
# 测试地图保存服务
ros2 service call /fastlio2/save_maps interface/srv/SaveMaps "{file_path: './saved_maps'}"

# 测试PGO地图保存
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: './saved_maps'}"

# 测试位姿更新服务
ros2 service call /fastlio2/update_pose interface/srv/UpdatePose "{timestamp: 0.0, x: 0.0, y: 0.0, z: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0}"

# 测试HBA地图精细化
ros2 service call /hba/refine_map interface/srv/RefineMap "{file_path: './saved_maps'}"

# 测试Localizer重定位
ros2 service call /localizer/relocalize interface/srv/Relocalize "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 测试自动地图加载
ros2 service call /localizer/auto_load_map interface/srv/IsValid "{code: 0}"
```

### 系统诊断命令

```bash
# 查看节点信息
ros2 node info /fastlio2_node
ros2 node info /pgo_node
ros2 node info /hba_node
ros2 node info /localizer_node

# 查看TF树
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map body

# 查看参数
ros2 param list /fastlio2_node
ros2 param get /pgo_node config_path
```

## 🏠 室内建图完整使用流程

### 前期准备

#### 1. 环境检查
```bash
# 检查网络连接 (MID360默认IP: 192.168.1.3)
ping 192.168.1.3

# 检查ROS2环境
echo $ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
```

#### 2. 系统编译
```bash
cd /home/tianyu/codes/Mid360map

# 修复已知问题
./tools/slam_tools.sh fix

# 编译系统 (按依赖顺序)
./tools/slam_tools.sh build

# 生成统一配置文件
python3 tools/config_generator.py config/slam_master_config.yaml
```

### 室内建图操作流程

#### Step 1: 系统启动

```bash
# 进入工作空间
cd /home/tianyu/codes/Mid360map/ws_livox
source install/setup.bash

# 启动完整SLAM系统 (推荐方式)
ros2 launch fastlio2 enhanced_visualization.launch.py
```

**启动顺序和时序:**
- 0s: Livox驱动 + FastLIO2 + 静态TF
- 3s: RViz可视化界面
- 8s: PGO位姿图优化
- 10s: HBA分层束调整  
- 12s: Localizer重定位

#### Step 2: 系统状态检查

```bash
# 新开终端，检查所有节点运行状态
ros2 node list
# 应该看到:
# /fastlio2_node
# /pgo_node  
# /hba_node
# /localizer_node
# /livox_lidar_publisher

# 检查关键话题
ros2 topic list | grep -E "(fastlio2|pgo|hba)"
# 应该看到:
# /fastlio2/lio_odom
# /fastlio2/body_cloud  
# /fastlio2/world_cloud
# /pgo/loop_markers
# /pgo/optimized_poses
```

#### Step 3: 初始化阶段 (0-30秒)

**MID360雷达预热:**
- 等待雷达稳定输出点云 (约10秒)
- 观察RViz中点云数据是否正常显示

**FastLIO2初始化:**
```bash
# 监控里程计输出
ros2 topic hz /fastlio2/lio_odom
# 应该看到 ~50Hz 的稳定输出

# 检查IMU数据
ros2 topic hz /livox/imu  
# 应该看到 ~200Hz 的高频输出
```

**系统健康检查:**
```bash
# 检查诊断信息
ros2 topic echo /fastlio2/diagnostics --once
```

#### Step 4: 室内建图执行 (30秒后开始)

**建图策略:**

1. **起始点设置**: 在房间中央固定位置启动，保持2-3秒静止
2. **路径规划**: 沿房间边缘慢速移动 (0.3-0.5 m/s)
3. **覆盖策略**: 
   - 房间外围一圈
   - 内部"之"字形扫描
   - 重要区域多次经过

**运动要求:**
- 移动速度: 0.2-0.8 m/s (室内推荐0.3 m/s)
- 转向速度: < 30°/s  
- 避免剧烈振动和急停急转

#### Step 5: 实时监控建图质量

**点云质量监控:**
```bash
# 监控实时点云
ros2 topic echo /fastlio2/body_cloud --once | head -20

# 监控世界地图累积
ros2 topic echo /fastlio2/world_cloud --once | head -10
```

**轨迹质量检查:**
```bash
# 监控轨迹平滑度
ros2 topic echo /fastlio2/lio_path --once | tail -20
```

**PGO回环检测:**
```bash
# 监控回环检测
ros2 topic echo /pgo/loop_markers --once

# 检查PGO工作状态  
ros2 topic hz /pgo/optimized_poses
```

#### Step 6: 回环闭合 (重要步骤)

**回环条件:**
- 回到起始位置附近 (1米内)
- 在相同位置停留3-5秒
- 观察RViz中是否出现回环标记

**回环验证:**
```bash
# 检查回环检测结果
ros2 topic echo /pgo/loop_markers | head -50

# 验证位姿反馈
ros2 service call /fastlio2/update_pose interface/srv/UpdatePose "{timestamp: 0.0, x: 0.0, y: 0.0, z: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0}"
```

#### Step 7: 建图完成和保存

**停止建图:**
- 回到起始位置
- 保持静止5-10秒让系统稳定

**多层级地图保存策略:**

##### 方式1: 快速保存 (推荐日常使用)
```bash
# 使用slam_tools.sh快速保存
./tools/slam_tools.sh save
# 优点: 1-3秒快速完成，实时备份
# 缺点: 原始SLAM数据，可能有累积误差
```

##### 方式2: PGO优化保存 (推荐大范围建图)
```bash
# 检查是否有回环检测
ros2 topic echo /pgo/loop_markers --once

# 保存PGO全局优化地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: './saved_maps'}"
# 优点: 消除累积误差，全局一致性好
# 缺点: 需要有效回环检测，5-15秒处理时间
```

##### 方式3: HBA精细化保存 (推荐高精度需求)
```bash
# 保存HBA分层束调整精细化地图
ros2 service call /hba/refine_map interface/srv/RefineMap "{file_path: './saved_maps'}"
# 优点: 亚毫米级精度，几何结构优化
# 缺点: 30秒-5分钟处理时间，内存消耗大
```

##### 方式4: FastLIO2+HBA联合保存 (推荐生产环境)
```bash
# 启用HBA在线精细化的FastLIO2保存
ros2 service call /fastlio2/save_maps interface/srv/SaveMaps \
    "{file_path: './saved_maps/enhanced_map.pcd', save_patches: false, enable_hba_refine: true}"
# 优点: 平衡精度与速度，一步完成
# 缺点: 需要HBA节点运行，10-30秒处理
```

**智能保存策略 (推荐):**
```bash
# 创建智能保存脚本
cat > smart_save_maps.sh << 'EOF'
#!/bin/bash
echo "🗺️ 智能地图保存开始..."

# 1. 快速保存原始地图
echo "📁 保存原始SLAM地图..."
./tools/slam_tools.sh save

# 2. 检查回环并保存优化地图
if ros2 topic echo /pgo/loop_markers --once 2>/dev/null | grep -q "markers"; then
    echo "🔄 检测到回环，保存PGO优化地图..."
    ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: './saved_maps'}"
    
    echo "⚡ 执行HBA精细化优化..."
    ros2 service call /hba/refine_map interface/srv/RefineMap "{file_path: './saved_maps'}"
    echo "✅ 完成高质量地图保存"
else
    echo "⚠️ 未检测到回环，仅保存原始地图"
fi
EOF

chmod +x smart_save_maps.sh
./smart_save_maps.sh
```

**验证保存结果:**
```bash
# 检查保存的文件
ls -la saved_maps/
# 可能包含:
# mid360_map_YYYYMMDD_HHMMSS.pcd (slam_tools.sh保存的原始地图)
# optimized_map.pcd (PGO全局优化地图)  
# refined_map.pcd (HBA精细化地图)
# enhanced_map.pcd (FastLIO2+HBA联合地图)
# trajectory.txt (轨迹文件)

# 检查地图质量
python3 tools/rviz_map_viewer.py saved_maps/optimized_map.pcd
```

### 结果评估和后处理

#### 1. 地图质量评估

**使用点云处理工具:**
```bash
# 使用高级点云处理器
python3 tools/advanced_point_cloud_processor.py saved_maps/optimized_map.pcd

# 启用表面重建
python3 tools/advanced_point_cloud_processor.py saved_maps/optimized_map.pcd \
    --surface-reconstruction \
    --reconstruction-method poisson
```

**使用GUI界面:**
```bash
# 启动GUI处理工具
./tools/start_gui.sh
# 或
python3 tools/point_cloud_processor_gui.py
```

#### 2. 地图可视化

**离线查看地图:**
```bash
# 使用RViz查看器
python3 tools/rviz_map_viewer.py saved_maps/optimized_map.pcd

# 在新终端启动RViz
rviz2 -d tools/rviz/saved_map_viewer.rviz
```

#### 3. 重定位测试 (可选)

**自动加载已建地图:**
```bash
# 重新启动Localizer (地图会自动加载)
ros2 run localizer localizer_node --ros-args --params-file ws_livox/src/localizer/config/localizer.yaml

# 手动触发地图重载
ros2 service call /localizer/auto_load_map interface/srv/IsValid "{code: 0}"
```

**测试重定位功能:**
```bash
# 在已知位置触发重定位
ros2 service call /localizer/relocalize interface/srv/Relocalize \
    "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 故障排除指南

#### 常见问题和解决方案

**1. 雷达连接问题**
```bash
# 检查网络
ping 192.168.1.3

# 检查驱动状态
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

**2. FastLIO2无输出**
```bash
# 检查配置文件
cat ws_livox/src/fastlio2/config/lio.yaml

# 重新生成配置
python3 tools/config_generator.py config/slam_master_config.yaml
```

**3. PGO不工作**  
```bash
# 检查PGO配置
cat ws_livox/src/pgo/config/pgo.yaml

# 检查回环检测参数
ros2 param get /pgo_node enable_pose_feedback
```

**4. 地图保存失败**
```bash
# 检查目录权限
ls -la saved_maps/
mkdir -p saved_maps
chmod 755 saved_maps/
```

### 性能优化建议

#### 室内建图特定优化

**参数调整 (config/slam_master_config.yaml):**
```yaml
global_params:
  scan_resolution: 0.1        # 室内可提高精度
  map_resolution: 0.15        # 适合室内细节

fastlio2:
  lidar_params:
    max_range: 30.0          # 室内范围限制
    filter_num: 1            # 减少过滤保留细节
    
pgo:
  loop_closure:
    search_radius: 2.0       # 室内增大搜索范围
    time_threshold: 30.0     # 缩短时间阈值
```

**系统资源优化:**
```yaml
advanced:
  threading:
    num_threads: 8           # 根据CPU核心数调整
  memory:
    max_cache_size: 8192     # 室内建图减少内存使用
```

## 📊 监控面板设置

### 推荐监控设置

创建监控脚本 `monitor_indoor_slam.sh`:
```bash
#!/bin/bash

echo "=== MID360 室内SLAM监控面板 ==="
echo "启动时间: $(date)"
echo

# 检查节点状态
echo "📡 系统节点状态:"
ros2 node list | grep -E "(fastlio2|pgo|hba|localizer|livox)" | while read node; do
    echo "  ✅ $node"
done
echo

# 检查话题频率
echo "📈 关键话题频率:"
echo "  FastLIO2里程计: $(ros2 topic hz /fastlio2/lio_odom --window 10 2>/dev/null | head -1 || echo '未运行')"
echo "  点云数据: $(ros2 topic hz /fastlio2/body_cloud --window 5 2>/dev/null | head -1 || echo '未运行')"
echo "  PGO优化: $(ros2 topic hz /pgo/optimized_poses --window 5 2>/dev/null | head -1 || echo '未运行')"
echo

# 检查服务状态
echo "🔧 服务可用性:"
services=("/fastlio2/save_maps" "/pgo/save_maps" "/hba/refine_map" "/localizer/relocalize")
for service in "${services[@]}"; do
    if ros2 service list | grep -q "$service"; then
        echo "  ✅ $service"
    else
        echo "  ❌ $service (不可用)"
    fi
done
echo

echo "使用 'watch -n 2 ./monitor_indoor_slam.sh' 实现实时监控"
```

## 📝 建图日志记录

### 自动化日志记录

创建建图会话记录脚本 `start_mapping_session.sh`:
```bash
#!/bin/bash

SESSION_ID=$(date +%Y%m%d_%H%M%S)
LOG_DIR="mapping_logs/$SESSION_ID"
mkdir -p "$LOG_DIR"

echo "🗺️ 开始室内建图会话: $SESSION_ID"
echo "📁 日志目录: $LOG_DIR"

# 记录系统状态
ros2 node list > "$LOG_DIR/nodes.txt"
ros2 topic list > "$LOG_DIR/topics.txt"  
ros2 service list > "$LOG_DIR/services.txt"

# 记录配置
cp config/slam_master_config.yaml "$LOG_DIR/"
cp -r ws_livox/src/*/config/*.yaml "$LOG_DIR/"

# 开始录制rosbag
ros2 bag record -a -o "$LOG_DIR/rosbag" &
BAG_PID=$!

echo "📹 开始录制rosbag (PID: $BAG_PID)"
echo "🛑 按任意键停止建图会话..."
read -n 1

# 停止录制
kill $BAG_PID
echo "🎬 建图会话结束，日志保存在: $LOG_DIR"
```

---

## 🎯 总结

本文档提供了MID360 SLAM系统的完整使用指南，包括：

1. **系统架构**: 多层级协同优化的完整架构
2. **组件功能**: 每个组件的详细功能和接口
3. **监控命令**: 全面的系统状态监控方法  
4. **室内建图流程**: 从准备到完成的完整操作流程
5. **故障排除**: 常见问题的诊断和解决方案
6. **性能优化**: 针对室内环境的参数调优建议

该系统是一个**企业级多层级协同优化SLAM系统**，具备完整的实时建图、位姿图优化、精细化处理和重定位功能。正确使用本指南，可以获得高质量的室内环境三维地图。


 # 1. 设置ROS2环境
  cd /home/tianyu/codes/Mid360map
  source ws_livox/install/setup.bash

  # 2. 创建录制目录
  RECORD_DIR="./recorded_bags/manual_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$RECORD_DIR"

  # 3. 检查可用话题
  ros2 topic list | grep -E "(livox|fastlio2)"

  # 4. 手动录制（基础版本 - 仅传感器数据）
  ros2 bag record /livox/imu /livox/lidar -o "$RECORD_DIR"

  # 5. 手动录制（完整版本 - 包含SLAM数据）
  ros2 bag record \
    /livox/imu \
    /livox/lidar \
    /fastlio2/lio_odom \
    /fastlio2/body_cloud \
    /fastlio2/lio_path \
    /fastlio2/world_cloud \
    -o "$RECORD_DIR"
