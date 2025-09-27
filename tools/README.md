# SLAM系统管理工具

本目录包含MID360 FASTLIO2_ROS2 SLAM系统的管理工具。

## 工具说明

### 主要脚本

1. **start_slam.sh** - SLAM系统启动脚本
   - 自动检查网络配置
   - 按顺序启动各个组件
   - 实时状态监控和错误处理

2. **stop_slam.sh** - 系统安全停止脚本
   - 优雅关闭所有进程
   - 清理临时文件和PID文件

3. **check_slam.sh** - 系统状态检查工具
   - 网络连接测试
   - ROS进程监控
   - 话题状态检查
   - 数据流测试

4. **slam_tools.sh** - 综合管理工具集
   - 一键式系统管理
   - 多种调试和监控功能
   - 保存/重建地图与轨迹（LIO/PGO/HBA 集成）

### 使用方法

```bash
# 基本操作
./start_slam.sh    # 启动系统
./stop_slam.sh     # 停止系统  
./check_slam.sh    # 检查状态

# 工具集使用
./slam_tools.sh help      # 查看帮助
./slam_tools.sh restart   # 重启系统
./slam_tools.sh monitor   # 监控数据流
./slam_tools.sh network   # 网络诊断

# 一步到位的启动子命令
./slam_tools.sh start realtime                                  # 应用实机预设并启动
./slam_tools.sh start replay --bag recorded_data/run1 --rate 1.0 # 应用回放预设并自动回放
./slam_tools.sh start replay --bag recorded_data/run1 \
    --topics "/livox/imu /livox/lidar" \
    --remap "/fastlio2/lio_odom:=/bag/lio_odom"             # 仅播放指定话题并避开冲突
./slam_tools.sh start replay                                     # 仅应用回放预设并启动（不自动播包）

注意：旧命令 `replay` / `replay-bag` 已废弃，请统一使用 `start replay`。

# 保存/查看/重建地图
./slam_tools.sh save                         # 保存 LIO 地图+轨迹、PGO 优化地图、HBA 轨迹
./slam_tools.sh view saved_maps/xxx.pcd --rviz  # 在RViz中查看 PCD（配置已预置 Saved Map 显示）
./slam_tools.sh reconstruct --pgo-dir saved_maps/run1_pgo --output saved_maps/run1_recon.pcd
./slam_tools.sh reconstruct --pgo-dir saved_maps/run1_pgo --poses saved_maps/run1_hba_poses.txt --output saved_maps/run1_final.pcd

# 预设切换（一键调整 master_config 并生成各包配置）
# 实机扫描（过滤桥开，PGO吃 /slam/body_cloud，Localizer过滤关）
./slam_tools.sh preset realtime
# 回放复现（过滤桥关，Localizer过滤开，PGO吃 /localizer/filtered_cloud，放宽回环参数）
./slam_tools.sh preset replay
```

### 权限要求

所有脚本都需要可执行权限：
```bash
chmod +x *.sh
```

## 新增工具

- `tools/save_maps_bundle.py`
  - 一次性保存：
    - LIO 累积地图（pcd/ply 可选）与 LIO 轨迹
    - PGO 优化地图（目录：map.pcd/poses.txt/patches/）
    - HBA 优化轨迹
  - 用法：`python3 tools/save_maps_bundle.py -o saved_maps -n run1 -f pcd`

- `tools/reconstruct_map_from_patches.py`
  - 使用 PGO 导出的 patches + poses 离线重建整图
  - 支持用 HBA 的位姿替换 PGO 位姿后重建
  - 用法：
    - `python3 tools/reconstruct_map_from_patches.py --pgo-dir saved_maps/run1_pgo --output saved_maps/run1_reconstructed.pcd`
    - `python3 tools/reconstruct_map_from_patches.py --pgo-dir saved_maps/run1_pgo --poses saved_maps/run1_hba_poses.txt --output saved_maps/run1_final.pcd`

提示：RViz 增强配置已预置 `Saved Map` 显示（话题 `/saved_map`），配合 `tools/publish_saved_map.py` 可快速查看保存的地图。
