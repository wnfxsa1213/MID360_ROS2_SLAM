# SLAM系统管理工具

本目录包含MID360 FASTLIO2_ROS2 SLAM系统的管理工具。

## 工具说明

### 主要脚本

**slam_tools.sh** - 综合管理工具集（统一入口）
- 一键式系统管理（启动/停止/重启/状态检查）
- 多种调试和监控功能
- 保存/重建地图与轨迹（LIO/PGO/HBA 集成）
- 配置管理、网络诊断、数据记录与回放

### 使用方法

```bash
# 基本操作
./slam_tools.sh help      # 查看完整帮助
./slam_tools.sh start     # 启动系统
./slam_tools.sh stop      # 停止系统
./slam_tools.sh status    # 检查状态
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

## 关键 Python 工具

- `save_maps_bundle.py` / `save_map_lio_only.py`：通过服务接口保存地图与轨迹（bundle 版本保存完整数据，lio_only 版本仅保存 LIO 地图）。
- `reconstruct_map_from_patches.py`：用 PGO/HBA 结果离线重建全图。
- `pcd_to_gridmap.py`：增强版 PCD → 栅格地图转换器，配合 `gridmap_gui.py`（PyQt GUI）和 `view_gridmap.py` 浏览结果。
- `map_refinement.py`：全流程点云精修管线，依赖 `map_refinement/` 子包。
  - **性能优化**：
    - `performance.intermediate_io_workers`：异步写入中间结果的线程数（默认 1，设为 0 则改为同步写入），避免 I/O 阻塞主流程。
    - `performance.batch_workers`：批处理时的并行进程数，>1 时对每个输入文件派生独立进程并行处理。
  - **密度模式**：`density_uniformization.mode` 支持 `kd_tree`（共享KDTree）、`voxel`（体素近似）、`hybrid`（Open3D Hybrid）、`gpu`（CuPy+cuML），并可通过 `voxel_density_downsample`、`voxel_neighbor_levels`、`hybrid_knn` 等参数细调准确度与性能。
  - **配置缓存**：YAML 配置文件会根据修改时间自动缓存，避免重复解析。
  - **内存优化**：点云拷贝优先使用 `clone()` 方法（若支持），大幅降低百万级点云的内存开销。
  - **索引修复**：修复了 `edge_enhancer.py` 中 set 转列表导致的索引错位问题，提升长流程稳定性。
- `slam_manager_gui.py`：基于 PyQt5 的管理面板原型，可启动/停止系统、编辑 master 配置、查看性能指标、联动 gridmap GUI。
- `performance_analyzer.py` / `comprehensive_performance_report.py`：分别用于实时性能采集与动态滤波器离线评估。
- `record_data.py` / `replay_validation.py` 等测试脚本：支持数据采集、回放验证。

## Legacy 工具（仅参考）

为避免误用的历史脚本已迁至 `tools/legacy/`：

- `network_detector.py`：旧的网卡探测脚本，推荐使用新的网络配置流程。
- `network_performance_tester.py`：带硬编码 IP/网卡参数的网络测试器，如需使用请自行配置。
- `test_with_bag.sh`：bag 测试脚本现已支持通过 `BAG_PATH`、`RVIZ_CONFIG` 环境变量指定资源，否则读取 `data/rosbags/`。
- `pcd_to_gridmap_backup.py`：早期的 PCD→栅格转换脚本已移除，统一使用增强版 `pcd_to_gridmap.py`。
- **已迁移 (2025-10-20)**：`start_slam.sh`, `stop_slam.sh`, `check_slam.sh` - 这些独立启动脚本的功能已完全集成到 `slam_tools.sh`，请使用 `./slam_tools.sh start|stop|status` 替代。

这些脚本默认不会被主流程调用，使用前请确认参数并根据实际环境调整。

## GUI 原型

```bash
python3 tools/slam_manager_gui.py
```

> 依赖 PyQt5。如需性能监控，请额外安装 `rclpy`（`sudo apt install ros-humble-rclpy`）。当前原型支持：
> - 一键 start/stop/restart/status、实时/回放预设、bag 回放；
> - 在 GUI 中浏览/编辑 `master_config.yaml` 主要参数并保存；
> - 调用 `slam_tools.sh config generate|validate|status`；
> - 订阅 `/fastlio2/performance_metrics` 和 `/coordinator/metrics` 观察实时指标；
> - 直接启动 `gridmap_gui`；所有命令输出汇总到日志窗口。
