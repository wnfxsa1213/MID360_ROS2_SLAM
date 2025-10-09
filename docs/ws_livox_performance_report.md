# WS Livox 性能问题审查报告

## 审查范围
- 启动链路：`ws_livox/src/fastlio2/launch/cooperative_slam_system.launch.py`
- 点云过滤桥：`ws_livox/src/point_cloud_filter`
- FAST-LIO2 主节点：`ws_livox/src/fastlio2/src`
- 协同优化节点：`ws_livox/src/cooperation/src/optimization_coordinator.cpp`
- 回环 / HBA / Localizer 相关配置

## 关键症状
- 实时建图在 RViz (`enhanced_fastlio2.rviz`) 中先正常后剧烈飘移。
- 启用点云过滤桥后，`/livox/lidar_filtered` 每帧点数从预期的 3–4 万骤降到约 4 千。

## 重点问题总览
- CRITICAL：3 项
- MAJOR：2 项
- MINOR：2 项

## 详细问题列表
| 严重级别 | 文件 (行) | 问题描述 | 性能影响 | 修复建议 |
| --- | --- | --- | --- | --- |
| CRITICAL | `ws_livox/src/point_cloud_filter/src/point_cloud_filter_bridge.cpp:284-302` | 重新封装 `CustomMsg` 时将 `offset_time/tag/line` 直接置零，导致 FAST-LIO2 无法使用扫描内时间戳做去畸变与同步。 | IMU/LiDAR 时间对齐丢失，系统数秒后姿态估计严重漂移。 | 在封包时保留原始点字段，至少传递 `offset_time`，并考虑保留 `tag/line`。 |
| CRITICAL | `ws_livox/src/point_cloud_filter/config/point_cloud_filter.yaml:1-22`<br>`ws_livox/src/localizer/src/localizers/dynamic_object_filter.cpp:721-733` | 默认 `voxel_size_base=0.05`，动态滤波器先体素降采样导致点云只剩 ~4k；且历史窗口 15 帧在高频场景下进一步扩大滤波力度。 | 点云密度骤降、有效特征不足，LIO 匹配失败。 | 将体素大小恢复至 0.01 左右，必要时调低 `history_size` 和 `stability_threshold`，确保过滤后仍有 ≥30k 点。 |
| CRITICAL | `ws_livox/src/cooperation/src/optimization_coordinator.cpp:442-449` | 协调器把 `Float64MultiArray[2]`（实际为 IMU 缓冲大小）误当 fitness score，缓冲稍大即累积漂移并触发紧急 PGO/HBA。 | 系统频繁抢占资源做优化，CPU 抢占、轨迹卡顿放大漂移。 | 调整订阅协议，新增真实质量指标字段或改用 publish 的第 0 位处理时间；对漂移指标做低通与阈值保护。 |
| MAJOR | `ws_livox/src/fastlio2/launch/cooperative_slam_system.launch.py:102-140` | 启动参数内联将 `max_processing_hz` 固定为 10 Hz，覆盖 YAML 默认 20 Hz。10 Hz 限流导致真实 10 Hz 激光一旦抖到 11 Hz 即被丢帧。 | 有效历史帧不足、动态判定失真，点云进一步稀疏。 | 放开限频（≥15 Hz）或改回完全使用参数文件；必要时根据真实频率自适应限流。 |
| MAJOR | `ws_livox/src/fastlio2/src/lio_node.cpp:381-403` | `publishPath` 每 10 ms 将整个路径重新发布并累计 poses，不做截断。长时间运行路径数组膨胀，CPU/网络压力陡增。 | RViz 与优化回路被大消息阻塞，表现为“突然卡顿后轨迹乱跑”。 | 引入长度上限或抽样发布（例如仅保留最近 N 米 / N 帧），或改为增量式 Path。 |
| MINOR | `ws_livox/src/point_cloud_filter/src/point_cloud_filter_bridge.cpp:118-164` | `dynamic_filter.downsample_ratio` 参数未被使用，实际只看 `voxel_size_base` 与 `max_points_per_frame`。 | 参数形同虚设，难以调优过滤强度。 | 代码补齐 ratio 逻辑，或从配置中移除该字段避免误导。 |
| MINOR | `ws_livox/src/cooperation/src/optimization_coordinator.cpp:200-238` | 任务队列使用 FIFO `std::queue`，优先级字段没有实际排序，紧急任务可能排到队尾。 | 紧急优化响应延迟，漂移控制不稳定。 | 换成 `std::priority_queue` 或手动插入排序，确保高优任务优先执行。 |

## 建议整改优先级
1. **修复点云消息字段 + 调整过滤参数**：确保 `/livox/lidar_filtered` 恢复 3–4 万点并保留时序信息，验证 FAST-LIO2 去畸变是否回归稳定。
2. **校正协同优化指标**：避免误触发紧急优化，观察 CPU 占用和轨迹是否恢复平稳。
3. **优化 FAST-LIO2 发布策略**：限制 Path 消息增长、合理配置限流，排查剩余卡顿来源。
4. **完善过滤桥参数逻辑**：补齐 `downsample_ratio`、确认限流策略与实际雷达频率匹配。
5. **整理任务调度**：重构协调器队列，确保优先级真正生效。

## 后续验证建议
- 调整后通过真实实时数据验证 `/livox/lidar_filtered` 点数、滤波比率和 FAST-LIO2 `performance_metrics`。
- 记录优化触发日志，确认漂移阈值与指标正确关联。
- 在 RViz 中连续运行 ≥10 分钟，验证轨迹无跳变、CPU/内存无异常增长。
- 更新 `docs/config_master_reference.md` 与相关 README，说明关键参数修改及调试流程。

## 已落实修复项概览
- 过滤桥重新封包时保留 Livox 点的最近邻元数据，恢复 `offset_time/tag/line`，并改用消息时间戳驱动动态滤波。
- 调整过滤参数（`voxel_size_base=0.01`、`history_size=8` 等）并取消 launch 中的 10 Hz 硬限，维持 20 Hz 处理能力。
- 协调器漂移指标改用处理耗时/点数等真实信号并加入低通滤波，同时以优先队列调度紧急任务。
- FAST-LIO2 将路径长度限制在最近 2000 帧，并对全局地图发布增加 1 s 节流，抑制消息膨胀。
