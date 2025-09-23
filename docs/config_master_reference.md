# MID360 SLAM 主配置文件（master_config.yaml）参数详解

本文档详细解释 `config/master_config.yaml` 中的每个配置段与关键参数，说明其含义、单位、当前取值、推荐范围，以及相互之间的重要约束关系。所有组件的运行时配置（如 `ws_livox/src/fastlio2/config/lio.yaml` 等）均由本主配置通过 `tools/config_generator.py` 生成，建议仅在主配置中修改参数，并使用生成器同步到各子配置。

参考文件：`config/master_config.yaml`

---

## 1. system（系统信息）
- name/version/description：标注系统名称、版本与说明，不参与算法逻辑。

## 2. network（网络）
- host（主机）
  - ip/interface/gateway/netmask：主机网卡与网络参数，用于与雷达设备通信。
- lidar（雷达）
  - ip：雷达设备 IP。
  - lidar_ports/host_ports：雷达端与主机端对应端口（命令、点云、IMU、日志）。Livox 驱动配置由此生成。

## 3. calibration（硬件标定）
- lidar_to_imu（LiDAR→IMU 外参，供 FAST-LIO2 使用）
  - translation: `[0.011, 0.02329, -0.04412]`（米）
    - 含义：IMU 在 LiDAR 坐标系下的位置（MID360 官方规格），等价于 LiDAR→IMU 的平移。
    - 用途：点云去畸变与 IESKF 状态传播。
  - rotation: `单位阵`（R_il=I）
    - 含义：IMU 轴与点云坐标轴方向一致。
    - 提示：若实际安装存在偏差，可先 `esti_il: true` 在线微调，收敛后写回此处并固化。
- lidar_to_baselink（LiDAR→base_link 外参，供驱动/URDF）
  - translation/rotation_rpy：用于生成 Livox 驱动与可视化的外参；若 base_link 放在 IMU 原点，IMU→LiDAR 的静态 TF 需与上文互为反向一致。

## 4. frames（坐标系命名）
- world_frame: `odom`（LIO 输出世界系）
- base_frame: `base_link`（机体/IMU 坐标系）
- lidar_frame: `livox_frame`（雷达坐标系）
说明：启动文件发布 `base_link→livox_frame` 静态 TF；务必避免重复来源（URDF 与 static_transform_publisher 二选一）。

## 5. topics（ROS 话题）
- 输入：
  - imu: `/livox/imu`
  - lidar: `/livox/lidar`
- 输出（协同总线）：
  - odometry: `/slam/lio_odom`
  - path: `/slam/lio_path`
  - body_cloud: `/slam/body_cloud`
  - world_cloud: `/slam/world_cloud`
  - performance: `/slam/performance_metrics`
  - diagnostics: `/slam/diagnostics`
说明：`cooperative_slam_system.launch.py` 已将 FAST-LIO2 的 `/fastlio2/*` remap 到 `/slam/*`，RViz 配置亦已切换到 `/slam/*`。

## 6. livox_driver（Livox 驱动）
- xfer_format: `1`（自定义消息：CustomMsg）
- multi_topic: `0`（单话题）
- data_src: `0`（设备数据）
- publish_freq: `10.0 Hz`
- output_data_type: `0`
- frame_id: `livox_frame`
- cmdline_input_bd_code: 设备识别码
- lidar_config：
  - pcl_data_type: `1`，pattern_mode: `0`
说明：驱动发布 `/livox/lidar`（自定义点云）与 `/livox/imu`。

## 7. fastlio2（FAST-LIO2 核心配置）
### 7.1 基本
- print_time_cost: `true`（打印时耗，便于调试）

### 7.2 雷达预处理参数
- lidar_filter_num: `3`
  - 每隔 N 个点采样一次，适度降采样降低负载。
- lidar_min_range / lidar_max_range: `0.5 / 25.0`（米）
  - 点云距离阈值，剔除太近/太远的点。
- scan_resolution: `0.05`（米）
  - 扫描体素滤波的叶大小；越大约束越稀疏、但更稳。
- map_resolution: `0.10`（米）
  - 地图体素分辨率；越大内存与计算越低、细节越少。

### 7.3 地图管理
- cube_len: `200`（米）— 局部地图立方体边长
- det_range: `50`（米）— 触发移动/裁剪的检测范围
- move_thresh: `1.0`— 移动阈值因子

### 7.4 IMU 噪声参数（单位：m/s²、rad/s）
- accelerometer(na): `0.005`
- gyroscope(ng): `0.005`
- acc_bias(nba): `0.0001`
- gyro_bias(nbg): `0.00005`
说明：驱动输出加速度为 g；LIO 节点已统一转换为 m/s²（乘 9.81）。

### 7.5 算法参数
- imu_init_num: `200`（IMU 初始化样本数，约 1s）
- near_search_num: `15`（KNN 近邻数，平面拟合用；数值越大约束更严格但负载更高）
- ieskf_max_iter: `8`
- gravity_align: `true`（重力对齐）
- esti_il: `true`（在线估计 LiDAR→IMU 外参）
建议：在空旷环境先跑一段数据估计外参并写回，然后任务时可切回 `false` 固化外参以增稳。

### 7.6 点云匹配
- point_cloud.lidar_cov_inv: `2000.0`（匹配残差权重）

### 7.7 缓冲区管理
- buffer_management.max_imu_buffer_size: `1000`
- buffer_management.max_lidar_buffer_size: `50`
- buffer_management.enable_buffer_monitoring: `true`
说明：启用后会在溢出或时间逆序时打印警告。

## 8. visualization（可视化）
- rviz.auto_start: `true`
- rviz.config_file: `enhanced_fastlio2.rviz`（已切换到 /slam/* 话题）
- display 开关：pointcloud/trajectory/tf_frames/imu_pose/diagnostics

## 9. rosbag（数据记录）
- auto_record: `false`（可按需启用）
- topics：`/livox/lidar`、`/livox/imu`、`/slam/*`、`/tf`、`/tf_static`
- output_dir: `data/rosbags`
- compression_format: `zstd`

## 10. monitoring（监控与日志）
- performance.enable_metrics: `true`，publish_frequency: `1.0`
- diagnostics.enable_diagnostics: `true`，check_frequency: `2.0`
- logging.level: `INFO`，verbose: `false`
补充工具：`tools/imu_odom_monitor.py` 可实时统计 IMU/Odom 频率、时延、抖动，支持 `--use-sim-time` 与 CSV 输出。

## 11. pgo（位姿图优化）
- cloud_topic/odom_topic：`/slam/body_cloud`、`/slam/lio_odom`
- map_frame/local_frame：`map`/`lidar`
- 关键帧选择：
  - key_pose_delta_deg: `10`（度）
  - key_pose_delta_trans: `0.3`（米）
- 回环检测：
  - loop_search_radius: `2.5`（米）
  - loop_time_tresh: `30.0`（秒）
  - loop_score_tresh: `0.15`
- 子地图：
  - loop_submap_half_range: `5`（米）
  - submap_resolution: `0.1`（米）
  - min_loop_detect_duration: `5.0`（秒）

## 12. hba（层次化捆绑调整）
- scan_resolution: `0.1`（米）
- window_size/stride: `20`/`10`
- voxel_size: `0.5`，min_point_num: `10`，max_layer: `3`
- plane_thresh: `0.01`，ba_max_iter: `10`，hba_iter: `5`，down_sample: `0.1`

## 13. localizer（定位器）
- cloud_topic/odom_topic：`/slam/body_cloud`、`/slam/lio_odom`
- map_frame/local_frame：`map`/`lidar`
- update_hz: `1.0`
- 粗定位（rough_localization）：
  - scan_resolution/map_resolution: `0.25/0.25`（米）
  - max_iteration: `5`，score_thresh: `0.2`
- 精定位（refine_localization）：
  - scan_resolution/map_resolution: `0.1/0.1`（米）
  - max_iteration: `10`，score_thresh: `0.1`
- 动态对象过滤（dynamic_filter）参数详见 `ws_livox/src/localizer/config/localizer.yaml`。

---

## 重要约束与最佳实践
1) 唯一 TF 源：`base_link→livox_frame` 只能有一个来源；当前保留 `static_transform_publisher`，已移除 URDF 重复关节。
2) 外参与 TF 一致：LiDAR→IMU（R_il/t_il）与 IMU→LiDAR 静态 TF 互为反向表达。
3) 仿真时间：回放 bag 时请使用 `--clock` 并设置 `use_sim_time=true`（RViz 与节点）。
4) 负载与稳健性：拐弯/狭窄空间建议使用本文档中较稳的体素与近邻参数；如仍负载高或“NO Effective Points!”频发，可适度增大 `scan_resolution/map_resolution` 或减小 `near_search_num`。
5) 生成器：修改主配置后，运行 `python3 tools/config_generator.py --master-config config/master_config.yaml` 以更新子配置；不要手动改写生成文件。

---

## 变更记录（与本次调优相关）
- 同步到 `/slam/*` 话题总线；统一 RViz、launch remap 与记录话题。
- 修正 LiDAR→IMU 外参并开启 `esti_il=true`，TF 与外参保持一致。
- 稳定参数（狭窄拐角）：`scan_resolution=0.05`，`map_resolution=0.10`，`near_search_num=15`，`lidar_filter_num=3`。
- 监控脚本：新增 `tools/imu_odom_monitor.py`（频率/时延/抖动）。

