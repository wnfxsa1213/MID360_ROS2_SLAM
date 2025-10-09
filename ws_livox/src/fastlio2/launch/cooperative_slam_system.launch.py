#!/usr/bin/env python3

import launch
import launch_ros.actions
from pathlib import Path
import yaml
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

PROJECT_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_CONFIG_FILE = PROJECT_ROOT / "config" / "launch_config.yaml"
if LAUNCH_CONFIG_FILE.exists():
    with open(LAUNCH_CONFIG_FILE, "r", encoding="utf-8") as f:
        LAUNCH_CONFIG = yaml.safe_load(f)
else:
    LAUNCH_CONFIG = {}

def generate_launch_description():
    """
    协同SLAM系统启动文件 - 真正的协同工作
    包含：FastLIO2 + PGO + HBA + Localizer + 优化协调器
    """

    # 启动参数（bag回放/模拟时间/是否启用过滤桥）
    use_bag = LaunchConfiguration('use_bag')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_filter_bridge = LaunchConfiguration('enable_filter_bridge')
    auto_play_bag = LaunchConfiguration('auto_play_bag')
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    bag_loop = LaunchConfiguration('bag_loop')

    # 配置文件路径
    livox_config_path = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    ])

    # LIO 配置（默认）
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])

    # LIO 配置（禁用过滤桥：直接吃原始 /livox/lidar）
    lio_config_no_filter_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio_no_filter.yaml"
    ])

    pgo_config_path = PathJoinSubstitution([
        FindPackageShare("pgo"), "config", "pgo.yaml"
    ])

    hba_config_path = PathJoinSubstitution([
        FindPackageShare("hba"), "config", "hba.yaml"
    ])

    localizer_config_path = PathJoinSubstitution([
        FindPackageShare("localizer"), "config", "localizer.yaml"
    ])

    point_cloud_filter_config_path = PathJoinSubstitution([
        FindPackageShare("point_cloud_filter"), "config", "point_cloud_filter.yaml"
    ])

    coordinator_config_path = PathJoinSubstitution([
        FindPackageShare("cooperation"), "config", "coordinator.yaml"
    ])

    driver_cfg = LAUNCH_CONFIG.get("lidar", {})

    # RViz配置
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "rviz", "enhanced_fastlio2.rviz"
    ])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_bag', default_value='false',
            description='为true时禁用Livox驱动与静态TF，适配bag回放'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='为true时为各节点启用模拟时间'),
        launch.actions.DeclareLaunchArgument(
            'enable_filter_bridge', default_value='true',
            description='是否启用点云过滤桥（true: 使用 /livox/lidar_filtered；false: 直接使用 /livox/lidar）'),
        launch.actions.DeclareLaunchArgument(
            'auto_play_bag', default_value='false',
            description='为true且use_bag=true时，自动回放bag'),
        launch.actions.DeclareLaunchArgument(
            'bag_path', default_value='',
            description='ros2 bag 目录（auto_play_bag启用时必填）'),
        launch.actions.DeclareLaunchArgument(
            'bag_rate', default_value='1.0',
            description='ros2 bag 回放速度倍率'),
        launch.actions.DeclareLaunchArgument(
            'bag_loop', default_value='false',
            description='是否循环回放bag'),

        # === 核心传感器驱动 ===
        launch_ros.actions.Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                'xfer_format': driver_cfg.get('xfer_format', 1),
                'multi_topic': driver_cfg.get('multi_topic', 0),
                'data_src': driver_cfg.get('data_src', 0),
                'publish_freq': driver_cfg.get('publish_freq', 10.0),
                'output_data_type': driver_cfg.get('output_data_type', 0),
                'frame_id': driver_cfg.get('frame_id', 'livox_frame'),
                'user_config_path': livox_config_path,
                'cmdline_input_bd_code': driver_cfg.get('cmdline_input_bd_code', 'livox0000000001'),
                'use_sim_time': use_sim_time,
            }],
            condition=UnlessCondition(use_bag)
        ),

        # === 点云过滤桥接节点 ===
        # 说明：ROS2 参数文件应以 YAML 文件路径传入（或使用 node-name/ros__parameters 结构）。
        # 之前这里错误地以 "config_path" 字段传入，节点并不会读取此自定义键，导致 YAML 未生效。
        # 修复：将 YAML 路径作为参数文件传入，并保留可覆盖的显式参数。
        launch_ros.actions.Node(
            package="point_cloud_filter",
            executable="point_cloud_filter_bridge_node",
            name="point_cloud_filter_bridge",
            output="screen",
            parameters=[
                point_cloud_filter_config_path,
                {'use_sim_time': use_sim_time}
            ],
            # 仅当未使用bag且显式启用过滤桥时启动
            condition=launch.conditions.IfCondition(PythonExpression([
                '("', use_bag, '" == "false") and ("', enable_filter_bridge, '" == "true")'
            ]))
        ),

        # === 协同增强FAST-LIO2核心 ===
        # LIO（启用过滤桥时使用标准配置，订阅 /livox/lidar_filtered）
        launch_ros.actions.Node(
            package="fastlio2",
            executable="lio_node",
            name="fastlio2_node",
            output="screen",
            parameters=[{"config_path": lio_config_path, 'use_sim_time': use_sim_time}],
            remappings=[
                ("/fastlio2/body_cloud", "/slam/body_cloud"),
                ("/fastlio2/lio_odom", "/slam/lio_odom"),
                ("/fastlio2/world_cloud", "/slam/world_cloud"),
                ("/fastlio2/lio_path", "/slam/lio_path"),
                ("/fastlio2/performance_metrics", "/slam/performance_metrics"),
                ("/fastlio2/diagnostics", "/slam/diagnostics")
            ],
            condition=launch.conditions.IfCondition(enable_filter_bridge)
        ),

        # LIO（禁用过滤桥时使用 no_filter 配置，直接订阅 /livox/lidar）
        launch_ros.actions.Node(
            package="fastlio2",
            executable="lio_node",
            name="fastlio2_node",
            output="screen",
            parameters=[{"config_path": lio_config_no_filter_path, 'use_sim_time': use_sim_time}],
            remappings=[
                ("/fastlio2/body_cloud", "/slam/body_cloud"),
                ("/fastlio2/lio_odom", "/slam/lio_odom"),
                ("/fastlio2/world_cloud", "/slam/world_cloud"),
                ("/fastlio2/lio_path", "/slam/lio_path"),
                ("/fastlio2/performance_metrics", "/slam/performance_metrics"),
                ("/fastlio2/diagnostics", "/slam/diagnostics")
            ],
            condition=launch.conditions.UnlessCondition(enable_filter_bridge)
        ),

        # === PGO位姿图优化 ===
        launch_ros.actions.Node(
            package="pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[{"config_path": pgo_config_path, 'use_sim_time': use_sim_time}],
            remappings=[
                ("/pgo/loop_markers", "/slam/loop_markers")
            ]
        ),

        # === HBA分层束调整 ===
        launch_ros.actions.Node(
            package="hba",
            executable="hba_node",
            name="hba_node",
            output="screen",
            parameters=[{"config_path": hba_config_path, 'use_sim_time': use_sim_time}]
        ),

        # === Localizer重定位 ===
        launch_ros.actions.Node(
            package="localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[{"config_path": localizer_config_path, 'use_sim_time': use_sim_time}]
        ),

        # === 🚀 优化协调器 - 协同工作核心 ===
        launch_ros.actions.Node(
            package="cooperation",
            executable="optimization_coordinator",
            name="optimization_coordinator",
            output="screen",
            parameters=[
                coordinator_config_path,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ("/coordinator/metrics", "/slam/coordination_metrics"),
                ("/fastlio2/lio_odom", "/slam/lio_odom"),
                ("/fastlio2/performance_metrics", "/slam/performance_metrics")
            ]
        ),

        # === TF变换系统 ===
        # Robot State Publisher - 发布机器人描述和静态变换
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': """<?xml version="1.0"?>
                <robot name="mid360_robot">
                  <link name="base_footprint"/>
                  <link name="base_link"/>
                  <link name="imu_link"/>
                  <link name="lidar_mount"/>
                  <link name="livox_frame"/>
                  <link name="body"/>
                  <link name="lidar"/>

                  <joint name="base_footprint_to_base_link" type="fixed">
                    <parent link="base_footprint"/>
                    <child link="base_link"/>
                    <origin xyz="0 0 0.1" rpy="0 0 0"/>
                  </joint>

                  <joint name="base_link_to_imu_link" type="fixed">
                    <parent link="base_link"/>
                    <child link="imu_link"/>
                    <origin xyz="0 0 0.05" rpy="0 0 0"/>
                  </joint>

                  <joint name="base_link_to_lidar_mount" type="fixed">
                    <parent link="base_link"/>
                    <child link="lidar_mount"/>
                    <origin xyz="0 0 0.1" rpy="0 0 0"/>
                  </joint>

                  <joint name="lidar_mount_to_livox_frame" type="fixed">
                    <parent link="lidar_mount"/>
                    <child link="livox_frame"/>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                  </joint>

                  <joint name="lidar_mount_to_lidar" type="fixed">
                    <parent link="lidar_mount"/>
                    <child link="lidar"/>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                  </joint>

                  <joint name="lidar_to_body" type="fixed">
                    <parent link="lidar"/>
                    <child link="body"/>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                  </joint>
                </robot>"""
            }]
        ),

        # Map to Odom变换发布器
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_broadcaster",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=UnlessCondition(use_bag)
        ),

        # Base Link (IMU) to Livox Frame (LiDAR) 变换发布器
        # 根据MID360规格：IMU在LiDAR坐标系下位置为(0.011, 0.02329, -0.04412)m
        # 因此IMU->LiDAR的平移为(-0.011, -0.02329, 0.04412)
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_frame_broadcaster",
            arguments=["-0.011", "-0.02329", "0.04412", "0", "0", "0", "base_link", "livox_frame"],
            condition=UnlessCondition(use_bag)
        ),

        # === 增强可视化系统 ===
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # === Bag回放（可选）===
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', bag_path,
                '--rate', bag_rate,
            ] + (['--loop'] if str(bag_loop) == 'true' else []),
            shell=False,
            output='screen',
            condition=IfCondition(PythonExpression([
                '("', use_bag, '" == "true") and ("', auto_play_bag, '" == "true")'
            ]))
        ),

        # === 系统监控节点 (暂时禁用，待实现) ===
        # launch_ros.actions.Node(
        #     package="cooperation",
        #     executable="system_monitor",
        #     name="system_monitor",
        #     output="screen",
        #     parameters=[{
        #         'monitor_interval': 5.0,
        #         'log_performance': True
        #     }],
        #     condition=launch.conditions.LaunchConfigurationEquals('enable_monitor', 'true')
        # )
    ])

# 使用方法：
# 实机：
#   ros2 launch fastlio2 cooperative_slam_system.launch.py
# 回放（推荐）：
#   ros2 launch fastlio2 cooperative_slam_system.launch.py use_bag:=true use_sim_time:=true
#   另开终端播放：ros2 bag play <bag_dir>

# 关闭过滤桥（直接使用 /livox/lidar 原始点云）：
#   ros2 launch fastlio2 cooperative_slam_system.launch.py enable_filter_bridge:=false
