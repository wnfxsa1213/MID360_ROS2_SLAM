#!/usr/bin/env python3

import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    协同SLAM系统启动文件 - 真正的协同工作
    包含：FastLIO2 + PGO + HBA + Localizer + 优化协调器
    """

    # 配置文件路径
    livox_config_path = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    ])

    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
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

    # RViz配置
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "rviz", "enhanced_fastlio2.rviz"
    ])

    return launch.LaunchDescription([

        # === 核心传感器驱动 ===
        launch_ros.actions.Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': livox_config_path,
                'cmdline_input_bd_code': 'livox0000000001'
            }]
        ),

        # === 协同增强FAST-LIO2核心 ===
        launch_ros.actions.Node(
            package="fastlio2",
            executable="lio_node",
            name="fastlio2_node",
            output="screen",
            parameters=[{"config_path": lio_config_path}],
            remappings=[
                ("/fastlio2/body_cloud", "/slam/body_cloud"),
                ("/fastlio2/lio_odom", "/slam/lio_odom"),
                ("/fastlio2/world_cloud", "/slam/world_cloud"),
                ("/fastlio2/lio_path", "/slam/lio_path"),
                ("/fastlio2/performance_metrics", "/slam/performance_metrics"),
                ("/fastlio2/diagnostics", "/slam/diagnostics")
            ]
        ),

        # === PGO位姿图优化 ===
        launch_ros.actions.Node(
            package="pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[{"config_path": pgo_config_path}],
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
            parameters=[{"config_path": hba_config_path}]
        ),

        # === Localizer重定位 ===
        launch_ros.actions.Node(
            package="localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[{"config_path": localizer_config_path}]
        ),

        # === 🚀 优化协调器 - 协同工作核心 ===
        launch_ros.actions.Node(
            package="cooperation",
            executable="optimization_coordinator",
            name="optimization_coordinator",
            output="screen",
            parameters=[{
                'drift_threshold': 0.3,      # 室内优化：更低的漂移阈值
                'time_threshold': 30.0,      # 室内优化：更频繁的优化
                'emergency_threshold': 1.0,  # 室内优化：更早的紧急处理
                'auto_optimization': True    # 启用自动优化
            }],
            remappings=[
                ("/coordinator/metrics", "/slam/coordination_metrics")
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
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        # Base Link (IMU) to Livox Frame (LiDAR) 变换发布器
        # 根据MID360规格：IMU在LiDAR坐标系下位置为(0.011, 0.02329, -0.04412)m
        # 因此IMU->LiDAR的平移为(-0.011, -0.02329, 0.04412)
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_frame_broadcaster",
            arguments=["-0.011", "-0.02329", "0.04412", "0", "0", "0", "base_link", "livox_frame"]
        ),

        # === 增强可视化系统 ===
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=[{
                'use_sim_time': False
            }]
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
# ros2 launch fastlio2 cooperative_slam_system.launch.py
# 或者启用监控：
# ros2 launch fastlio2 cooperative_slam_system.launch.py enable_monitor:=true
