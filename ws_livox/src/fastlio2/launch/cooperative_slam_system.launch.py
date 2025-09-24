#!/usr/bin/env python3

import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """
    协同SLAM系统启动文件 - 真正的协同工作
    包含：FastLIO2 + PGO + HBA + Localizer + 优化协调器
    """

    # 启动参数（bag回放/模拟时间）
    use_bag = LaunchConfiguration('use_bag')
    use_sim_time = LaunchConfiguration('use_sim_time')
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
        launch.actions.DeclareLaunchArgument(
            'use_bag', default_value='false',
            description='为true时禁用Livox驱动与静态TF，适配bag回放'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='为true时为各节点启用模拟时间'),
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
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': livox_config_path,
                'cmdline_input_bd_code': 'livox0000000001',
                'use_sim_time': use_sim_time,
            }],
            condition=UnlessCondition(use_bag)
        ),

        # === 协同增强FAST-LIO2核心 ===
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
            ]
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
            parameters=[{
                'drift_threshold': 0.15,     # 测试优化：更低的漂移阈值
                'time_threshold': 15.0,      # 测试优化：更频繁的优化
                'emergency_threshold': 0.5,  # 测试优化：更早的紧急处理
                'auto_optimization': True,   # 启用自动优化
                'use_sim_time': use_sim_time,
            }],
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
