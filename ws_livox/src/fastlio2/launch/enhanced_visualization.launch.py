#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('fastlio2'),
            'config',
            'lio.yaml'
        ]),
        description='Path to the FAST-LIO2 configuration file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('fastlio2'),
            'rviz',
            'enhanced_fastlio2.rviz'
        ]),
        description='Path to the enhanced RViz configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Livox雷达驱动节点
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        parameters=[{
            'xfer_format': 1,    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            'multi_topic': 0,    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
            'data_src': 0,       # 0-lidar, others-Invalid data src
            'publish_freq': 10.0, # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'config',
                'MID360_config.json'
            ]),
            'cmdline_input_bd_code': 'livox0000000001',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # FAST-LIO2节点
    fastlio2_node = Node(
        package='fastlio2',
        executable='lio_node',
        name='fastlio2_node',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # RViz2节点，使用增强的配置文件
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # rqt_robot_monitor用于显示诊断信息（需要单独安装和启动）
    # 可以通过以下命令手动启动诊断监控：
    # ros2 run rqt_robot_monitor rqt_robot_monitor
    
    # 可选：添加plotjuggler用于性能数据可视化
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=['--layout', PathJoinSubstitution([
            FindPackageShare('fastlio2'),
            'config',
            'plotjuggler_layout.xml'
        ])],
        condition=lambda context: os.path.exists(
            os.path.join(get_package_share_directory('fastlio2'), 'config', 'plotjuggler_layout.xml')
        ),
        output='screen'
    )
    
    return LaunchDescription([
        config_path_arg,
        rviz_config_arg,
        use_sim_time_arg,
        livox_driver_node,
        fastlio2_node,
        rviz2_node,
        # plotjuggler_node  # 取消注释以启用PlotJuggler
    ])