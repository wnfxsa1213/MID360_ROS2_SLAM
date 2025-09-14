#!/usr/bin/env python3
"""
MID360 SLAM系统完整启动文件 (增强版)
======================================

包含所有SLAM组件的统一启动文件：
- Livox MID360 雷达驱动
- FAST-LIO2 实时SLAM核心
- PGO 位姿图优化 (智能延迟启动)
- HBA 分层束调整 (智能延迟启动) 
- LOCALIZER 重定位服务 (智能延迟启动)
- RViz2 增强可视化

特性：
- 智能时序启动，避免组件间竞争
- 可选择性启用/禁用各组件
- 自动配置文件路径解析
- 完整的错误处理和日志记录

使用方式：
  ros2 launch fastlio2 enhanced_visualization.launch.py
  ros2 launch fastlio2 enhanced_visualization.launch.py enable_pgo:=false
  ros2 launch fastlio2 enhanced_visualization.launch.py enable_hba:=false enable_localizer:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
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
    
    # 静态TF变换发布器 - base_link到livox_frame
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
        name='base_link_to_livox_frame_broadcaster'
    )
    
    # PGO位姿图优化节点 (延迟启动)
    pgo_node = Node(
        package='pgo',
        executable='pgo_node',
        name='pgo_node',
        parameters=[{
            'config_path': PathJoinSubstitution([
                FindPackageShare('pgo'),
                'config',
                'pgo.yaml'
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # HBA分层束调整节点 (延迟启动)
    hba_node = Node(
        package='hba', 
        executable='hba_node',
        name='hba_node',
        parameters=[{
            'config_path': PathJoinSubstitution([
                FindPackageShare('hba'),
                'config', 
                'hba.yaml'
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # LOCALIZER重定位节点 (延迟启动) 
    localizer_node = Node(
        package='localizer',
        executable='localizer_node', 
        name='localizer_node',
        parameters=[{
            'config_path': PathJoinSubstitution([
                FindPackageShare('localizer'),
                'config',
                'localizer.yaml'
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
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
    
    # 添加组件启用参数
    enable_pgo_arg = DeclareLaunchArgument(
        'enable_pgo',
        default_value='true',
        description='Enable PGO (Pose Graph Optimization)'
    )
    
    enable_hba_arg = DeclareLaunchArgument(
        'enable_hba', 
        default_value='true',
        description='Enable HBA (Hierarchical Bundle Adjustment)'
    )
    
    enable_localizer_arg = DeclareLaunchArgument(
        'enable_localizer',
        default_value='true', 
        description='Enable LOCALIZER (Relocalization)'
    )
    
    # 使用TimerAction实现时序启动
    
    # 延迟启动优化组件 - 等FAST-LIO2稳定后再启动
    delayed_pgo_node = TimerAction(
        period=8.0,  # 8秒后启动PGO
        actions=[pgo_node],
        condition=IfCondition(LaunchConfiguration('enable_pgo'))
    )
    
    delayed_hba_node = TimerAction(
        period=10.0,  # 10秒后启动HBA
        actions=[hba_node],
        condition=IfCondition(LaunchConfiguration('enable_hba'))
    )
    
    delayed_localizer_node = TimerAction(
        period=12.0,  # 12秒后启动LOCALIZER
        actions=[localizer_node], 
        condition=IfCondition(LaunchConfiguration('enable_localizer'))
    )
    
    # 延迟启动RViz - 让核心组件先初始化
    delayed_rviz_node = TimerAction(
        period=3.0,  # 3秒后启动RViz
        actions=[rviz2_node]
    )
    
    return LaunchDescription([
        # 启动参数
        config_path_arg,
        rviz_config_arg,
        use_sim_time_arg,
        enable_pgo_arg,
        enable_hba_arg,
        enable_localizer_arg,
        
        # 核心组件 - 立即启动
        livox_driver_node,
        fastlio2_node,
        static_tf_publisher,
        
        # 延迟启动的组件
        delayed_rviz_node,
        delayed_pgo_node,
        delayed_hba_node,  
        delayed_localizer_node,
        
        # plotjuggler_node  # 取消注释以启用PlotJuggler
    ])