#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('slam_gui')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, '..', '..', '..', '..', 'config', 'slam_master_config.yaml'),
        description='Full path to config file to load'
    )
    
    # SLAM GUI节点
    slam_gui_node = Node(
        package='slam_gui',
        executable='slam_gui',
        name='slam_gui_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': LaunchConfiguration('config_file')
        }],
        remappings=[
            # 重映射话题名称以匹配系统标准
            ('/slam_gui/input_cloud', '/fastlio2/body_cloud'),
            ('/slam_gui/input_odom', '/fastlio2/lio_odom'),
            ('/slam_gui/input_path', '/fastlio2/lio_path'),
            ('/slam_gui/input_diagnostics', '/fastlio2/diagnostics')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        slam_gui_node
    ])