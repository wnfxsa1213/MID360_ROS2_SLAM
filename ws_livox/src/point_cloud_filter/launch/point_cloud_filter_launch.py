#!/usr/bin/env python3
"""
点云过滤桥接节点启动文件
Point Cloud Filter Bridge Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    package_dir = FindPackageShare(package='point_cloud_filter')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        package_dir,
        'config',
        'point_cloud_filter.yaml'
    ])
    
    # 声明启动参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='点云过滤器配置文件路径'
    )
    
    declare_debug_enabled = DeclareLaunchArgument(
        'debug_enabled',
        default_value='false',
        description='是否启用调试模式'
    )
    
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='输入话题名称'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic', 
        default_value='/livox/lidar_filtered',
        description='输出话题名称'
    )
    
    declare_max_processing_hz = DeclareLaunchArgument(
        'max_processing_hz',
        default_value='10.0',
        description='最大处理频率(Hz)'
    )

    # 点云过滤桥接节点
    point_cloud_filter_node = Node(
        package='point_cloud_filter',
        executable='point_cloud_filter_bridge_node',
        name='point_cloud_filter_bridge',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'debug_enabled': LaunchConfiguration('debug_enabled'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'max_processing_hz': LaunchConfiguration('max_processing_hz'),
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动点云过滤桥接节点...\n',
            '  配置文件: ', LaunchConfiguration('config_file'), '\n',
            '  输入话题: ', LaunchConfiguration('input_topic'), '\n', 
            '  输出话题: ', LaunchConfiguration('output_topic'), '\n',
            '  调试模式: ', LaunchConfiguration('debug_enabled'), '\n',
            '  最大处理频率: ', LaunchConfiguration('max_processing_hz'), ' Hz\n',
        ]
    )

    return LaunchDescription([
        # 声明参数
        declare_config_file,
        declare_debug_enabled,
        declare_input_topic,
        declare_output_topic,
        declare_max_processing_hz,
        
        # 启动信息
        launch_info,
        
        # 节点
        point_cloud_filter_node,
    ])


if __name__ == '__main__':
    generate_launch_description()