#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def get_tf_params_from_config(master_config_path):
    """
    从主配置文件读取TF变换参数

    Args:
        master_config_path: 主配置文件路径

    Returns:
        list: TF变换参数 [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
    """
    try:
        if os.path.exists(master_config_path):
            with open(master_config_path, 'r', encoding='utf-8') as f:
                master_config = yaml.safe_load(f)

            # 读取外参配置
            calibration = master_config.get('calibration', {})
            lidar_to_baselink = calibration.get('lidar_to_baselink', {})

            # 提取平移和旋转参数
            translation = lidar_to_baselink.get('translation', [0.0, 0.0, 0.0])
            rotation_rpy = lidar_to_baselink.get('rotation_rpy', [0.0, 0.0, 0.0])

            # 构造TF参数（格式：x, y, z, roll, pitch, yaw, parent_frame, child_frame）
            tf_params = [
                str(translation[0]),  # x
                str(translation[1]),  # y
                str(translation[2]),  # z
                str(rotation_rpy[0]), # roll
                str(rotation_rpy[1]), # pitch
                str(rotation_rpy[2]), # yaw
                'base_link',          # parent_frame
                'livox_frame'         # child_frame
            ]

            print(f"从配置文件读取TF参数: translation={translation}, rotation_rpy={rotation_rpy}")
            return tf_params

    except Exception as e:
        print(f"警告: 读取外参配置失败: {e}")

    # 默认参数（向后兼容）
    print("使用默认TF参数（零值）")
    return ['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']

def generate_launch_description():
    # 读取启动配置文件
    config_file_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))),
        'config',
        'launch_config.yaml'
    )

    # 检查主配置文件
    master_config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))),
        'config',
        'master_config.yaml'
    )

    # 从配置文件获取TF变换参数
    tf_params = get_tf_params_from_config(master_config_path)

    # 配置一致性检查
    if os.path.exists(master_config_path):
        print("检查配置文件一致性...")

        # 检查各组件配置文件是否存在且是由配置生成器生成的
        config_files_to_check = [
            config_file_path,
            os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'lio.yaml'),
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                'livox_ros_driver2', 'config', 'MID360_config.json'
            )
        ]

        need_regenerate = False
        for config_file in config_files_to_check:
            if not os.path.exists(config_file):
                need_regenerate = True
                print(f"配置文件缺失: {config_file}")
                break

            # 检查是否由配置生成器生成
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if '由配置生成器自动生成' not in content:
                        need_regenerate = True
                        print(f"配置文件需要重新生成: {config_file}")
                        break
            except:
                pass

        if need_regenerate:
            print("="*60)
            print("⚠️  检测到配置文件不一致或缺失")
            print("   这可能导致轨迹飘移等问题")
            print("")
            print("请运行以下命令生成统一配置文件:")
            print("   ./tools/slam_tools.sh config generate")
            print("="*60)
            import time
            time.sleep(3)  # 给用户时间阅读提示

    launch_config = {}
    if os.path.exists(config_file_path):
        with open(config_file_path, 'r', encoding='utf-8') as f:
            launch_config = yaml.safe_load(f)

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

    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value=str(launch_config.get('rosbag', {}).get('auto_record', False)).lower(),
        description='Enable automatic rosbag recording'
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
    
    # 静态TF变换：base_link -> livox_frame (从配置文件读取外参)
    static_tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_livox_frame_broadcaster',
        arguments=tf_params
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

    # rosbag2录制功能
    rosbag_config = launch_config.get('rosbag', {})
    record_topics = rosbag_config.get('topics', ['/livox/lidar', '/livox/imu'])
    output_dir = rosbag_config.get('output_dir', 'data/rosbags')

    # 确保输出目录存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    # 生成带时间戳的bag文件名
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'slam_data_{timestamp}'

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', os.path.join(output_dir, bag_name),
            '--compression-mode', 'file',
            '--compression-format', 'zstd'
        ] + record_topics,
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_recording').perform(context).lower() == 'true'
    )
    
    return LaunchDescription([
        config_path_arg,
        rviz_config_arg,
        use_sim_time_arg,
        enable_recording_arg,
        livox_driver_node,
        static_tf_base_to_livox,
        fastlio2_node,
        rviz2_node,
        rosbag_record,
        # plotjuggler_node  # 取消注释以启用PlotJuggler
    ])