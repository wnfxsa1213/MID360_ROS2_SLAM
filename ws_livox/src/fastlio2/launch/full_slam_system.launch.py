import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    完整SLAM系统启动文件 - 现已支持协同工作模式
    """
    # Livox驱动配置文件路径
    livox_config_path = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    ])
    # 配置文件路径
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )
    hba_config_path = PathJoinSubstitution(
        [FindPackageShare("hba"), "config", "hba.yaml"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    
    # RViz配置 - 使用增强可视化配置
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "enhanced_fastlio2.rviz"]
    )

    return launch.LaunchDescription(
        [
            # Livox雷达驱动节点
            launch_ros.actions.Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[{
                    'xfer_format': 1,    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
                    'multi_topic': 0,    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
                    'data_src': 0,       # 0-lidar, others-Invalid data src
                    'publish_freq': 10.0, # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
                    'output_data_type': 0,
                    'frame_id': 'livox_frame',
                    'user_config_path': livox_config_path,
                    'cmdline_input_bd_code': 'livox0000000001'
                }]
            ),
            
            # FAST-LIO2 SLAM核心节点
            launch_ros.actions.Node(
                package="fastlio2",
                executable="lio_node",
                name="fastlio2_node",
                output="screen",
                parameters=[{"config_path": lio_config_path}]
            ),
            
            # PGO位姿图优化节点
            launch_ros.actions.Node(
                package="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[{"config_path": pgo_config_path}]
            ),
            
            # HBA束优化节点（后台运行，无RViz）
            launch_ros.actions.Node(
                package="hba",
                executable="hba_node",
                name="hba_node",
                output="screen",
                parameters=[{"config_path": hba_config_path}]
            ),
            
            # Localizer重定位节点（后台运行，无RViz）
            launch_ros.actions.Node(
                package="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[{"config_path": localizer_config_path}]
            ),
            
            # 🚀 优化协调器 - 启用协同工作
            launch_ros.actions.Node(
                package="cooperation",
                executable="optimization_coordinator",
                name="optimization_coordinator",
                output="screen",
                parameters=[{
                    'drift_threshold': 0.3,
                    'time_threshold': 30.0,
                    'emergency_threshold': 1.0,
                    'auto_optimization': True
                }]
            ),

            # RViz增强可视化显示
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
            )
        ]
    )