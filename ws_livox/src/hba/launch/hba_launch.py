import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([FindPackageShare("hba"), "config", "hba.yaml"])

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="hba",
                namespace="hba",
                executable="hba_node",
                name="hba",
                output="screen",
                parameters=[
                    {"config_path": config_path.perform(launch.LaunchContext())}
                ],
            ),
            # RViz节点已移除 - HBA功能保留，通过PGO窗口查看优化结果
        ]
    )
