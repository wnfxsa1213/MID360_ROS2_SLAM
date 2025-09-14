#!/usr/bin/env python3
"""
MID360 SLAM系统统一启动文件
基于主配置文件动态启动所有组件，确保配置一致性
"""

import os
import sys
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

class UnifiedSLAMLauncher:
    def __init__(self):
        self.config_loaded = False
        self.master_config = {}
        self.workspace_root = Path(__file__).parent.parent.parent.parent.parent
        
    def load_master_config(self, config_path):
        """加载主配置文件"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                self.master_config = yaml.safe_load(f)
            self.config_loaded = True
            return True
        except Exception as e:
            print(f"❌ 加载主配置文件失败: {e}")
            return False
    
    def resolve_config_variables(self, value):
        """解析配置文件中的变量引用"""
        if isinstance(value, str) and "${" in value:
            # 简单的变量替换实现
            global_params = self.master_config.get('global_params', {})
            for key, param_value in global_params.items():
                var_ref = f"${{global_params.{key}}}"
                if var_ref in value:
                    value = value.replace(var_ref, str(param_value))
        return value
    
    def create_livox_driver_node(self):
        """创建Livox雷达驱动节点"""
        system_config = self.master_config.get('system', {})
        global_params = self.master_config.get('global_params', {})
        
        return Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': global_params.get('lidar_frame', 'livox_frame'),
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
    
    def create_fastlio2_node(self):
        """创建FastLIO2 SLAM节点"""
        fastlio2_config = self.master_config.get('fastlio2', {})
        
        if not fastlio2_config.get('enabled', True):
            return None
            
        return Node(
            package='fastlio2',
            executable='lio_node',
            name='fastlio2_node',
            parameters=[{
                'config_path': PathJoinSubstitution([
                    FindPackageShare('fastlio2'),
                    'config',
                    'lio.yaml'
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen',
            emulate_tty=True
        )
    
    def create_pgo_node(self):
        """创建PGO节点"""
        pgo_config = self.master_config.get('pgo', {})
        
        if not pgo_config.get('enabled', True):
            return None
            
        return Node(
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
    
    def create_hba_node(self):
        """创建HBA节点"""
        hba_config = self.master_config.get('hba', {})
        
        if not hba_config.get('enabled', True):
            return None
            
        return Node(
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
    
    def create_localizer_node(self):
        """创建Localizer节点"""
        localizer_config = self.master_config.get('localizer', {})
        
        if not localizer_config.get('enabled', True):
            return None
            
        return Node(
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
    
    def create_rviz_node(self):
        """创建RViz可视化节点"""
        viz_config = self.master_config.get('visualization', {}).get('rviz', {})
        
        if not viz_config.get('enabled', True):
            return None
            
        rviz_config_file = viz_config.get('config_file', 'enhanced_fastlio2.rviz')
        
        return Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('fastlio2'),
                'rviz',
                rviz_config_file
            ])],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz'))
        )
    
    def generate_launch_description(self, master_config_path):
        """生成启动描述"""
        if not self.load_master_config(master_config_path):
            raise RuntimeError(f"无法加载主配置文件: {master_config_path}")
        
        # 启动参数声明
        launch_args = [
            DeclareLaunchArgument(
                'master_config',
                default_value=str(master_config_path),
                description='主配置文件路径'
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='使用仿真时间'
            ),
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='true',
                description='启用RViz可视化'
            ),
            DeclareLaunchArgument(
                'enable_monitoring',
                default_value='true',
                description='启用性能监控'
            )
        ]
        
        # 节点创建
        nodes = []
        system_config = self.master_config.get('system', {})
        wait_time = float(system_config.get('wait_time_between_nodes', 3))
        
        # 1. Livox驱动 - 优先级最高，立即启动
        livox_node = self.create_livox_driver_node()
        if livox_node:
            nodes.append(livox_node)
            nodes.append(LogInfo(msg="🚀 Livox雷达驱动已启动"))
        
        # 2. FastLIO2 SLAM - 核心组件，等待雷达启动后启动
        fastlio2_node = self.create_fastlio2_node()
        if fastlio2_node:
            nodes.append(TimerAction(
                period=wait_time,
                actions=[
                    fastlio2_node,
                    LogInfo(msg="🧭 FastLIO2 SLAM已启动")
                ]
            ))
        
        # 3. PGO - 可选组件，等待SLAM稳定后启动
        pgo_node = self.create_pgo_node()
        if pgo_node:
            nodes.append(TimerAction(
                period=wait_time * 2,
                actions=[
                    pgo_node,
                    LogInfo(msg="🔄 PGO位姿图优化已启动")
                ]
            ))
        
        # 4. HBA - 可选组件
        hba_node = self.create_hba_node()
        if hba_node:
            nodes.append(TimerAction(
                period=wait_time * 2.5,
                actions=[
                    hba_node,
                    LogInfo(msg="📐 HBA分层束调整已启动")
                ]
            ))
        
        # 5. Localizer - 可选组件
        localizer_node = self.create_localizer_node()
        if localizer_node:
            nodes.append(TimerAction(
                period=wait_time * 3,
                actions=[
                    localizer_node,
                    LogInfo(msg="📍 Localizer定位器已启动")
                ]
            ))
        
        # 6. RViz可视化 - 最后启动
        rviz_node = self.create_rviz_node()
        if rviz_node:
            nodes.append(TimerAction(
                period=wait_time * 1.5,
                actions=[
                    rviz_node,
                    LogInfo(msg="👁️ RViz可视化已启动")
                ]
            ))
        
        # 添加系统状态信息
        nodes.append(TimerAction(
            period=1.0,
            actions=[LogInfo(msg="🎯 MID360 SLAM系统启动完成！基于统一配置管理")]
        ))
        
        return LaunchDescription(launch_args + nodes)

def generate_launch_description():
    """Launch文件入口点"""
    launcher = UnifiedSLAMLauncher()
    
    # 确定主配置文件路径 - 修正路径计算
    # 从安装路径向上找到真正的项目根目录
    current_path = Path(__file__).parent
    project_root = None
    
    # 向上搜索直到找到真正的项目根目录（包含ws_livox目录和正确的config目录）
    for i in range(10):  # 最多向上搜索10级
        test_path = current_path
        for _ in range(i):
            test_path = test_path.parent
        
        # 检查是否是真正的项目根目录：必须包含ws_livox目录和config目录
        ws_livox_dir = test_path / "ws_livox"
        config_dir = test_path / "config"
        slam_config_file = config_dir / "slam_master_config.yaml"
        
        if ws_livox_dir.exists() and config_dir.exists() and slam_config_file.exists():
            project_root = test_path
            break
    
    if project_root is None:
        # 如果找不到，尝试硬编码路径
        project_root = Path("/home/tianyu/codes/Mid360map")
    
    master_config_path = project_root / "config" / "slam_master_config.yaml"
    
    if not master_config_path.exists():
        # 备用配置路径
        master_config_path = project_root / "config" / "deprecated" / "full_slam_config.yaml"
        
    if not master_config_path.exists():
        raise RuntimeError(f"找不到主配置文件，尝试过的路径: {project_root / 'config' / 'slam_master_config.yaml'} 和 {project_root / 'config' / 'deprecated' / 'full_slam_config.yaml'}")
    
    return launcher.generate_launch_description(master_config_path)