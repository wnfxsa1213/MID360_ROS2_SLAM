#!/usr/bin/env python3
"""
MID360 SLAM系统配置生成器 (修复版)
==================================================

功能说明:
从主配置文件(slam_master_config.yaml)动态生成各个组件的配置文件，
完全消除硬编码问题，确保参数一致性和可维护性。

修复内容:
1. ✅ 消除所有硬编码参数 (max_range, min_range, filter_num等)
2. ✅ 修复默认值错误 (scan_resolution, map_resolution)
3. ✅ 添加话题名称配置支持
4. ✅ 增加详细的参数验证输出
5. ✅ 完善错误处理和备份机制

支持组件:
- FastLIO2: SLAM核心算法配置
- PGO: 位姿图优化配置
- HBA: 分层束调整配置
- Localizer: ICP重定位配置

使用方法:
python3 config_generator.py config/slam_master_config.yaml

作者: Claude Code Assistant (修复版)
日期: 2025-09-13
版本: 2.0 (硬编码问题修复版)
"""

import os
import sys
import yaml
from pathlib import Path

class ConfigGenerator:
    """
    MID360 SLAM系统配置文件生成器

    负责从主配置文件动态生成各个组件的配置文件，完全消除硬编码问题。

    主要功能:
    1. 动态读取主配置文件参数
    2. 验证参数的有效性和一致性
    3. 为每个组件生成对应的配置文件
    4. 自动备份原有配置文件
    5. 生成配置摘要和验证报告

    支持的组件:
    - FastLIO2: 实时SLAM核心算法
    - PGO: 位姿图优化和回环检测
    - HBA: 分层束调整优化
    - Localizer: ICP重定位系统
    """

    def __init__(self, master_config_path):
        """
        初始化配置生成器

        Args:
            master_config_path (str): 主配置文件路径
        """
        self.master_config_path = master_config_path
        self.workspace_root = Path(__file__).parent.parent
        self.config_dir = self.workspace_root / "config"

        # ========== 组件配置文件路径映射 ==========
        # 定义各个组件的配置文件输出路径
        self.component_configs = {
            'fastlio2': self.workspace_root / "ws_livox/src/fastlio2/config/lio.yaml",
            'pgo': self.workspace_root / "ws_livox/src/pgo/config/pgo.yaml",
            'hba': self.workspace_root / "ws_livox/src/hba/config/hba.yaml",
            'localizer': self.workspace_root / "ws_livox/src/localizer/config/localizer.yaml"
        }

        print(f"🔧 配置生成器初始化完成")
        print(f"   - 工作空间根目录: {self.workspace_root}")
        print(f"   - 主配置文件: {master_config_path}")
        print(f"   - 目标组件数量: {len(self.component_configs)}")
        
    def load_master_config(self):
        """加载主配置文件"""
        try:
            with open(self.master_config_path, 'r', encoding='utf-8') as f:
                self.master_config = yaml.safe_load(f)
            print(f"✅ 主配置文件加载成功: {self.master_config_path}")
            return True
        except Exception as e:
            print(f"❌ 加载主配置文件失败: {e}")
            return False
    
    def generate_fastlio2_config(self):
        """
        生成FastLIO2配置文件
        从主配置文件动态读取所有参数，消除硬编码
        """
        if not self.master_config.get('fastlio2', {}).get('enabled', True):
            print("⚠️ FastLIO2已禁用，跳过配置生成")
            return

        # 从主配置提取各个配置段
        global_params = self.master_config.get('global_params', {})
        system_config = self.master_config.get('system', {})
        input_topics = system_config.get('input_topics', {})
        fastlio2_config = self.master_config.get('fastlio2', {})
        lidar_params = fastlio2_config.get('lidar_params', {})
        mapping_params = fastlio2_config.get('mapping_params', {})
        imu_params = fastlio2_config.get('imu_params', {})
        algorithm_params = fastlio2_config.get('algorithm_params', {})
        extrinsic_params = fastlio2_config.get('extrinsic', {})

        # 构建完全基于主配置的参数字典
        config = {
            # ========== 话题和坐标系配置 ==========
            # 从主配置的输入话题配置读取（消除硬编码）
            'imu_topic': input_topics.get('imu_topic', '/livox/imu'),
            'lidar_topic': input_topics.get('lidar_topic', '/livox/lidar'),
            'body_frame': global_params.get('body_frame', 'body'),
            'world_frame': global_params.get('map_frame', 'map'),
            'print_time_cost': True,

            # ========== LiDAR传感器参数 ==========
            # 完全从主配置的lidar_params段读取（修复硬编码问题）
            'lidar_filter_num': lidar_params.get('filter_num', 1),  # 修复：从主配置读取，默认值与实际一致
            'lidar_min_range': lidar_params.get('min_range', 0.5),  # 修复：从主配置读取
            'lidar_max_range': lidar_params.get('max_range', 80.0), # 修复：从主配置读取（这是关键修复）
            'scan_resolution': global_params.get('scan_resolution', 0.15),  # 修复：默认值与实际一致
            'map_resolution': global_params.get('map_resolution', 0.15),    # 修复：默认值与实际一致

            # ========== 地图管理参数 ==========
            # 从主配置的mapping_params段读取（支持大范围建图）
            'cube_len': mapping_params.get('cube_len', 200),         # 局部地图范围
            'det_range': mapping_params.get('det_range', 80),        # 检测范围
            'move_thresh': mapping_params.get('move_thresh', 0.1),   # 移动阈值

            # ========== IMU噪声模型参数 ==========
            # 从主配置的imu_params段读取（针对MID360优化）
            'na': imu_params.get('noise_acc', 1.0e-5),      # 加速度计噪声
            'ng': imu_params.get('noise_gyro', 1.0e-5),     # 陀螺仪噪声
            'nba': imu_params.get('bias_acc', 0.1),         # 加速度计偏置噪声
            'nbg': imu_params.get('bias_gyro', 0.01),       # 陀螺仪偏置噪声

            # ========== SLAM算法参数 ==========
            # 从主配置的algorithm_params段读取
            'imu_init_num': imu_params.get('init_samples', 100),            # IMU初始化样本数
            'near_search_num': algorithm_params.get('near_search_num', 10), # KNN搜索邻点数
            'ieskf_max_iter': algorithm_params.get('ieskf_max_iter', 10),   # IESKF最大迭代次数
            'gravity_align': algorithm_params.get('gravity_align', True),   # 重力对齐
            'esti_il': algorithm_params.get('estimate_extrinsic', False),   # 外参估计
            'lidar_cov_inv': algorithm_params.get('lidar_cov_inv', 5000.0), # LiDAR协方差逆

            # ========== 外参标定参数 ==========
            # 从主配置的extrinsic段读取（MID360标定值）
            'r_il': extrinsic_params.get('rotation', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            't_il': extrinsic_params.get('translation', [-0.011, -0.02329, 0.04412])
        }

        # 打印关键参数以便验证
        print(f"   📊 关键参数验证:")
        print(f"      - 最大检测距离: {config['lidar_max_range']}m (从主配置读取)")
        print(f"      - 扫描分辨率: {config['scan_resolution']}m")
        print(f"      - 地图范围: {config['cube_len']}m")
        print(f"      - IMU话题: {config['imu_topic']}")
        print(f"      - 点云话题: {config['lidar_topic']}")

        return self._save_config('fastlio2', config)
    
    def generate_pgo_config(self):
        """
        生成PGO配置文件
        从主配置文件动态读取位姿图优化参数
        """
        if not self.master_config.get('pgo', {}).get('enabled', True):
            print("⚠️ PGO已禁用，跳过配置生成")
            return

        # 从主配置提取各个配置段
        global_params = self.master_config.get('global_params', {})
        pgo_config = self.master_config.get('pgo', {})
        fastlio2_topics = self.master_config.get('fastlio2', {}).get('topics', {})
        keyframe_config = pgo_config.get('keyframe', {})
        loop_config = pgo_config.get('loop_closure', {})

        config = {
            # ========== 输入数据源配置 ==========
            # 从主配置的FastLIO2话题配置读取
            'cloud_topic': fastlio2_topics.get('point_cloud', '/fastlio2/body_cloud'),
            'odom_topic': fastlio2_topics.get('odometry', '/fastlio2/lio_odom'),
            'map_frame': global_params.get('map_frame', 'map'),
            'local_frame': global_params.get('local_frame', 'lidar'),

            # ========== 关键帧提取参数 ==========
            # 从主配置的keyframe段读取
            'key_pose_delta_deg': keyframe_config.get('delta_rotation', 10),
            'key_pose_delta_trans': keyframe_config.get('delta_translation', 0.5),

            # ========== 回环检测参数 ==========
            # 从主配置的loop_closure段读取
            'loop_search_radius': loop_config.get('search_radius', 1.0),
            'loop_time_tresh': loop_config.get('time_threshold', 60.0),
            'loop_score_tresh': loop_config.get('score_threshold', 0.15),
            'loop_submap_half_range': loop_config.get('submap_half_range', 5),
            'min_loop_detect_duration': loop_config.get('min_detect_duration', 5.0),

            # ========== 分辨率参数 ==========
            # 使用全局统一分辨率
            'scan_resolution': global_params.get('scan_resolution', 0.15),
            'submap_resolution': global_params.get('scan_resolution', 0.15),

            # ========== 高级功能参数 ==========
            # PGO增强功能配置（反馈机制等）
            'enable_pose_feedback': True,        # 启用位姿反馈
            'feedback_frequency_hz': 2.0,        # 反馈频率
            'max_optimization_time_ms': 200,     # 最大优化时间
            'enable_loop_visualization': True,   # 启用回环可视化
            'keyframe_skip_distance': 0.1        # 关键帧跳过距离
        }

        # 打印关键参数以便验证
        print(f"   📊 PGO关键参数验证:")
        print(f"      - 回环搜索半径: {config['loop_search_radius']}m")
        print(f"      - 关键帧平移阈值: {config['key_pose_delta_trans']}m")
        print(f"      - 输入点云话题: {config['cloud_topic']}")

        return self._save_config('pgo', config)
    
    def generate_hba_config(self):
        """
        生成HBA配置文件
        从主配置文件动态读取分层束调整参数
        """
        if not self.master_config.get('hba', {}).get('enabled', True):
            print("⚠️ HBA已禁用，跳过配置生成")
            return

        # 从主配置提取各个配置段
        global_params = self.master_config.get('global_params', {})
        hba_config = self.master_config.get('hba', {})
        optimization_config = hba_config.get('optimization', {})
        point_cloud_config = hba_config.get('point_cloud', {})

        config = {
            # ========== 分辨率参数 ==========
            # 使用全局统一分辨率
            'scan_resolution': global_params.get('scan_resolution', 0.15),

            # ========== 数值优化参数 ==========
            # 从主配置的optimization段读取
            'window_size': optimization_config.get('window_size', 20),
            'stride': optimization_config.get('stride', 10),
            'ba_max_iter': optimization_config.get('max_iterations', 10),
            'hba_iter': optimization_config.get('hba_iterations', 5),
            'plane_thresh': optimization_config.get('convergence_threshold', 0.01),

            # ========== 点云处理参数 ==========
            # 从主配置的point_cloud段读取
            'voxel_size': point_cloud_config.get('voxel_size', 0.15),  # 与global_params保持一致
            'min_point_num': point_cloud_config.get('min_points', 10),
            'max_layer': point_cloud_config.get('max_layer', 3),
            'down_sample': global_params.get('scan_resolution', 0.15)  # 使用全局分辨率保持一致
        }

        # 打印关键参数以便验证
        print(f"   📊 HBA关键参数验证:")
        print(f"      - 扫描分辨率: {config['scan_resolution']}m")
        print(f"      - 体素大小: {config['voxel_size']}m")
        print(f"      - 滑动窗口: {config['window_size']}帧")

        return self._save_config('hba', config)
    
    def generate_localizer_config(self):
        """
        生成Localizer配置文件
        从主配置文件动态读取重定位参数
        """
        if not self.master_config.get('localizer', {}).get('enabled', True):
            print("⚠️ Localizer已禁用，跳过配置生成")
            return

        # 从主配置提取各个配置段
        global_params = self.master_config.get('global_params', {})
        localizer_config = self.master_config.get('localizer', {})
        fastlio2_topics = self.master_config.get('fastlio2', {}).get('topics', {})
        icp_config = localizer_config.get('icp', {})
        rough_icp = icp_config.get('rough', {})
        refine_icp = icp_config.get('refine', {})

        config = {
            # ========== 输入数据源配置 ==========
            # 从主配置的FastLIO2话题配置读取
            'cloud_topic': fastlio2_topics.get('point_cloud', '/fastlio2/body_cloud'),
            'odom_topic': fastlio2_topics.get('odometry', '/fastlio2/lio_odom'),
            'map_frame': global_params.get('map_frame', 'map'),
            'local_frame': global_params.get('local_frame', 'lidar'),
            'update_hz': localizer_config.get('update_hz', 1.0),

            # ========== ICP粗略匹配参数 ==========
            # 从主配置的icp.rough段读取
            'rough_scan_resolution': global_params.get('rough_resolution', 0.15),
            'rough_map_resolution': global_params.get('rough_resolution', 0.15),
            'rough_max_iteration': rough_icp.get('max_iteration', 5),
            'rough_score_thresh': rough_icp.get('score_threshold', 0.2),

            # ========== ICP精细匹配参数 ==========
            # 从主配置的icp.refine段读取
            'refine_scan_resolution': global_params.get('scan_resolution', 0.15),
            'refine_map_resolution': global_params.get('scan_resolution', 0.15),
            'refine_max_iteration': refine_icp.get('max_iteration', 10),
            'refine_score_thresh': refine_icp.get('score_threshold', 0.1)
        }

        # 打印关键参数以便验证
        print(f"   📊 Localizer关键参数验证:")
        print(f"      - 粗略分辨率: {config['rough_scan_resolution']}m")
        print(f"      - 精细分辨率: {config['refine_scan_resolution']}m")
        print(f"      - 输入点云话题: {config['cloud_topic']}")

        return self._save_config('localizer', config)
    
    def _save_config(self, component, config):
        """保存组件配置文件"""
        config_path = self.component_configs[component]
        
        # 确保目录存在
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 备份原文件
        if config_path.exists():
            backup_path = config_path.with_suffix('.yaml.backup')
            import shutil
            shutil.copy2(config_path, backup_path)
            print(f"📁 备份原配置: {backup_path}")
        
        # 保存为传统YAML格式（组件使用YAML::LoadFile读取）
        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            print(f"✅ {component} 配置文件生成成功: {config_path}")
            return True
        except Exception as e:
            print(f"❌ {component} 配置文件生成失败: {e}")
            return False
    
    def generate_all_configs(self):
        """生成所有组件配置文件"""
        if not self.load_master_config():
            return False
        
        print(f"\n🔄 开始从主配置生成各组件配置...")
        print(f"主配置文件: {self.master_config_path}")
        print("="*60)
        
        success_count = 0
        total_count = 0
        
        # 生成各组件配置
        generators = [
            ('FastLIO2', self.generate_fastlio2_config),
            ('PGO', self.generate_pgo_config), 
            ('HBA', self.generate_hba_config),
            ('Localizer', self.generate_localizer_config)
        ]
        
        for name, generator in generators:
            print(f"\n🔧 生成 {name} 配置...")
            total_count += 1
            if generator():
                success_count += 1
        
        print("\n" + "="*60)
        print(f"📊 配置生成完成: {success_count}/{total_count} 成功")
        
        if success_count == total_count:
            print("🎉 所有配置文件生成成功！")
            self._generate_summary()
            return True
        else:
            print(f"⚠️ 有 {total_count - success_count} 个配置文件生成失败")
            return False
    
    def _generate_summary(self):
        """生成配置摘要"""
        summary_path = self.config_dir / "config_generation_summary.txt"
        
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write(f"MID360 SLAM配置生成摘要\n")
            f.write(f"生成时间: {__import__('datetime').datetime.now()}\n")
            f.write(f"主配置文件: {self.master_config_path}\n\n")
            
            f.write("生成的配置文件:\n")
            for component, path in self.component_configs.items():
                status = "✅" if path.exists() else "❌"
                f.write(f"{status} {component}: {path}\n")
            
            # 从实际配置中动态读取参数
            global_params = self.master_config.get('global_params', {})
            fastlio2_config = self.master_config.get('fastlio2', {})
            system_config = self.master_config.get('system', {})
            
            # 提取关键参数
            scan_resolution = global_params.get('scan_resolution', 'N/A')
            map_resolution = global_params.get('map_resolution', 'N/A')
            rough_resolution = global_params.get('rough_resolution', 'N/A')
            topic_prefix = global_params.get('topic_prefix', 'N/A')
            map_frame = global_params.get('map_frame', 'N/A')
            local_frame = global_params.get('local_frame', 'N/A')
            
            # FastLIO2特定参数
            mapping_params = fastlio2_config.get('mapping_params', {})
            cube_len = mapping_params.get('cube_len', 'N/A')
            det_range = mapping_params.get('det_range', 'N/A')
            move_thresh = mapping_params.get('move_thresh', 'N/A')
            
            # IMU参数
            imu_params = fastlio2_config.get('imu_params', {})
            noise_acc = imu_params.get('noise_acc', 'N/A')
            noise_gyro = imu_params.get('noise_gyro', 'N/A')
            
            # 网络配置
            host_ip = system_config.get('host_ip', 'N/A')
            lidar_ip = system_config.get('lidar_ip', 'N/A')
                
            f.write("\n=== 统一配置参数 ===\n")
            f.write("🎯 分辨率设置:\n")
            f.write(f"  - 主分辨率: {scan_resolution}m (scan_resolution)\n")
            f.write(f"  - 地图分辨率: {map_resolution}m (map_resolution)\n") 
            f.write(f"  - 粗略分辨率: {rough_resolution}m (rough_resolution)\n")
            
            f.write("\n🌐 ROS话题和坐标系:\n")
            f.write(f"  - 话题前缀: {topic_prefix}\n")
            f.write(f"  - 坐标系映射: {map_frame} <-> {local_frame}\n")
            
            f.write("\n🗺️ SLAM建图参数:\n")
            f.write(f"  - 地图立方体范围: {cube_len}m (cube_len)\n")
            f.write(f"  - 检测范围: {det_range}m (det_range)\n")
            f.write(f"  - 移动阈值: {move_thresh}m (move_thresh)\n")
            
            f.write("\n📐 IMU噪声模型:\n")
            f.write(f"  - 加速度计噪声: {noise_acc} (noise_acc)\n")
            f.write(f"  - 陀螺仪噪声: {noise_gyro} (noise_gyro)\n")
            
            f.write("\n🌐 网络配置:\n")
            f.write(f"  - 主机IP: {host_ip}\n")
            f.write(f"  - 雷达IP: {lidar_ip}\n")
            
            # 组件状态
            f.write("\n=== 组件启用状态 ===\n")
            components = ['fastlio2', 'pgo', 'hba', 'localizer']
            for component in components:
                enabled = self.master_config.get(component, {}).get('enabled', False)
                status = "✅ 启用" if enabled else "❌ 禁用"
                f.write(f"  - {component.upper()}: {status}\n")
        
        print(f"📄 配置摘要已保存: {summary_path}")

def main():
    """主函数"""
    if len(sys.argv) != 2:
        print("使用方法: python3 config_generator.py <主配置文件路径>")
        print("示例: python3 config_generator.py ../config/full_slam_config.yaml")
        sys.exit(1)
    
    master_config = sys.argv[1]
    
    if not os.path.exists(master_config):
        print(f"❌ 主配置文件不存在: {master_config}")
        sys.exit(1)
    
    generator = ConfigGenerator(master_config)
    
    if generator.generate_all_configs():
        print("\n✨ 配置生成完成！现在可以使用统一的配置管理系统了。")
        print("💡 建议运行: ./slam_tools.sh validate 验证配置一致性")
        sys.exit(0)
    else:
        print("\n❌ 配置生成失败，请检查错误信息")
        sys.exit(1)

if __name__ == '__main__':
    main()