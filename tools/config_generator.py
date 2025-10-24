#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MID360 SLAM系统配置生成器
从统一主配置文件生成各组件配置文件，解决配置不一致问题
"""

import os
import sys
import json
import yaml
import argparse
import shutil
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List

class ConfigGenerator:
    """配置生成器类"""

    def __init__(self, master_config_path: str):
        """初始化配置生成器

        Args:
            master_config_path: 主配置文件路径
        """
        self.master_config_path = master_config_path
        self.master_config = None
        self.project_root = Path(__file__).parent.parent
        self.backup_dir = self.project_root / "config" / "backups"

        # 确保备份目录存在
        self.backup_dir.mkdir(parents=True, exist_ok=True)

    def load_master_config(self) -> bool:
        """加载主配置文件

        Returns:
            是否加载成功
        """
        try:
            with open(self.master_config_path, 'r', encoding='utf-8') as f:
                self.master_config = yaml.safe_load(f)
            print(f"✓ 成功加载主配置文件: {self.master_config_path}")
            return True
        except Exception as e:
            print(f"✗ 加载主配置文件失败: {e}")
            return False

    def backup_existing_configs(self) -> None:
        """备份现有配置文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_subdir = self.backup_dir / f"backup_{timestamp}"
        backup_subdir.mkdir(exist_ok=True)

        config_files = [
            "ws_livox/src/fastlio2/config/lio.yaml",
            "ws_livox/src/livox_ros_driver2/config/MID360_config.json",
            "config/launch_config.yaml",
            "ws_livox/src/pgo/config/pgo.yaml",
            "ws_livox/src/hba/config/hba.yaml",
            "ws_livox/src/localizer/config/localizer.yaml",
            "ws_livox/src/point_cloud_filter/config/point_cloud_filter.yaml",
            "ws_livox/src/cooperation/config/coordinator.yaml",
            "ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz"
        ]

        for config_file in config_files:
            src_path = self.project_root / config_file
            if src_path.exists():
                dst_path = backup_subdir / src_path.name
                shutil.copy2(src_path, dst_path)
                print(f"✓ 备份配置文件: {src_path.name} -> {dst_path}")

    def generate_fastlio2_config(self) -> bool:
        """生成FAST-LIO2配置文件 (lio.yaml)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config

            pc_cfg = mc['fastlio2'].get('point_cloud', {})
            buffer_cfg = mc['fastlio2'].get('buffer_management', {})

            # 构建lio.yaml配置
            lio_config = {
                # 基本话题配置
                'imu_topic': mc['topics']['imu'],
                'lidar_topic': mc['topics']['lidar'],
                'body_frame': mc['frames']['base_frame'],
                'world_frame': mc['frames']['world_frame'],
                'print_time_cost': mc['fastlio2']['print_time_cost'],

                # 雷达参数
                'lidar_filter_num': mc['fastlio2']['lidar_filter_num'],
                'lidar_min_range': mc['fastlio2']['lidar_min_range'],
                'lidar_max_range': mc['fastlio2']['lidar_max_range'],
                'scan_resolution': mc['fastlio2']['scan_resolution'],
                'map_resolution': mc['fastlio2']['map_resolution'],

                # 地图管理
                'cube_len': mc['fastlio2']['cube_len'],
                'det_range': mc['fastlio2']['det_range'],
                'move_thresh': mc['fastlio2']['move_thresh'],

                # IMU噪声参数
                'na': mc['fastlio2']['imu_noise']['accelerometer'],
                'ng': mc['fastlio2']['imu_noise']['gyroscope'],
                'nba': mc['fastlio2']['imu_noise']['acc_bias'],
                'nbg': mc['fastlio2']['imu_noise']['gyro_bias'],

                # 算法参数
                'imu_init_num': mc['fastlio2']['algorithm']['imu_init_num'],
                'near_search_num': mc['fastlio2']['algorithm']['near_search_num'],
                'ieskf_max_iter': mc['fastlio2']['algorithm']['ieskf_max_iter'],
                'gravity_align': mc['fastlio2']['algorithm']['gravity_align'],
                'esti_il': mc['fastlio2']['algorithm']['esti_il'],

                # 外参配置（关键！）
                'r_il': mc['calibration']['lidar_to_imu']['rotation'],
                't_il': mc['calibration']['lidar_to_imu']['translation'],

                # 点云匹配参数
                'lidar_cov_inv': pc_cfg.get('lidar_cov_inv', 2000.0),
                'lidar_cov_scale': pc_cfg.get('lidar_cov_scale', 1.0),
                'point_filter_num': pc_cfg.get('point_filter_num', 4),
                'converge_thresh': pc_cfg.get('converge_thresh', 1e-4),

                # 缓冲区管理参数 (内存安全和线程优化)
                'max_imu_buffer_size': buffer_cfg.get('max_imu_buffer_size', 8000),
                'max_lidar_buffer_size': buffer_cfg.get('max_lidar_buffer_size', 1000),
                'enable_buffer_monitoring': buffer_cfg.get('enable_buffer_monitoring', True),
            }

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/fastlio2/config/lio.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入配置头注释
                f.write("# FAST-LIO2配置文件\n")
                f.write("# 由配置生成器自动生成，请勿手动修改\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 主配置文件: {self.master_config_path}\n\n")

                # 写入配置内容
                yaml.dump(lio_config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 成功生成FAST-LIO2配置: {output_path}")
            # 同步到安装目录（如存在且非符号链接则覆盖），避免 FindPackageShare 读取旧文件
            try:
                install_cfg = self.project_root / "install/fastlio2/share/fastlio2/config/lio.yaml"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败（可忽略，建议使用 --symlink-install 或重新编译）: {e}")
            return True

        except Exception as e:
            print(f"✗ 生成FAST-LIO2配置失败: {e}")
            return False

    def generate_livox_config(self) -> bool:
        """生成Livox驱动配置文件 (MID360_config.json)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config
            net = mc['network']
            drv = mc['livox_driver']

            # 构建MID360_config.json配置
            livox_config = {
                "lidar_summary_info": {
                    "lidar_type": 8
                },
                "MID360": {
                    "lidar_net_info": {
                        "cmd_data_port": net['lidar']['lidar_ports']['cmd_data'],
                        "push_msg_port": net['lidar']['lidar_ports']['push_msg'],
                        "point_data_port": net['lidar']['lidar_ports']['point_data'],
                        "imu_data_port": net['lidar']['lidar_ports']['imu_data'],
                        "log_data_port": net['lidar']['lidar_ports']['log_data']
                    },
                    "host_net_info": {
                        "cmd_data_ip": net['host']['ip'],
                        "cmd_data_port": net['lidar']['host_ports']['cmd_data'],
                        "push_msg_ip": net['host']['ip'],
                        "push_msg_port": net['lidar']['host_ports']['push_msg'],
                        "point_data_ip": net['host']['ip'],
                        "point_data_port": net['lidar']['host_ports']['point_data'],
                        "imu_data_ip": net['host']['ip'],
                    "imu_data_port": net['lidar']['host_ports']['imu_data'],
                    "log_data_ip": "",
                    "log_data_port": net['lidar']['host_ports']['log_data']
                    }
                },
                "driver_params": {
                    "xfer_format": drv.get("xfer_format", 1),
                    "multi_topic": drv.get("multi_topic", 0),
                    "data_src": drv.get("data_src", 0),
                    "publish_freq": drv.get("publish_freq", 10.0),
                    "output_data_type": drv.get("output_data_type", 0),
                    "frame_id": drv.get("frame_id", "livox_frame"),
                    "cmdline_input_bd_code": drv.get("cmdline_input_bd_code", "")
                },
                "lidar_configs": [
                    {
                        "ip": net['lidar']['ip'],
                        "pcl_data_type": drv['lidar_config']['pcl_data_type'],
                        "pattern_mode": drv['lidar_config']['pattern_mode'],
                        "extrinsic_parameter": {
                            "roll": mc['calibration']['lidar_to_baselink']['rotation_rpy'][0],
                            "pitch": mc['calibration']['lidar_to_baselink']['rotation_rpy'][1],
                            "yaw": mc['calibration']['lidar_to_baselink']['rotation_rpy'][2],
                            "x": mc['calibration']['lidar_to_baselink']['translation'][0],
                            "y": mc['calibration']['lidar_to_baselink']['translation'][1],
                            "z": mc['calibration']['lidar_to_baselink']['translation'][2]
                        }
                    }
                ]
            }

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/livox_ros_driver2/config/MID360_config.json"
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(livox_config, f, indent=2, ensure_ascii=False)

            print(f"✓ 成功生成Livox驱动配置: {output_path}")
            return True

        except Exception as e:
            print(f"✗ 生成Livox驱动配置失败: {e}")
            return False

    def generate_launch_config(self) -> bool:
        """生成启动配置文件 (launch_config.yaml)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config

            # 构建launch_config.yaml配置
            launch_config = {
                "# MID360 启动配置文件": None,
                "# 由配置生成器自动生成，请勿手动修改": None,

                "network": {
                    "host_ip": mc['network']['host']['ip'],
                    "lidar_ip": mc['network']['lidar']['ip'],
                    "gateway": mc['network']['host']['gateway'],
                    "netmask": mc['network']['host']['netmask'],
                    "interface": mc['network']['host']['interface'],
                    "ports": {
                        "cmd_data": mc['network']['lidar']['lidar_ports']['cmd_data'],
                        "push_msg": mc['network']['lidar']['lidar_ports']['push_msg'],
                        "point_data": mc['network']['lidar']['lidar_ports']['point_data'],
                        "imu_data": mc['network']['lidar']['lidar_ports']['imu_data'],
                        "log_data": mc['network']['lidar']['lidar_ports']['log_data']
                    }
                },

                "lidar": {
                    "pcl_data_type": mc['livox_driver']['lidar_config']['pcl_data_type'],
                    "publish_freq": mc['livox_driver']['publish_freq'],
                    "frame_id": mc['livox_driver']['frame_id'],
                    "xfer_format": mc['livox_driver']['xfer_format'],
                    "multi_topic": mc['livox_driver']['multi_topic'],
                    "data_src": mc['livox_driver']['data_src'],
                    "output_data_type": mc['livox_driver']['output_data_type'],
                    "cmdline_input_bd_code": mc['livox_driver'].get('cmdline_input_bd_code', ''),
                    "extrinsic": {
                        "x": mc['calibration']['lidar_to_baselink']['translation'][0],
                        "y": mc['calibration']['lidar_to_baselink']['translation'][1],
                        "z": mc['calibration']['lidar_to_baselink']['translation'][2],
                        "roll": mc['calibration']['lidar_to_baselink']['rotation_rpy'][0],
                        "pitch": mc['calibration']['lidar_to_baselink']['rotation_rpy'][1],
                        "yaw": mc['calibration']['lidar_to_baselink']['rotation_rpy'][2]
                    }
                },

                "rviz": mc['visualization']['rviz'],
                "rosbag": mc['rosbag'],
                "debug": {
                    "log_level": mc['monitoring']['logging']['level'],
                    "verbose": mc['monitoring']['logging']['verbose'],
                    "performance_monitor": mc['monitoring']['performance']['enable_metrics']
                }
            }

            # 生成配置文件
            output_path = self.project_root / "config/launch_config.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(launch_config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 成功生成启动配置: {output_path}")
            return True

        except Exception as e:
            print(f"✗ 生成启动配置失败: {e}")
            return False

    def validate_config(self) -> bool:
        """验证配置有效性

        Returns:
            配置是否有效
        """
        try:
            mc = self.master_config
            validation = mc.get('validation', {})
            constraints = validation.get('constraints', {})

            issues = []

            # 验证参数范围
            if 'lidar_max_range' in constraints:
                max_range = mc['fastlio2']['lidar_max_range']
                min_val = constraints['lidar_max_range']['min']
                max_val = constraints['lidar_max_range']['max']
                if not (min_val <= max_range <= max_val):
                    issues.append(f"lidar_max_range ({max_range}) 超出范围 [{min_val}, {max_val}]")

            if 'publish_freq' in constraints:
                freq = mc['livox_driver']['publish_freq']
                min_val = constraints['publish_freq']['min']
                max_val = constraints['publish_freq']['max']
                if not (min_val <= freq <= max_val):
                    issues.append(f"publish_freq ({freq}) 超出范围 [{min_val}, {max_val}]")

            if 'imu_init_num' in constraints:
                imu_init = mc['fastlio2']['algorithm']['imu_init_num']
                min_val = constraints['imu_init_num']['min']
                max_val = constraints['imu_init_num']['max']
                if not (min_val <= imu_init <= max_val):
                    issues.append(f"imu_init_num ({imu_init}) 超出范围 [{min_val}, {max_val}]")

            # 验证外参一致性
            lidar_to_imu_trans = mc['calibration']['lidar_to_imu']['translation']
            if len(lidar_to_imu_trans) != 3:
                issues.append("lidar_to_imu translation必须包含3个元素")

            lidar_to_imu_rot = mc['calibration']['lidar_to_imu']['rotation']
            if len(lidar_to_imu_rot) != 9:
                issues.append("lidar_to_imu rotation必须包含9个元素")

            # 验证网络配置
            host_ip = mc['network']['host']['ip']
            lidar_ip = mc['network']['lidar']['ip']
            if not self._is_valid_ip(host_ip):
                issues.append(f"无效的主机IP地址: {host_ip}")
            if not self._is_valid_ip(lidar_ip):
                issues.append(f"无效的雷达IP地址: {lidar_ip}")

            if issues:
                print("✗ 配置验证失败:")
                for issue in issues:
                    print(f"  - {issue}")
                return False
            else:
                print("✓ 配置验证通过")
                return True

        except Exception as e:
            print(f"✗ 配置验证出错: {e}")
            return False

    def _is_valid_ip(self, ip: str) -> bool:
        """验证IP地址格式

        Args:
            ip: IP地址字符串

        Returns:
            是否为有效IP地址
        """
        try:
            parts = ip.split('.')
            return len(parts) == 4 and all(0 <= int(part) <= 255 for part in parts)
        except:
            return False

    def generate_pgo_config(self) -> bool:
        """生成PGO配置文件 (pgo.yaml)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config
            pgo_master = mc.get('pgo', {})

            # 构建pgo.yaml配置
            pgo_config = {
                # 基本话题配置
                'cloud_topic': pgo_master['cloud_topic'],
                'odom_topic': pgo_master['odom_topic'],
                'map_frame': pgo_master['map_frame'],
                'local_frame': pgo_master['local_frame'],
                'log_level': pgo_master.get('log_level', "INFO"),

                # 关键帧参数
                'key_pose_delta_deg': pgo_master['key_pose_delta_deg'],
                'key_pose_delta_trans': pgo_master['key_pose_delta_trans'],

                # 回环检测参数
                'loop_search_radius': pgo_master['loop_search_radius'],
                'loop_time_tresh': pgo_master['loop_time_tresh'],
                'loop_score_tresh': pgo_master['loop_score_tresh'],
                'loop_submap_half_range': pgo_master['loop_submap_half_range'],
                'submap_resolution': pgo_master['submap_resolution'],
                'min_loop_detect_duration': pgo_master.get('min_loop_detect_duration', 0.0),

                # 高级ICP/回环与优化参数
                'min_inlier_ratio': pgo_master.get('min_inlier_ratio', 0.30),
                'icp_max_distance': pgo_master.get('icp_max_distance', 2.0),
                'max_candidates': pgo_master.get('max_candidates', 5),
                'min_index_separation': pgo_master.get('min_index_separation', 10),
                'inlier_threshold': pgo_master.get('inlier_threshold', 0.30),
                'icp_max_iterations': pgo_master.get('icp_max_iterations', 50),
                'icp_transformation_epsilon': pgo_master.get('icp_transformation_epsilon', 1e-6),
                'icp_euclidean_fitness_epsilon': pgo_master.get('icp_euclidean_fitness_epsilon', 1e-6),
                'isam_relinearize_threshold': pgo_master.get('isam_relinearize_threshold', 0.01),
                'isam_relinearize_skip': pgo_master.get('isam_relinearize_skip', 1),
                'loop_retry_max_attempts': pgo_master.get('loop_retry_max_attempts', 3),
                'loop_retry_interval': pgo_master.get('loop_retry_interval', 5.0),
                'loop_retry_capacity': pgo_master.get('loop_retry_capacity', 32),
                'loop_retry_score_relax': pgo_master.get('loop_retry_score_relax', 1.5),
                'loop_retry_inlier_relax': pgo_master.get('loop_retry_inlier_relax', 0.7),
            }

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/pgo/config/pgo.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入配置头注释
                f.write("# PGO配置文件\n")
                f.write("# 由配置生成器自动生成，请勿手动修改\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 主配置文件: {self.master_config_path}\n\n")

                # 写入配置内容
                yaml.dump(pgo_config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 成功生成PGO配置: {output_path}")
            try:
                install_cfg = self.project_root / "install/pgo/share/pgo/config/pgo.yaml"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败: {e}")
            return True

        except Exception as e:
            print(f"✗ 生成PGO配置失败: {e}")
            return False

    def generate_hba_config(self) -> bool:
        """生成HBA配置文件 (hba.yaml)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config

            # 构建hba.yaml配置
            hba_config = {
                'scan_resolution': mc['hba']['scan_resolution'],
                'window_size': mc['hba']['window_size'],
                'stride': mc['hba']['stride'],
                'voxel_size': mc['hba']['voxel_size'],
                'min_point_num': mc['hba']['min_point_num'],
                'max_layer': mc['hba']['max_layer'],
                'plane_thresh': mc['hba']['plane_thresh'],
                'ba_max_iter': mc['hba']['ba_max_iter'],
                'hba_iter': mc['hba']['hba_iter'],
                'down_sample': mc['hba']['down_sample'],
            }

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/hba/config/hba.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入配置头注释
                f.write("# HBA配置文件\n")
                f.write("# 由配置生成器自动生成，请勿手动修改\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 主配置文件: {self.master_config_path}\n\n")

                # 写入配置内容
                yaml.dump(hba_config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 成功生成HBA配置: {output_path}")
            try:
                install_cfg = self.project_root / "install/hba/share/hba/config/hba.yaml"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败: {e}")
            return True

        except Exception as e:
            print(f"✗ 生成HBA配置失败: {e}")
            return False

    def generate_localizer_config(self) -> bool:
        """生成Localizer配置文件 (localizer.yaml)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config

            # 构建localizer.yaml配置
            localizer_config = {
                # 基本话题配置
                'cloud_topic': mc['localizer']['cloud_topic'],
                'odom_topic': mc['localizer']['odom_topic'],
                'map_frame': mc['localizer']['map_frame'],
                'local_frame': mc['localizer']['local_frame'],
                'update_hz': mc['localizer']['update_hz'],

                # 粗定位参数
                'rough_scan_resolution': mc['localizer']['rough_localization']['scan_resolution'],
                'rough_map_resolution': mc['localizer']['rough_localization']['map_resolution'],
                'rough_max_iteration': mc['localizer']['rough_localization']['max_iteration'],
                'rough_score_thresh': mc['localizer']['rough_localization']['score_thresh'],

                # 精细定位参数
                'refine_scan_resolution': mc['localizer']['refine_localization']['scan_resolution'],
                'refine_map_resolution': mc['localizer']['refine_localization']['map_resolution'],
                'refine_max_iteration': mc['localizer']['refine_localization']['max_iteration'],
                'refine_score_thresh': mc['localizer']['refine_localization']['score_thresh'],
            }

            # 添加动态过滤器配置（如果存在）
            # 注意：Localizer节点期望YAML中为嵌套的dynamic_filter: {...}
            if 'dynamic_filter' in mc['localizer']:
                dynamic_filter_config = mc['localizer']['dynamic_filter'].copy()
                localizer_config['dynamic_filter'] = dynamic_filter_config

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/localizer/config/localizer.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入配置头注释
                f.write("# Localizer配置文件\n")
                f.write("# 由配置生成器自动生成，请勿手动修改\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 主配置文件: {self.master_config_path}\n\n")

                # 写入配置内容
                yaml.dump(localizer_config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 成功生成Localizer配置: {output_path}")
            try:
                install_cfg = self.project_root / "install/localizer/share/localizer/config/localizer.yaml"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败: {e}")
            return True

        except Exception as e:
            print(f"✗ 生成Localizer配置失败: {e}")
            return False

    def generate_point_cloud_filter_config(self) -> bool:
        """生成点云过滤桥接节点的ROS2参数文件 (point_cloud_filter.yaml)

        说明：采用 ROS2 node-scoped 参数结构，便于通过 launch parameters=[path] 直接加载。
        - 输入默认指向原始话题 /livox/lidar
        - 输出指向主配置 topics.lidar（通常为 /livox/lidar_filtered）
        - 动态过滤参数复用 master_config 中 localizer.dynamic_filter 的设置（保持一致性）
        """
        try:
            mc = self.master_config

            # 期望输出话题（供下游使用，如FAST-LIO2）
            output_topic = mc['topics']['lidar']
            # 原始输入固定为驱动输出原始话题
            input_topic = "/livox/lidar"

            # 动态过滤器参数（来自 localizer 配置，保持一致性）
            df = mc.get('localizer', {}).get('dynamic_filter', {})

            # 兼容字段读取（提供合理默认）
            def get_df(k, default):
                return df.get(k, default)

            df_enable = bool(df.get('enable', True))
            params = {
                'point_cloud_filter_bridge': {
                    'ros__parameters': {
                        # 话题
                        'input_topic': input_topic,
                        'output_topic': output_topic,
                        'debug_topic': '/point_cloud_filter/debug_cloud',
                        'stats_topic': '/point_cloud_filter/filter_stats',

                        # 控制
                        'debug_enabled': bool(df.get('debug_enabled', False)),
                        'publish_stats': bool(df.get('publish_stats', True)),
                        'max_processing_hz': float(df.get('max_processing_hz', 20.0)),

                        # 动态过滤器
                        'dynamic_filter': {
                            'enable': df_enable,
                            'publish_stats': bool(df.get('publish_stats', True)),
                            'motion_threshold': float(get_df('motion_threshold', 0.1)),
                            'history_size': int(get_df('history_size', 8)),
                            'stability_threshold': float(get_df('stability_threshold', 0.75)),
                            'max_time_diff': float(get_df('max_time_diff', 1.0)),

                            'search_radius': float(get_df('search_radius', 0.2)),
                            'min_neighbors': int(get_df('min_neighbors', 10)),
                            'normal_consistency_thresh': float(get_df('normal_consistency_thresh', 0.8)),
                            'density_ratio_thresh': float(get_df('density_ratio_thresh', 0.5)),

                            'downsample_ratio': int(get_df('downsample_ratio', 1)),
                            'max_points_per_frame': int(get_df('max_points_per_frame', 60000)),
                            'voxel_size_base': float(get_df('voxel_size_base', 0.01)),
                        }
                    }
                }
            }

            # 写入文件
            output_path = self.project_root / "ws_livox/src/point_cloud_filter/config/point_cloud_filter.yaml"
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(params, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            print(f"✓ 成功生成点云过滤参数: {output_path}")

            # 尝试同步到安装目录（若存在且非符号链接）
            try:
                install_cfg = self.project_root / "ws_livox/install/point_cloud_filter/share/point_cloud_filter/config/point_cloud_filter.yaml"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败: {e}")

            return True
        except Exception as e:
            print(f"✗ 生成点云过滤参数失败: {e}")
            return False

    def generate_cooperation_config(self) -> bool:
        """生成协同优化协调器配置 (coordinator.yaml)

        Returns:
            是否生成成功
        """
        try:
            coop = self.master_config.get('cooperation', {})

            def get(key, default):
                return coop.get(key, default)

            raw_hba_path = get('hba_maps_path', str(self.project_root / "saved_maps" / "hba_workspace"))
            hba_path = Path(raw_hba_path)
            if not hba_path.is_absolute():
                hba_path = (self.project_root / hba_path).resolve()

            params = {
                'coordinator': {
                    'ros__parameters': {
                        'drift_threshold': float(get('drift_threshold', 0.5)),
                        'time_threshold': float(get('time_threshold', 60.0)),
                        'emergency_threshold': float(get('emergency_threshold', 2.0)),
                        'auto_optimization': bool(get('auto_optimization', True)),
                        'service_timeout_ms': int(get('service_timeout_ms', 3000)),
                        'emergency_failure_threshold': int(get('emergency_failure_threshold', 3)),
                        'hba_maps_path': str(hba_path),
                        'enable_relocalization': bool(get('enable_relocalization', True)),
                        'min_pose_delta': float(get('min_pose_delta', 0.03)),
                        'min_orientation_delta_deg': float(get('min_orientation_delta_deg', 0.5)),
                        'optimization_score_threshold': float(get('optimization_score_threshold', 0.6)),
                        'drift_history_size': int(get('drift_history_size', 20)),
                        'relocalization_score_threshold': float(get('relocalization_score_threshold', 0.15)),
                        'relocalization_timeout_sec': float(get('relocalization_timeout_sec', 10.0)),
                        'relocalization_min_translation': float(get('relocalization_min_translation', 0.5)),
                        'relocalization_min_yaw': float(get('relocalization_min_yaw', 5.0))
                    }
                }
            }

            output_path = self.project_root / "ws_livox/src/cooperation/config/coordinator.yaml"
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(params, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            print(f"✓ 成功生成协同协调器配置: {output_path}")
            return True
        except Exception as e:
            print(f"✗ 生成协同配置失败: {e}")
            return False

    def generate_rviz_config(self) -> bool:
        """生成RViz配置文件 (enhanced_fastlio2.rviz)

        Returns:
            是否生成成功
        """
        try:
            mc = self.master_config

            # RViz配置模板 (基于当前配置文件的结构)
            rviz_config = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /PointCloud21
        - /PointCloud22
        - /Path1
        - /Marker1
        - /Loop Closures1
        - /HBA Map1
        - /Saved Map1
      Splitter Ratio: 0.5
    Tree Height: 1079
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 0.8
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 0; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 237
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: "Saved Map"
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 1
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /saved_map
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 237
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: "Body Point Cloud"
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 10
        Durability Policy: Volatile
        Filter size: 100
        History Policy: Keep All
        Reliability Policy: Reliable
        Value: {mc['topics']['body_cloud']}
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.3
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 60
      Enabled: false
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 237
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: "World Point Cloud"
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 1
      Size (m): 0.01
      Style: Points
      Topic:
        Depth: 50
        Durability Policy: Volatile
        Filter size: 1000
        History Policy: Keep All
        Reliability Policy: Reliable
        Value: {mc['topics']['world_cloud']}
      Use Fixed Frame: true
      Use rainbow: true
      Value: false
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: "SLAM Path"
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {mc['topics']['path']}
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: "IMU Pose Marker"
      Namespaces:
        {{}}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /fastlio2/imu_pose_marker
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: "Loop Closures"
      Namespaces:
        {{}}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {mc['topics'].get('loop_markers', '/slam/loop_markers')}
      Value: true
    - Alpha: 0.6
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 0
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 237
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: "HBA Map"
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 10
        Durability Policy: Volatile
        Filter size: 100
        History Policy: Keep All
        Reliability Policy: Reliable
        Value: {mc['topics'].get('hba_map', '/hba/map_points')}
      Use Fixed Frame: true
      Use rainbow: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: {mc['frames']['world_frame']}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 19.600534439086914
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1376
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000051efc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000051e000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730020000000000000016a0000010e00fffffffb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000051efc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d0000051e000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007800000003efc0100000002fb0000000800540069006d00650100000000000007800000025b00fffffffb0000000800540069006d006501000000000000045000000000000000000000061f0000051e00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1920
"""

            # 生成配置文件
            output_path = self.project_root / "ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz"
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入配置头注释
                f.write("# RViz配置文件\n")
                f.write("# 由配置生成器自动生成，请勿手动修改\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 主配置文件: {self.master_config_path}\n\n")

                # 写入配置内容
                f.write(rviz_config)

            print(f"✓ 成功生成RViz配置: {output_path}")
            # 尝试同步到安装目录，便于通过 FindPackageShare 读取最新配置
            try:
                install_cfg = self.project_root / "install/fastlio2/share/fastlio2/rviz/enhanced_fastlio2.rviz"
                if install_cfg.exists() and not install_cfg.is_symlink():
                    install_cfg.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(output_path, install_cfg)
                    print(f"  ↳ 已同步到安装目录: {install_cfg}")
            except Exception as e:
                print(f"  ⚠ 同步到安装目录失败（可忽略，建议使用 --symlink-install 或重新编译）: {e}")
            return True

        except Exception as e:
            print(f"✗ 生成RViz配置失败: {e}")
            return False

    def generate_all_configs(self) -> bool:
        """生成所有配置文件

        Returns:
            是否全部生成成功
        """
        print("开始生成配置文件...")
        print("="*50)

        # 备份现有配置
        self.backup_existing_configs()
        print()

        # 验证主配置
        if not self.validate_config():
            return False
        print()

        # 生成各组件配置
        success = True
        success &= self.generate_fastlio2_config()
        success &= self.generate_livox_config()
        success &= self.generate_launch_config()
        success &= self.generate_pgo_config()
        success &= self.generate_hba_config()
        success &= self.generate_localizer_config()
        success &= self.generate_point_cloud_filter_config()
        success &= self.generate_cooperation_config()
        success &= self.generate_rviz_config()

        print()
        if success:
            print("✓ 所有配置文件生成成功！")
            print("✓ 外参配置已统一，应该能解决轨迹飘移问题")
            print("✓ 所有SLAM组件参数已统一管理")
        else:
            print("✗ 部分配置文件生成失败")

        return success

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='MID360 SLAM系统配置生成器')
    parser.add_argument('--master-config', '-m',
                        default='config/master_config.yaml',
                        help='主配置文件路径 (默认: config/master_config.yaml)')
    parser.add_argument('--validate-only', '-v',
                        action='store_true',
                        help='仅验证配置，不生成文件')

    args = parser.parse_args()

    # 确定项目根目录和主配置文件路径
    project_root = Path(__file__).parent.parent
    master_config_path = project_root / args.master_config

    if not master_config_path.exists():
        print(f"✗ 主配置文件不存在: {master_config_path}")
        sys.exit(1)

    # 创建配置生成器
    generator = ConfigGenerator(str(master_config_path))

    # 加载主配置
    if not generator.load_master_config():
        sys.exit(1)

    if args.validate_only:
        # 仅验证配置
        if generator.validate_config():
            print("配置验证通过")
            sys.exit(0)
        else:
            print("配置验证失败")
            sys.exit(1)
    else:
        # 生成所有配置文件
        if generator.generate_all_configs():
            sys.exit(0)
        else:
            sys.exit(1)

if __name__ == "__main__":
    main()
