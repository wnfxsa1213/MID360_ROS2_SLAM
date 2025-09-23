"""
点云文件输入输出工具
"""

import os
import numpy as np
import open3d as o3d
from typing import Optional, Tuple, List
import yaml
import logging

logger = logging.getLogger(__name__)

class PointCloudIO:
    """点云文件IO处理类"""
    
    @staticmethod
    def load_point_cloud(file_path: str) -> o3d.geometry.PointCloud:
        """
        加载点云文件
        
        Args:
            file_path: 点云文件路径，支持.pcd, .ply, .xyz格式
            
        Returns:
            open3d点云对象
            
        Raises:
            FileNotFoundError: 文件不存在
            ValueError: 不支持的文件格式
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"点云文件不存在: {file_path}")
            
        # 检查文件格式
        ext = os.path.splitext(file_path)[1].lower()
        if ext not in ['.pcd', '.ply', '.xyz']:
            raise ValueError(f"不支持的文件格式: {ext}")
            
        logger.info(f"加载点云文件: {file_path}")
        
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            if len(pcd.points) == 0:
                raise ValueError("点云为空")
                
            logger.info(f"成功加载点云，包含 {len(pcd.points)} 个点")
            return pcd
            
        except Exception as e:
            logger.error(f"加载点云失败: {str(e)}")
            raise
            
    @staticmethod
    def save_point_cloud(pcd: o3d.geometry.PointCloud, file_path: str, 
                        write_ascii: bool = False) -> bool:
        """
        保存点云文件
        
        Args:
            pcd: open3d点云对象
            file_path: 保存路径
            write_ascii: 是否保存为ASCII格式
            
        Returns:
            是否保存成功
        """
        try:
            # 确保输出目录存在
            output_dir = os.path.dirname(file_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            success = o3d.io.write_point_cloud(file_path, pcd, write_ascii=write_ascii)
            
            if success:
                logger.info(f"成功保存点云到: {file_path}")
            else:
                logger.error(f"保存点云失败: {file_path}")
                
            return success
            
        except Exception as e:
            logger.error(f"保存点云异常: {str(e)}")
            return False
            
    @staticmethod
    def load_config(config_path: str) -> dict:
        """
        加载YAML配置文件
        
        Args:
            config_path: 配置文件路径
            
        Returns:
            配置字典
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            logger.info(f"成功加载配置文件: {config_path}")
            return config
        except Exception as e:
            logger.error(f"加载配置文件失败: {str(e)}")
            raise
            
    @staticmethod
    def save_config(config: dict, config_path: str):
        """
        保存配置文件
        
        Args:
            config: 配置字典
            config_path: 保存路径
        """
        try:
            output_dir = os.path.dirname(config_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            logger.info(f"成功保存配置文件: {config_path}")
        except Exception as e:
            logger.error(f"保存配置文件失败: {str(e)}")
            raise
            
    @staticmethod
    def get_point_cloud_info(pcd: o3d.geometry.PointCloud) -> dict:
        """
        获取点云基本信息
        
        Args:
            pcd: open3d点云对象
            
        Returns:
            点云信息字典
        """
        info = {
            'num_points': len(pcd.points),
            'has_colors': pcd.has_colors(),
            'has_normals': pcd.has_normals(),
            'bounding_box': {
                'min': np.asarray(pcd.get_min_bound()).tolist(),
                'max': np.asarray(pcd.get_max_bound()).tolist()
            }
        }
        
        if len(pcd.points) > 0:
            points = np.asarray(pcd.points)
            info['center'] = np.mean(points, axis=0).tolist()
            info['std'] = np.std(points, axis=0).tolist()
            
        return info
        
    @staticmethod
    def create_output_directory(base_path: str, suffix: str = "refined") -> str:
        """
        创建输出目录
        
        Args:
            base_path: 基础路径
            suffix: 后缀名
            
        Returns:
            输出目录路径
        """
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"{base_path}_{suffix}_{timestamp}"
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        return output_dir