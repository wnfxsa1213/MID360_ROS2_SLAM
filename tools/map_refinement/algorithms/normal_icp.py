import copy
import numpy as np
import open3d as o3d
from typing import Tuple, Optional
import logging
from sklearn.neighbors import NearestNeighbors
"""
法向量增强ICP配准算法

实现基于法向量信息的点云配准算法，相比传统ICP具有更高的精度和鲁棒性。
核心思想：使用点到平面距离代替点到点距离，结合法向量角度权重。
"""


logger = logging.getLogger(__name__)

class NormalICP:
    """法向量增强ICP算法类"""
    
    def __init__(self, config: dict):
        """
        初始化法向量ICP算法
        
        Args:
            config: 算法配置字典
        """
        self.max_iterations = config.get('max_iterations', 50)
        self.convergence_threshold = config.get('convergence_threshold', 1e-6)
        self.normal_search_radius = config.get('normal_search_radius', 0.1)
        self.normal_weight = config.get('normal_weight', 0.7)
        self.distance_weight = config.get('distance_weight', 0.3)
        self.angle_threshold = np.radians(config.get('angle_threshold', 30.0))
        
        # 内部状态
        self.source_pcd = None
        self.target_pcd = None
        self.source_normals = None
        self.target_normals = None
        self.transformation_history = []
        
        logger.info("法向量ICP算法初始化完成")
        
    def compute_normals(self, pcd: o3d.geometry.PointCloud, 
                       radius: Optional[float] = None) -> np.ndarray:
        """
        计算点云法向量
        
        Args:
            pcd: 输入点云
            radius: 邻域搜索半径
            
        Returns:
            法向量数组 (N, 3)
        """
        if radius is None:
            radius = self.normal_search_radius
            
        # 估计法向量
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamRadius(radius)
        )
        
        # 确保法向量方向一致性
        pcd.orient_normals_consistent_tangent_plane(100)
        
        normals = np.asarray(pcd.normals)
        logger.debug(f"计算了 {len(normals)} 个点的法向量")
        
        return normals
        
    def find_correspondences(self, source_points: np.ndarray, 
                           target_points: np.ndarray,
                           source_normals: np.ndarray,
                           target_normals: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        寻找对应点对，基于距离和法向量角度
        
        Args:
            source_points: 源点云点坐标 (N, 3)
            target_points: 目标点云点坐标 (M, 3)
            source_normals: 源点云法向量 (N, 3)
            target_normals: 目标点云法向量 (M, 3)
            
        Returns:
            (对应点对索引, 距离, 法向量角度)
        """
        # 使用KD树进行最近邻搜索
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_points)
        distances, indices = nbrs.kneighbors(source_points)
        
        indices = indices.flatten()
        distances = distances.flatten()
        
        # 计算法向量角度差异
        source_normals_norm = source_normals / (np.linalg.norm(source_normals, axis=1, keepdims=True) + 1e-8)
        target_normals_matched = target_normals[indices]
        target_normals_norm = target_normals_matched / (np.linalg.norm(target_normals_matched, axis=1, keepdims=True) + 1e-8)
        
        # 计算法向量夹角
        dot_products = np.sum(source_normals_norm * target_normals_norm, axis=1)
        dot_products = np.clip(dot_products, -1.0, 1.0)
        angles = np.arccos(np.abs(dot_products))  # 使用绝对值处理法向量方向不确定性
        
        # 过滤角度过大的对应点
        valid_mask = angles < self.angle_threshold
        
        valid_indices = np.arange(len(source_points))[valid_mask]
        valid_matches = indices[valid_mask]
        valid_distances = distances[valid_mask]
        valid_angles = angles[valid_mask]
        
        logger.debug(f"找到 {len(valid_indices)} 个有效对应点对")
        
        return valid_indices, valid_matches, valid_distances, valid_angles
        
    def compute_transformation(self, source_points: np.ndarray,
                             target_points: np.ndarray,
                             source_normals: np.ndarray,
                             target_normals: np.ndarray,
                             weights: np.ndarray) -> np.ndarray:
        """
        计算变换矩阵，使用加权最小二乘法
        
        Args:
            source_points: 源点坐标 (N, 3)
            target_points: 目标点坐标 (N, 3)  
            source_normals: 源点法向量 (N, 3)
            target_normals: 目标点法向量 (N, 3)
            weights: 权重 (N,)
            
        Returns:
            4x4变换矩阵
        """
        if len(source_points) < 3:
            logger.warning("对应点对数量不足，返回单位矩阵")
            return np.eye(4)
            
        # 计算加权质心
        weights_sum = np.sum(weights)
        if weights_sum < 1e-8:
            logger.warning("权重和过小，返回单位矩阵")
            return np.eye(4)
            
        source_centroid = np.sum(source_points * weights.reshape(-1, 1), axis=0) / weights_sum
        target_centroid = np.sum(target_points * weights.reshape(-1, 1), axis=0) / weights_sum
        
        # 去中心化
        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid
        
        # 构建协方差矩阵（加权）
        W = np.diag(weights)
        H = source_centered.T @ W @ target_centered
        
        # SVD分解
        try:
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # 确保旋转矩阵的行列式为正
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
                
            # 计算平移
            t = target_centroid - R @ source_centroid
            
            # 构建变换矩阵
            transformation = np.eye(4)
            transformation[:3, :3] = R
            transformation[:3, 3] = t
            
            return transformation
            
        except np.linalg.LinAlgError:
            logger.warning("SVD分解失败，返回单位矩阵")
            return np.eye(4)
            
    def compute_weights(self, distances: np.ndarray, angles: np.ndarray) -> np.ndarray:
        """
        计算对应点对的权重
        
        Args:
            distances: 点对距离 (N,)
            angles: 法向量角度差 (N,)
            
        Returns:
            权重数组 (N,)
        """
        # 距离权重（高斯核）
        distance_std = np.std(distances) + 1e-8
        distance_weights = np.exp(-0.5 * (distances / distance_std) ** 2)
        
        # 角度权重（余弦相似度）
        angle_weights = np.cos(angles)
        
        # 组合权重
        combined_weights = (self.distance_weight * distance_weights + 
                          self.normal_weight * angle_weights)
        
        return combined_weights
        
    def register(self, source: o3d.geometry.PointCloud,
                target: o3d.geometry.PointCloud,
                initial_transformation: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
        """
        执行法向量ICP配准
        
        Args:
            source: 源点云
            target: 目标点云  
            initial_transformation: 初始变换矩阵
            
        Returns:
            (最终变换矩阵, 配准信息)
        """
        logger.info("开始法向量ICP配准")
        
        # 保存输入点云
        self.source_pcd = o3d.geometry.PointCloud(source)
        self.target_pcd = o3d.geometry.PointCloud(target)
        
        # 计算法向量
        self.source_normals = self.compute_normals(self.source_pcd)
        self.target_normals = self.compute_normals(self.target_pcd)
        
        # 应用初始变换
        current_transformation = initial_transformation if initial_transformation is not None else np.eye(4)
        current_source = o3d.geometry.PointCloud(self.source_pcd)
        current_source.transform(current_transformation)
        
        self.transformation_history = [current_transformation.copy()]
        
        # 迭代配准
        for iteration in range(self.max_iterations):
            source_points = np.asarray(current_source.points)
            target_points = np.asarray(self.target_pcd.points)
            
            # 变换源点云法向量
            R = current_transformation[:3, :3]
            transformed_source_normals = (R @ self.source_normals.T).T
            
            # 寻找对应点对
            src_indices, tgt_indices, distances, angles = self.find_correspondences(
                source_points, target_points, 
                transformed_source_normals, self.target_normals
            )
            
            if len(src_indices) < 3:
                logger.warning(f"第 {iteration+1} 次迭代：对应点对不足")
                break
                
            # 提取对应点对
            src_corr = source_points[src_indices]
            tgt_corr = target_points[tgt_indices]
            src_normals_corr = transformed_source_normals[src_indices]
            tgt_normals_corr = self.target_normals[tgt_indices]
            
            # 计算权重
            weights = self.compute_weights(distances, angles)
            
            # 计算增量变换
            delta_transformation = self.compute_transformation(
                src_corr, tgt_corr, src_normals_corr, tgt_normals_corr, weights
            )
            
            # 更新变换
            new_transformation = delta_transformation @ current_transformation
            
            # 检查收敛性
            transformation_diff = np.linalg.norm(
                current_transformation - new_transformation
            )
            
            current_transformation = new_transformation
            current_source.transform(delta_transformation)
            self.transformation_history.append(current_transformation.copy())
            
            logger.debug(f"第 {iteration+1} 次迭代: 对应点对={len(src_indices)}, "
                        f"变换差异={transformation_diff:.6f}")
                        
            if transformation_diff < self.convergence_threshold:
                logger.info(f"第 {iteration+1} 次迭代收敛")
                break
        else:
            logger.info(f"达到最大迭代次数 {self.max_iterations}")
            
        # 计算最终配准信息
        final_source = o3d.geometry.PointCloud(self.source_pcd)
        final_source.transform(current_transformation)
        
        # 计算配准误差
        final_src_points = np.asarray(final_source.points)
        final_src_normals = (current_transformation[:3, :3] @ self.source_normals.T).T
        
        src_indices, tgt_indices, final_distances, final_angles = self.find_correspondences(
            final_src_points, target_points, final_src_normals, self.target_normals
        )
        
        registration_info = {
            'num_iterations': iteration + 1,
            'final_error': np.mean(final_distances) if len(final_distances) > 0 else float('inf'),
            'num_correspondences': len(src_indices),
            'transformation_history': self.transformation_history
        }
        
        logger.info(f"法向量ICP配准完成: 迭代={registration_info['num_iterations']}, "
                   f"误差={registration_info['final_error']:.6f}, "
                   f"对应点对={registration_info['num_correspondences']}")
                   
        return current_transformation, registration_info