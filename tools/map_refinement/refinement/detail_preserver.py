import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional, Union
import logging
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.distance import cdist
from scipy.ndimage import gaussian_filter1d
"""
细节保护过滤器

在点云处理过程中保护重要的细节特征，包括：
- 多尺度特征保持
- 边缘敏感平滑
- 局部几何特征保护
- 纹理细节增强
- 自适应特征保护
- 分层细节保护
"""


logger = logging.getLogger(__name__)

class DetailPreservationResult:
    """细节保护结果类"""
    
    def __init__(self, processed_pcd: o3d.geometry.PointCloud,
                 feature_map: np.ndarray,
                 preservation_map: np.ndarray,
                 method: str,
                 parameters: Dict):
        self.processed_pcd = processed_pcd
        self.feature_map = feature_map  # 特征重要性地图
        self.preservation_map = preservation_map  # 保护强度地图
        self.method = method
        self.parameters = parameters
        
    def __repr__(self):
        return f"DetailPreservation(method={self.method}, points={len(self.processed_pcd.points)})"

class DetailPreserver:
    """细节保护过滤器类"""
    
    def __init__(self, config: dict):
        """
        初始化细节保护过滤器
        
        Args:
            config: 算法配置字典
        """
        self.multi_scale_levels = config.get('multi_scale_levels', 3)
        self.edge_sensitivity = config.get('edge_sensitivity', 0.8)
        self.texture_enhancement = config.get('texture_enhancement', True)
        
        # 特征检测参数
        self.feature_radius = 0.05
        self.curvature_threshold = 0.1
        self.normal_variation_threshold = 0.3
        
        logger.info("细节保护过滤器初始化完成")
        
    def compute_feature_importance(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """
        计算每个点的特征重要性
        
        Args:
            pcd: 输入点云
            
        Returns:
            特征重要性数组 (N,)，值越大表示越重要
        """
        logger.info("开始计算特征重要性")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return np.array([])
            
        # 确保有法向量
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(self.feature_radius)
            )
            
        normals = np.asarray(pcd.normals)
        
        # 1. 曲率特征
        curvatures = self._compute_curvature(pcd)
        curvature_importance = self._normalize_feature(curvatures)
        
        # 2. 法向量变化特征
        normal_variations = self._compute_normal_variation(pcd)
        normal_importance = self._normalize_feature(normal_variations)
        
        # 3. 边缘特征
        edge_features = self._compute_edge_features(pcd)
        edge_importance = self._normalize_feature(edge_features)
        
        # 4. 局部密度变化特征
        density_variations = self._compute_density_variation(pcd)
        density_importance = self._normalize_feature(density_variations)
        
        # 5. 几何显著性特征
        geometric_saliency = self._compute_geometric_saliency(pcd)
        saliency_importance = self._normalize_feature(geometric_saliency)
        
        # 综合特征重要性
        importance_weights = [0.25, 0.2, 0.3, 0.15, 0.1]  # 对应上述5种特征
        
        feature_importance = (
            importance_weights[0] * curvature_importance +
            importance_weights[1] * normal_importance +
            importance_weights[2] * edge_importance +
            importance_weights[3] * density_importance +
            importance_weights[4] * saliency_importance
        )
        
        logger.info("特征重要性计算完成")
        
        return feature_importance
        
    def _compute_curvature(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """计算曲率"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)
        curvatures = np.zeros(n_points)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            
            if k < 4:
                continue
                
            neighbor_normals = normals[idx[1:]]
            center_normal = normals[i]
            
            # 计算法向量变化率
            normal_differences = neighbor_normals - center_normal
            normal_variations = np.linalg.norm(normal_differences, axis=1)
            curvatures[i] = np.mean(normal_variations)
            
        return curvatures
        
    def _compute_normal_variation(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """计算法向量变化"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)
        variations = np.zeros(n_points)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            
            if k < 3:
                continue
                
            neighbor_normals = normals[idx]
            
            # 计算法向量的标准差
            variations[i] = np.std(neighbor_normals.flatten())
            
        return variations
        
    def _compute_edge_features(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """计算边缘特征"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)
        edge_features = np.zeros(n_points)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            
            if k < 6:
                continue
                
            neighbor_points = points[idx[1:]]
            center_point = points[i]
            
            # 计算协方差矩阵的特征值
            centered_points = neighbor_points - center_point
            cov_matrix = np.cov(centered_points.T)
            eigenvalues = np.linalg.eigvals(cov_matrix)
            eigenvalues = np.sort(eigenvalues)[::-1]
            
            # 边缘特征：最大特征值与其他特征值的比值
            if len(eigenvalues) >= 2 and eigenvalues[1] > 1e-8:
                edge_features[i] = eigenvalues[0] / eigenvalues[1]
            else:
                edge_features[i] = 1.0
                
        return edge_features
        
    def _compute_density_variation(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """计算密度变化"""
        points = np.asarray(pcd.points)
        n_points = len(points)
        density_variations = np.zeros(n_points)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        # 计算每个点的局部密度
        local_densities = np.zeros(n_points)
        for i in range(n_points):
            [k, _, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            local_densities[i] = k
            
        # 计算密度变化
        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            
            if k < 3:
                continue
                
            neighbor_densities = local_densities[idx]
            density_variations[i] = np.std(neighbor_densities)
            
        return density_variations
        
    def _compute_geometric_saliency(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """计算几何显著性"""
        points = np.asarray(pcd.points)
        n_points = len(points)
        saliency = np.zeros(n_points)
        
        # 多尺度显著性计算
        scales = [self.feature_radius * scale for scale in [0.5, 1.0, 2.0]]
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        for i in range(n_points):
            scale_saliency = []
            
            for scale in scales:
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], scale)
                
                if k < 3:
                    scale_saliency.append(0.0)
                    continue
                    
                neighbor_points = points[idx[1:]]
                center_point = points[i]
                
                # 计算到质心的距离作为显著性度量
                centroid = np.mean(neighbor_points, axis=0)
                distance_to_centroid = np.linalg.norm(center_point - centroid)
                scale_saliency.append(distance_to_centroid)
                
            # 多尺度融合
            saliency[i] = np.mean(scale_saliency)
            
        return saliency
        
    def _normalize_feature(self, feature: np.ndarray) -> np.ndarray:
        """特征归一化"""
        if len(feature) == 0:
            return feature
            
        min_val = np.min(feature)
        max_val = np.max(feature)
        
        if max_val - min_val < 1e-8:
            return np.zeros_like(feature)
            
        return (feature - min_val) / (max_val - min_val)
        
    def multi_scale_detail_preservation(self, pcd: o3d.geometry.PointCloud) -> DetailPreservationResult:
        """
        多尺度细节保护
        
        Args:
            pcd: 输入点云
            
        Returns:
            细节保护结果
        """
        logger.info("开始多尺度细节保护")
        
        # 计算特征重要性
        feature_importance = self.compute_feature_importance(pcd)
        
        if len(feature_importance) == 0:
            return DetailPreservationResult(
                pcd, np.array([]), np.array([]), 
                "multi_scale", {}
            )
            
        # 多尺度处理
        scales = [0.01, 0.02, 0.05]  # 不同的处理尺度
        processed_points = []
        
        points = np.asarray(pcd.points)
        original_points = points.copy()
        
        for level, scale in enumerate(scales):
            logger.debug(f"处理尺度级别 {level+1}/{len(scales)}: {scale}m")
            
            # 基于特征重要性的自适应平滑
            smoothed_points = self._adaptive_smooth(
                original_points, feature_importance, scale
            )
            
            processed_points.append(smoothed_points)
            
        # 多尺度融合
        final_points = self._fuse_multi_scale_results(
            original_points, processed_points, feature_importance
        )
        
        # 创建结果点云
        result_pcd = o3d.geometry.PointCloud(pcd)
        result_pcd.points = o3d.utility.Vector3dVector(final_points)
        
        # 保护强度地图
        preservation_map = 1.0 - feature_importance  # 重要特征保护强度高
        
        parameters = {
            'scales': scales,
            'levels': len(scales)
        }
        
        result = DetailPreservationResult(
            result_pcd, feature_importance, preservation_map,
            "multi_scale", parameters
        )
        
        logger.info("多尺度细节保护完成")
        
        return result
        
    def _adaptive_smooth(self, points: np.ndarray, 
                        feature_importance: np.ndarray,
                        scale: float) -> np.ndarray:
        """自适应平滑"""
        n_points = len(points)
        smoothed_points = points.copy()
        
        # 构建KD树
        nbrs = NearestNeighbors(radius=scale, algorithm='kd_tree').fit(points)
        
        for i in range(n_points):
            # 搜索邻域
            distances, indices = nbrs.radius_neighbors([points[i]])
            neighbor_indices = indices[0]
            
            if len(neighbor_indices) < 2:
                continue
                
            neighbor_points = points[neighbor_indices]
            neighbor_importance = feature_importance[neighbor_indices]
            
            # 基于重要性的加权平均
            # 重要特征附近的点受到较少的平滑
            current_importance = feature_importance[i]
            smooth_factor = 1.0 - current_importance * self.edge_sensitivity
            
            if smooth_factor > 0.1:  # 只对非关键特征进行平滑
                # 距离权重
                neighbor_distances = distances[0]
                distance_weights = np.exp(-neighbor_distances / scale)
                
                # 重要性权重
                importance_weights = 1.0 / (1.0 + neighbor_importance)
                
                # 综合权重
                combined_weights = distance_weights * importance_weights
                combined_weights /= np.sum(combined_weights)
                
                # 加权平均
                smoothed_position = np.average(neighbor_points, axis=0, weights=combined_weights)
                
                # 插值到原始位置
                smoothed_points[i] = (1.0 - smooth_factor) * points[i] + smooth_factor * smoothed_position
                
        return smoothed_points
        
    def _fuse_multi_scale_results(self, original_points: np.ndarray,
                                 processed_points_list: List[np.ndarray],
                                 feature_importance: np.ndarray) -> np.ndarray:
        """多尺度结果融合"""
        n_points = len(original_points)
        final_points = np.zeros_like(original_points)
        
        # 为每个尺度分配权重
        scale_weights = [0.5, 0.3, 0.2]  # 细尺度权重更高
        
        for i in range(n_points):
            # 基于特征重要性调整尺度权重
            importance = feature_importance[i]
            
            # 重要特征更依赖原始数据和细尺度处理
            adjusted_weights = np.array(scale_weights)
            if importance > 0.5:  # 高重要性特征
                adjusted_weights[0] *= 2.0  # 增加细尺度权重
                adjusted_weights = adjusted_weights / np.sum(adjusted_weights)
                
            # 加权融合
            weighted_sum = original_points[i] * (1.0 - importance * 0.5)
            
            for j, processed_points in enumerate(processed_points_list):
                weighted_sum += processed_points[i] * adjusted_weights[j] * importance * 0.5
                
            final_points[i] = weighted_sum
            
        return final_points
        
    def edge_sensitive_filtering(self, pcd: o3d.geometry.PointCloud) -> DetailPreservationResult:
        """
        边缘敏感过滤
        
        Args:
            pcd: 输入点云
            
        Returns:
            细节保护结果
        """
        logger.info("开始边缘敏感过滤")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return DetailPreservationResult(
                pcd, np.array([]), np.array([]), 
                "edge_sensitive", {}
            )
            
        # 检测边缘
        edge_map = self._detect_edges(pcd)
        
        # 基于边缘的自适应过滤
        filtered_points = self._edge_aware_filter(points, edge_map)
        
        # 创建结果点云
        result_pcd = o3d.geometry.PointCloud(pcd)
        result_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        
        # 保护强度地图
        preservation_map = edge_map
        
        parameters = {
            'edge_sensitivity': self.edge_sensitivity
        }
        
        result = DetailPreservationResult(
            result_pcd, edge_map, preservation_map,
            "edge_sensitive", parameters
        )
        
        logger.info("边缘敏感过滤完成")
        
        return result
        
    def _detect_edges(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """检测边缘"""
        points = np.asarray(pcd.points)
        n_points = len(points)
        edge_map = np.zeros(n_points)
        
        if not pcd.has_normals():
            pcd.estimate_normals()
            
        normals = np.asarray(pcd.normals)
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.feature_radius)
            
            if k < 4:
                continue
                
            neighbor_normals = normals[idx[1:]]
            center_normal = normals[i]
            
            # 计算法向量角度变化
            dot_products = np.dot(neighbor_normals, center_normal)
            dot_products = np.clip(dot_products, -1, 1)
            angles = np.arccos(np.abs(dot_products))
            
            # 边缘强度
            max_angle = np.max(angles)
            edge_map[i] = max_angle / np.pi  # 归一化到[0,1]
            
        return edge_map
        
    def _edge_aware_filter(self, points: np.ndarray, edge_map: np.ndarray) -> np.ndarray:
        """边缘感知过滤"""
        n_points = len(points)
        filtered_points = points.copy()
        
        # 构建KD树
        nbrs = NearestNeighbors(radius=self.feature_radius, algorithm='kd_tree').fit(points)
        
        for i in range(n_points):
            edge_strength = edge_map[i]
            
            # 边缘点保护
            if edge_strength > self.curvature_threshold:
                continue  # 不处理边缘点
                
            # 搜索邻域
            distances, indices = nbrs.radius_neighbors([points[i]])
            neighbor_indices = indices[0]
            
            if len(neighbor_indices) < 2:
                continue
                
            neighbor_points = points[neighbor_indices]
            neighbor_distances = distances[0]
            
            # 距离权重
            weights = np.exp(-neighbor_distances / self.feature_radius)
            weights /= np.sum(weights)
            
            # 基于边缘强度的平滑
            smooth_factor = 1.0 - edge_strength * self.edge_sensitivity
            smoothed_position = np.average(neighbor_points, axis=0, weights=weights)
            
            filtered_points[i] = (1.0 - smooth_factor) * points[i] + smooth_factor * smoothed_position
            
        return filtered_points
        
    def preserve_details(self, pcd: o3d.geometry.PointCloud,
                        method: str = "multi_scale") -> DetailPreservationResult:
        """
        执行细节保护
        
        Args:
            pcd: 输入点云
            method: 保护方法，可选值：'multi_scale', 'edge_sensitive'
            
        Returns:
            细节保护结果
        """
        logger.info(f"开始细节保护，方法: {method}")
        
        if len(pcd.points) < 3:
            logger.warning("点云点数不足，跳过细节保护")
            return DetailPreservationResult(
                pcd, np.array([]), np.array([]), 
                method, {}
            )
            
        if method == "multi_scale":
            return self.multi_scale_detail_preservation(pcd)
        elif method == "edge_sensitive":
            return self.edge_sensitive_filtering(pcd)
        else:
            logger.error(f"未知的细节保护方法: {method}")
            return self.multi_scale_detail_preservation(pcd)  # 默认方法