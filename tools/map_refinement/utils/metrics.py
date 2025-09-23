import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional, Union
import logging
import json
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.stats import entropy
from sklearn.neighbors import NearestNeighbors
"""
质量评估和对比分析功能

提供点云地图质量评估的各种指标：
- 几何质量指标
- 密度分布分析
- 特征保持度评估
- 配准精度评估
- 综合质量评分
- 对比分析报告
"""


logger = logging.getLogger(__name__)

class QualityMetrics:
    """点云质量评估指标类"""
    
    def __init__(self):
        """初始化质量评估器"""
        self.metrics_cache = {}
        logger.info("质量评估器初始化完成")
        
    def compute_density_metrics(self, pcd: o3d.geometry.PointCloud, 
                              radius: float = 0.05) -> Dict:
        """
        计算密度相关指标
        
        Args:
            pcd: 输入点云
            radius: 密度计算半径
            
        Returns:
            密度指标字典
        """
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return {'error': 'empty_point_cloud'}
            
        logger.debug("计算密度分布指标")
        
        # 计算局部密度
        densities = self._compute_local_densities(points, radius)
        
        # 计算密度统计
        density_stats = {
            'mean_density': float(np.mean(densities)),
            'std_density': float(np.std(densities)),
            'min_density': float(np.min(densities)),
            'max_density': float(np.max(densities)),
            'median_density': float(np.median(densities)),
            'density_range': float(np.max(densities) - np.min(densities)),
            'density_cv': float(np.std(densities) / np.mean(densities)) if np.mean(densities) > 0 else 0
        }
        
        # 密度分布均匀性
        density_histogram, _ = np.histogram(densities, bins=50)
        density_entropy = entropy(density_histogram + 1e-8)  # 避免log(0)
        max_entropy = np.log(len(density_histogram))
        uniformity_score = density_entropy / max_entropy if max_entropy > 0 else 0
        
        # 空洞检测
        low_density_threshold = np.percentile(densities, 10)
        hole_points = np.sum(densities < low_density_threshold)
        hole_ratio = hole_points / n_points
        
        density_metrics = {
            'statistics': density_stats,
            'uniformity_score': float(uniformity_score),
            'hole_ratio': float(hole_ratio),
            'density_distribution': densities.tolist()
        }
        
        return density_metrics
        
    def compute_geometric_metrics(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """
        计算几何质量指标
        
        Args:
            pcd: 输入点云
            
        Returns:
            几何指标字典
        """
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return {'error': 'empty_point_cloud'}
            
        logger.debug("计算几何质量指标")
        
        # 确保有法向量
        if not pcd.has_normals():
            pcd.estimate_normals()
            
        normals = np.asarray(pcd.normals)
        
        # 1. 法向量一致性
        normal_consistency = self._compute_normal_consistency(normals)
        
        # 2. 表面平滑度
        surface_smoothness = self._compute_surface_smoothness(pcd)
        
        # 3. 曲率分布
        curvature_metrics = self._compute_curvature_metrics(pcd)
        
        # 4. 几何复杂度
        geometric_complexity = self._compute_geometric_complexity(points)
        
        # 5. 边缘质量
        edge_metrics = self._compute_edge_metrics(pcd)
        
        geometric_metrics = {
            'normal_consistency': normal_consistency,
            'surface_smoothness': surface_smoothness,
            'curvature_metrics': curvature_metrics,
            'geometric_complexity': geometric_complexity,
            'edge_metrics': edge_metrics
        }
        
        return geometric_metrics
        
    def compute_feature_preservation_metrics(self, original_pcd: o3d.geometry.PointCloud,
                                           processed_pcd: o3d.geometry.PointCloud) -> Dict:
        """
        计算特征保持度指标
        
        Args:
            original_pcd: 原始点云
            processed_pcd: 处理后点云
            
        Returns:
            特征保持度指标字典
        """
        logger.debug("计算特征保持度指标")
        
        # 确保两个点云都有法向量
        for pcd in [original_pcd, processed_pcd]:
            if not pcd.has_normals():
                pcd.estimate_normals()
                
        # 1. 特征点保持度
        feature_preservation = self._compute_feature_point_preservation(
            original_pcd, processed_pcd
        )
        
        # 2. 几何结构保持度
        structure_preservation = self._compute_structure_preservation(
            original_pcd, processed_pcd
        )
        
        # 3. 细节保持度
        detail_preservation = self._compute_detail_preservation(
            original_pcd, processed_pcd
        )
        
        # 4. 空间分布保持度
        spatial_preservation = self._compute_spatial_preservation(
            original_pcd, processed_pcd
        )
        
        preservation_metrics = {
            'feature_preservation': feature_preservation,
            'structure_preservation': structure_preservation,
            'detail_preservation': detail_preservation,
            'spatial_preservation': spatial_preservation
        }
        
        return preservation_metrics
        
    def compute_registration_quality(self, source_pcd: o3d.geometry.PointCloud,
                                   target_pcd: o3d.geometry.PointCloud,
                                   transformation: np.ndarray) -> Dict:
        """
        计算配准质量指标
        
        Args:
            source_pcd: 源点云
            target_pcd: 目标点云
            transformation: 变换矩阵
            
        Returns:
            配准质量指标字典
        """
        logger.debug("计算配准质量指标")
        
        # 应用变换
        source_transformed = o3d.geometry.PointCloud(source_pcd)
        source_transformed.transform(transformation)
        
        source_points = np.asarray(source_transformed.points)
        target_points = np.asarray(target_pcd.points)
        
        # 1. 点对距离误差
        distance_errors = self._compute_point_to_point_errors(source_points, target_points)
        
        # 2. 法向量对齐误差
        normal_errors = self._compute_normal_alignment_errors(source_transformed, target_pcd)
        
        # 3. 重叠度分析
        overlap_analysis = self._compute_overlap_analysis(source_points, target_points)
        
        # 4. 配准收敛性
        convergence_metrics = self._analyze_transformation_quality(transformation)
        
        registration_metrics = {
            'distance_errors': distance_errors,
            'normal_errors': normal_errors,
            'overlap_analysis': overlap_analysis,
            'convergence_metrics': convergence_metrics
        }
        
        return registration_metrics
        
    def compute_comprehensive_score(self, metrics: Dict) -> float:
        """
        计算综合质量评分
        
        Args:
            metrics: 各项指标字典
            
        Returns:
            综合评分 (0-100)
        """
        logger.debug("计算综合质量评分")
        
        scores = []
        weights = []
        
        # 密度质量评分 (权重: 20%)
        if 'density' in metrics:
            density_score = self._score_density_quality(metrics['density'])
            scores.append(density_score)
            weights.append(0.2)
            
        # 几何质量评分 (权重: 30%)
        if 'geometric' in metrics:
            geometric_score = self._score_geometric_quality(metrics['geometric'])
            scores.append(geometric_score)
            weights.append(0.3)
            
        # 特征保持评分 (权重: 25%)
        if 'feature_preservation' in metrics:
            preservation_score = self._score_preservation_quality(metrics['feature_preservation'])
            scores.append(preservation_score)
            weights.append(0.25)
            
        # 配准质量评分 (权重: 25%)
        if 'registration' in metrics:
            registration_score = self._score_registration_quality(metrics['registration'])
            scores.append(registration_score)
            weights.append(0.25)
            
        if not scores:
            return 0.0
            
        # 加权平均
        weights = np.array(weights)
        weights = weights / np.sum(weights)  # 归一化
        
        comprehensive_score = np.average(scores, weights=weights)
        
        return float(comprehensive_score)
        
    def generate_comparison_report(self, original_metrics: Dict, 
                                 refined_metrics: Dict,
                                 output_path: str = None) -> str:
        """
        生成对比分析报告
        
        Args:
            original_metrics: 原始点云指标
            refined_metrics: 精细化后点云指标
            output_path: 报告保存路径
            
        Returns:
            报告内容
        """
        logger.info("生成对比分析报告")
        
        # 计算改进指标
        improvements = self._compute_improvements(original_metrics, refined_metrics)
        
        # 生成报告内容
        report = {
            'timestamp': datetime.now().isoformat(),
            'comparison_summary': {
                'original_score': self.compute_comprehensive_score(original_metrics),
                'refined_score': self.compute_comprehensive_score(refined_metrics),
                'overall_improvement': improvements.get('overall_improvement', 0)
            },
            'detailed_improvements': improvements,
            'original_metrics': original_metrics,
            'refined_metrics': refined_metrics,
            'recommendations': self._generate_recommendations(improvements)
        }
        
        # 保存报告
        if output_path:
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False, default=str)
            logger.info(f"对比报告保存到: {output_path}")
            
        return json.dumps(report, indent=2, ensure_ascii=False, default=str)
        
    # ==================== 私有方法 ====================
    
    def _compute_local_densities(self, points: np.ndarray, radius: float) -> np.ndarray:
        """计算局部密度"""
        n_points = len(points)
        densities = np.zeros(n_points)
        
        nbrs = NearestNeighbors(radius=radius, algorithm='kd_tree').fit(points)
        
        for i in range(n_points):
            distances, indices = nbrs.radius_neighbors([points[i]])
            neighbor_count = len(indices[0]) - 1  # 排除自身
            volume = (4/3) * np.pi * (radius ** 3)
            densities[i] = neighbor_count / volume
            
        return densities
        
    def _compute_normal_consistency(self, normals: np.ndarray) -> Dict:
        """计算法向量一致性"""
        # 计算相邻法向量的角度差异
        dot_products = np.abs(np.sum(normals * normals, axis=1))
        angles = np.arccos(np.clip(dot_products, -1, 1))
        
        consistency_metrics = {
            'mean_angle_deviation': float(np.mean(angles)),
            'std_angle_deviation': float(np.std(angles)),
            'consistency_score': float(1.0 - np.mean(angles) / np.pi)
        }
        
        return consistency_metrics
        
    def _compute_surface_smoothness(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """计算表面平滑度"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        # 构建KD树
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        smoothness_values = []
        
        for i in range(len(points)):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], 0.05)
            
            if k < 3:
                continue
                
            neighbor_normals = normals[idx[1:]]  # 排除自身
            center_normal = normals[i]
            
            # 计算法向量变化率
            normal_variations = np.linalg.norm(neighbor_normals - center_normal, axis=1)
            smoothness_values.append(np.mean(normal_variations))
            
        smoothness_metrics = {
            'mean_smoothness': float(np.mean(smoothness_values)),
            'std_smoothness': float(np.std(smoothness_values)),
            'smoothness_score': float(1.0 / (1.0 + np.mean(smoothness_values)))
        }
        
        return smoothness_metrics
        
    def _compute_curvature_metrics(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """计算曲率分布指标"""
        # 简化的曲率计算
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        curvatures = []
        
        for i in range(len(points)):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], 0.05)
            
            if k < 4:
                continue
                
            neighbor_normals = normals[idx[1:]]
            center_normal = normals[i]
            
            # 计算曲率近似值
            normal_differences = neighbor_normals - center_normal
            curvature = np.mean(np.linalg.norm(normal_differences, axis=1))
            curvatures.append(curvature)
            
        curvature_metrics = {
            'mean_curvature': float(np.mean(curvatures)),
            'std_curvature': float(np.std(curvatures)),
            'max_curvature': float(np.max(curvatures)),
            'curvature_distribution': curvatures
        }
        
        return curvature_metrics
        
    def _compute_geometric_complexity(self, points: np.ndarray) -> Dict:
        """计算几何复杂度"""
        # 使用主成分分析评估几何复杂度
        if len(points) < 3:
            return {'complexity_score': 0.0}
            
        # 计算协方差矩阵
        centered_points = points - np.mean(points, axis=0)
        cov_matrix = np.cov(centered_points.T)
        
        # 计算特征值
        eigenvalues = np.linalg.eigvals(cov_matrix)
        eigenvalues = np.sort(eigenvalues)[::-1]  # 降序
        
        # 计算几何复杂度指标
        linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0] if eigenvalues[0] > 0 else 0
        planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0] if eigenvalues[0] > 0 else 0
        sphericity = eigenvalues[2] / eigenvalues[0] if eigenvalues[0] > 0 else 0
        
        complexity_metrics = {
            'linearity': float(linearity),
            'planarity': float(planarity),
            'sphericity': float(sphericity),
            'complexity_score': float(1.0 - linearity)  # 复杂度与线性度相反
        }
        
        return complexity_metrics
        
    def _compute_edge_metrics(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """计算边缘质量指标"""
        # 简化的边缘检测
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        edge_strengths = []
        
        for i in range(len(points)):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], 0.05)
            
            if k < 4:
                continue
                
            neighbor_normals = normals[idx[1:]]
            center_normal = normals[i]
            
            # 计算法向量角度变化
            dot_products = np.dot(neighbor_normals, center_normal)
            angles = np.arccos(np.clip(np.abs(dot_products), 0, 1))
            edge_strength = np.max(angles)
            edge_strengths.append(edge_strength)
            
        edge_metrics = {
            'mean_edge_strength': float(np.mean(edge_strengths)),
            'std_edge_strength': float(np.std(edge_strengths)),
            'edge_count': int(np.sum(np.array(edge_strengths) > 0.3)),  # 阈值
            'edge_sharpness': float(np.mean(edge_strengths))
        }
        
        return edge_metrics
        
    def _compute_feature_point_preservation(self, original_pcd: o3d.geometry.PointCloud,
                                          processed_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算特征点保持度"""
        # 简化实现：比较高曲率点的保持情况
        
        def get_feature_points(pcd, threshold=0.1):
            points = np.asarray(pcd.points)
            normals = np.asarray(pcd.normals)
            kdtree = o3d.geometry.KDTreeFlann(pcd)
            
            feature_indices = []
            for i in range(len(points)):
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], 0.05)
                if k < 4:
                    continue
                    
                neighbor_normals = normals[idx[1:]]
                center_normal = normals[i]
                curvature = np.mean(np.linalg.norm(neighbor_normals - center_normal, axis=1))
                
                if curvature > threshold:
                    feature_indices.append(i)
                    
            return feature_indices, points[feature_indices] if feature_indices else np.array([])
        
        original_feature_indices, original_features = get_feature_points(original_pcd)
        processed_feature_indices, processed_features = get_feature_points(processed_pcd)
        
        if len(original_features) == 0:
            return {'preservation_ratio': 1.0, 'feature_loss': 0.0}
            
        # 计算特征点保持率
        if len(processed_features) == 0:
            return {'preservation_ratio': 0.0, 'feature_loss': 1.0}
            
        # 使用最近邻匹配计算保持率
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(processed_features)
        distances, _ = nbrs.kneighbors(original_features)
        
        preserved_count = np.sum(distances.flatten() < 0.02)  # 2cm阈值
        preservation_ratio = preserved_count / len(original_features)
        
        preservation_metrics = {
            'preservation_ratio': float(preservation_ratio),
            'feature_loss': float(1.0 - preservation_ratio),
            'original_feature_count': len(original_features),
            'processed_feature_count': len(processed_features)
        }
        
        return preservation_metrics
        
    def _compute_structure_preservation(self, original_pcd: o3d.geometry.PointCloud,
                                      processed_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算结构保持度"""
        # 比较几何结构的保持情况
        original_points = np.asarray(original_pcd.points)
        processed_points = np.asarray(processed_pcd.points)
        
        # 计算包围盒变化
        original_bbox = original_pcd.get_axis_aligned_bounding_box()
        processed_bbox = processed_pcd.get_axis_aligned_bounding_box()
        
        original_extent = original_bbox.get_extent()
        processed_extent = processed_bbox.get_extent()
        
        extent_change = np.linalg.norm(processed_extent - original_extent) / np.linalg.norm(original_extent)
        
        # 计算点数变化
        point_count_change = abs(len(processed_points) - len(original_points)) / len(original_points)
        
        structure_metrics = {
            'extent_preservation': float(1.0 - extent_change),
            'point_count_stability': float(1.0 - point_count_change),
            'structure_score': float((1.0 - extent_change + 1.0 - point_count_change) / 2)
        }
        
        return structure_metrics
        
    def _compute_detail_preservation(self, original_pcd: o3d.geometry.PointCloud,
                                   processed_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算细节保持度"""
        # 比较高频几何细节的保持情况
        
        def compute_local_variation(pcd):
            points = np.asarray(pcd.points)
            kdtree = o3d.geometry.KDTreeFlann(pcd)
            variations = []
            
            for i in range(len(points)):
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], 0.02)  # 小半径
                if k < 3:
                    continue
                    
                neighbor_points = points[idx]
                centroid = np.mean(neighbor_points, axis=0)
                distances = np.linalg.norm(neighbor_points - centroid, axis=1)
                variations.append(np.std(distances))
                
            return np.array(variations)
        
        original_variations = compute_local_variation(original_pcd)
        processed_variations = compute_local_variation(processed_pcd)
        
        if len(original_variations) == 0 or len(processed_variations) == 0:
            return {'detail_preservation': 0.5}
            
        # 比较变化分布
        original_mean_var = np.mean(original_variations)
        processed_mean_var = np.mean(processed_variations)
        
        detail_preservation = 1.0 - abs(processed_mean_var - original_mean_var) / (original_mean_var + 1e-8)
        detail_preservation = max(0, min(1, detail_preservation))
        
        detail_metrics = {
            'detail_preservation': float(detail_preservation),
            'original_detail_level': float(original_mean_var),
            'processed_detail_level': float(processed_mean_var)
        }
        
        return detail_metrics
        
    def _compute_spatial_preservation(self, original_pcd: o3d.geometry.PointCloud,
                                    processed_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算空间分布保持度"""
        original_points = np.asarray(original_pcd.points)
        processed_points = np.asarray(processed_pcd.points)
        
        # 计算空间分布统计
        def compute_spatial_stats(points):
            return {
                'centroid': np.mean(points, axis=0),
                'std': np.std(points, axis=0),
                'extent': np.max(points, axis=0) - np.min(points, axis=0)
            }
        
        original_stats = compute_spatial_stats(original_points)
        processed_stats = compute_spatial_stats(processed_points)
        
        # 计算保持度
        centroid_change = np.linalg.norm(processed_stats['centroid'] - original_stats['centroid'])
        std_change = np.linalg.norm(processed_stats['std'] - original_stats['std'])
        extent_change = np.linalg.norm(processed_stats['extent'] - original_stats['extent'])
        
        # 归一化
        centroid_change /= (np.linalg.norm(original_stats['centroid']) + 1e-8)
        std_change /= (np.linalg.norm(original_stats['std']) + 1e-8)
        extent_change /= (np.linalg.norm(original_stats['extent']) + 1e-8)
        
        spatial_preservation = 1.0 - (centroid_change + std_change + extent_change) / 3
        spatial_preservation = max(0, min(1, spatial_preservation))
        
        spatial_metrics = {
            'spatial_preservation': float(spatial_preservation),
            'centroid_stability': float(1.0 - centroid_change),
            'distribution_stability': float(1.0 - std_change),
            'extent_stability': float(1.0 - extent_change)
        }
        
        return spatial_metrics
        
    def _compute_point_to_point_errors(self, source_points: np.ndarray,
                                     target_points: np.ndarray) -> Dict:
        """计算点对点距离误差"""
        if len(source_points) == 0 or len(target_points) == 0:
            return {'error': 'empty_points'}
            
        # 使用最近邻计算距离
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_points)
        distances, _ = nbrs.kneighbors(source_points)
        distances = distances.flatten()
        
        error_metrics = {
            'mean_distance': float(np.mean(distances)),
            'std_distance': float(np.std(distances)),
            'max_distance': float(np.max(distances)),
            'rmse': float(np.sqrt(np.mean(distances ** 2))),
            'inlier_ratio': float(np.sum(distances < 0.05) / len(distances))  # 5cm阈值
        }
        
        return error_metrics
        
    def _compute_normal_alignment_errors(self, source_pcd: o3d.geometry.PointCloud,
                                       target_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算法向量对齐误差"""
        if not source_pcd.has_normals() or not target_pcd.has_normals():
            return {'error': 'missing_normals'}
            
        source_points = np.asarray(source_pcd.points)
        source_normals = np.asarray(source_pcd.normals)
        target_points = np.asarray(target_pcd.points)
        target_normals = np.asarray(target_pcd.normals)
        
        # 为每个源点找到最近的目标点
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_points)
        _, indices = nbrs.kneighbors(source_points)
        indices = indices.flatten()
        
        # 计算法向量夹角
        matched_target_normals = target_normals[indices]
        dot_products = np.sum(source_normals * matched_target_normals, axis=1)
        dot_products = np.clip(dot_products, -1, 1)
        angles = np.arccos(np.abs(dot_products))  # 使用绝对值处理方向不确定性
        
        normal_error_metrics = {
            'mean_angle_error': float(np.mean(angles)),
            'std_angle_error': float(np.std(angles)),
            'max_angle_error': float(np.max(angles)),
            'normal_consistency': float(np.mean(np.abs(dot_products)))
        }
        
        return normal_error_metrics
        
    def _compute_overlap_analysis(self, source_points: np.ndarray,
                                target_points: np.ndarray) -> Dict:
        """计算重叠度分析"""
        if len(source_points) == 0 or len(target_points) == 0:
            return {'overlap_ratio': 0.0}
            
        # 计算重叠区域
        overlap_threshold = 0.05  # 5cm
        
        nbrs_target = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_points)
        distances_to_target, _ = nbrs_target.kneighbors(source_points)
        
        nbrs_source = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(source_points)
        distances_to_source, _ = nbrs_source.kneighbors(target_points)
        
        source_overlap = np.sum(distances_to_target.flatten() < overlap_threshold)
        target_overlap = np.sum(distances_to_source.flatten() < overlap_threshold)
        
        overlap_metrics = {
            'source_overlap_ratio': float(source_overlap / len(source_points)),
            'target_overlap_ratio': float(target_overlap / len(target_points)),
            'overall_overlap_ratio': float((source_overlap + target_overlap) / (len(source_points) + len(target_points))),
            'overlap_symmetry': float(min(source_overlap, target_overlap) / max(source_overlap, target_overlap)) if max(source_overlap, target_overlap) > 0 else 0
        }
        
        return overlap_metrics
        
    def _analyze_transformation_quality(self, transformation: np.ndarray) -> Dict:
        """分析变换质量"""
        # 提取旋转和平移
        rotation = transformation[:3, :3]
        translation = transformation[:3, 3]
        
        # 检查旋转矩阵的正交性
        orthogonality_error = np.linalg.norm(
            np.dot(rotation, rotation.T) - np.eye(3)
        )
        
        # 检查行列式（应该接近1）
        det_error = abs(np.linalg.det(rotation) - 1.0)
        
        # 计算旋转角度
        trace = np.trace(rotation)
        rotation_angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        
        # 计算平移距离
        translation_distance = np.linalg.norm(translation)
        
        transformation_metrics = {
            'orthogonality_error': float(orthogonality_error),
            'determinant_error': float(det_error),
            'rotation_angle_deg': float(np.degrees(rotation_angle)),
            'translation_distance': float(translation_distance),
            'transformation_quality': float(1.0 / (1.0 + orthogonality_error + det_error))
        }
        
        return transformation_metrics
        
    def _score_density_quality(self, density_metrics: Dict) -> float:
        """密度质量评分"""
        uniformity_score = density_metrics.get('uniformity_score', 0) * 50
        hole_penalty = density_metrics.get('hole_ratio', 0) * 30
        cv_penalty = min(density_metrics.get('statistics', {}).get('density_cv', 1), 1) * 20
        
        score = max(0, uniformity_score - hole_penalty - cv_penalty)
        return min(100, score)
        
    def _score_geometric_quality(self, geometric_metrics: Dict) -> float:
        """几何质量评分"""
        normal_score = geometric_metrics.get('normal_consistency', {}).get('consistency_score', 0) * 30
        smoothness_score = geometric_metrics.get('surface_smoothness', {}).get('smoothness_score', 0) * 30
        edge_score = min(geometric_metrics.get('edge_metrics', {}).get('edge_sharpness', 0) * 200, 40)
        
        score = normal_score + smoothness_score + edge_score
        return min(100, score)
        
    def _score_preservation_quality(self, preservation_metrics: Dict) -> float:
        """特征保持质量评分"""
        feature_score = preservation_metrics.get('feature_preservation', {}).get('preservation_ratio', 0) * 30
        structure_score = preservation_metrics.get('structure_preservation', {}).get('structure_score', 0) * 30
        detail_score = preservation_metrics.get('detail_preservation', {}).get('detail_preservation', 0) * 20
        spatial_score = preservation_metrics.get('spatial_preservation', {}).get('spatial_preservation', 0) * 20
        
        score = feature_score + structure_score + detail_score + spatial_score
        return min(100, score)
        
    def _score_registration_quality(self, registration_metrics: Dict) -> float:
        """配准质量评分"""
        distance_score = max(0, 50 - registration_metrics.get('distance_errors', {}).get('rmse', 1) * 1000)
        overlap_score = registration_metrics.get('overlap_analysis', {}).get('overall_overlap_ratio', 0) * 30
        transformation_score = registration_metrics.get('convergence_metrics', {}).get('transformation_quality', 0) * 20
        
        score = distance_score + overlap_score + transformation_score
        return min(100, score)
        
    def _compute_improvements(self, original_metrics: Dict, refined_metrics: Dict) -> Dict:
        """计算改进指标"""
        improvements = {}
        
        # 整体评分改进
        original_score = self.compute_comprehensive_score(original_metrics)
        refined_score = self.compute_comprehensive_score(refined_metrics)
        improvements['overall_improvement'] = refined_score - original_score
        
        # 密度改进
        if 'density' in original_metrics and 'density' in refined_metrics:
            original_uniformity = original_metrics['density'].get('uniformity_score', 0)
            refined_uniformity = refined_metrics['density'].get('uniformity_score', 0)
            improvements['density_uniformity_improvement'] = refined_uniformity - original_uniformity
            
        # 几何质量改进
        if 'geometric' in original_metrics and 'geometric' in refined_metrics:
            original_smoothness = original_metrics['geometric'].get('surface_smoothness', {}).get('smoothness_score', 0)
            refined_smoothness = refined_metrics['geometric'].get('surface_smoothness', {}).get('smoothness_score', 0)
            improvements['smoothness_improvement'] = refined_smoothness - original_smoothness
            
        return improvements
        
    def _generate_recommendations(self, improvements: Dict) -> List[str]:
        """生成优化建议"""
        recommendations = []
        
        if improvements.get('overall_improvement', 0) < 5:
            recommendations.append("整体改进有限，建议调整处理参数或选择不同的处理管道")
            
        if improvements.get('density_uniformity_improvement', 0) < 0:
            recommendations.append("密度均匀性下降，建议检查密度均匀化参数设置")
            
        if improvements.get('smoothness_improvement', 0) < 0:
            recommendations.append("表面平滑度下降，建议减少噪声去除强度或增强细节保护")
            
        if not recommendations:
            recommendations.append("处理效果良好，各项指标均有改善")
            
        return recommendations