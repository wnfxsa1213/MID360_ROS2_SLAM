"""
表面重建功能模块

实现多种表面重建算法：
- 泊松表面重建（Poisson Surface Reconstruction）
- 移动立方体算法（Marching Cubes）
- 球面投影重建（Ball Pivoting Algorithm）
- 自适应网格生成
- 多分辨率重建
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional, Union
import logging
from scipy.spatial import cKDTree
from sklearn.neighbors import NearestNeighbors

logger = logging.getLogger(__name__)

class SurfaceReconstructionResult:
    """表面重建结果类"""
    
    def __init__(self, mesh: o3d.geometry.TriangleMesh, 
                 method: str, 
                 parameters: Dict,
                 quality_metrics: Dict = None):
        self.mesh = mesh
        self.method = method
        self.parameters = parameters
        self.quality_metrics = quality_metrics or {}
        
    def __repr__(self):
        return f"SurfaceReconstruction(method={self.method}, vertices={len(self.mesh.vertices)}, faces={len(self.mesh.triangles)})"

class SurfaceReconstructor:
    """表面重建算法类"""
    
    def __init__(self, config: dict):
        """
        初始化表面重建器
        
        Args:
            config: 算法配置字典
        """
        self.method = config.get('method', 'poisson')
        
        # Poisson重建配置
        self.poisson_depth = config.get('poisson_depth', 9)
        self.poisson_width = config.get('poisson_width', 0)
        self.poisson_scale = config.get('poisson_scale', 1.1)
        
        # Ball Pivoting配置
        self.ball_radii = config.get('ball_radii', [0.005, 0.01, 0.02, 0.04])
        
        # Alpha Shape配置
        self.alpha_value = config.get('alpha_value', 0.03)
        
        logger.info(f"表面重建器初始化完成，使用方法: {self.method}")
        
    def poisson_reconstruction(self, pcd: o3d.geometry.PointCloud) -> SurfaceReconstructionResult:
        """
        泊松表面重建
        
        Args:
            pcd: 输入点云（必须有法向量）
            
        Returns:
            重建结果
        """
        logger.info("开始泊松表面重建")
        
        # 确保点云有法向量
        if not pcd.has_normals():
            logger.info("点云缺少法向量，正在估计法向量")
            pcd.estimate_normals()
            pcd.orient_normals_consistent_tangent_plane(100)
            
        # 执行泊松重建
        try:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd,
                depth=self.poisson_depth,
                width=self.poisson_width,
                scale=self.poisson_scale,
                linear_fit=False
            )
            
            # 移除低密度区域的三角形
            if len(densities) > 0:
                density_threshold = np.quantile(np.asarray(densities), 0.01)
                mesh.remove_triangles_by_mask(np.asarray(densities) < density_threshold)
                mesh.remove_unreferenced_vertices()
                
            # 计算质量指标
            quality_metrics = self._compute_mesh_quality(mesh, pcd)
            
            parameters = {
                'depth': self.poisson_depth,
                'width': self.poisson_width,
                'scale': self.poisson_scale,
                'density_threshold': density_threshold if len(densities) > 0 else 0
            }
            
            result = SurfaceReconstructionResult(mesh, 'poisson', parameters, quality_metrics)
            
            logger.info(f"泊松重建完成: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 三角面")
            
            return result
            
        except Exception as e:
            logger.error(f"泊松重建失败: {str(e)}")
            # 返回空网格
            empty_mesh = o3d.geometry.TriangleMesh()
            return SurfaceReconstructionResult(empty_mesh, 'poisson', {}, {})
            
    def ball_pivoting_reconstruction(self, pcd: o3d.geometry.PointCloud) -> SurfaceReconstructionResult:
        """
        球面投影重建（Ball Pivoting Algorithm）
        
        Args:
            pcd: 输入点云（必须有法向量）
            
        Returns:
            重建结果
        """
        logger.info("开始Ball Pivoting重建")
        
        # 确保点云有法向量
        if not pcd.has_normals():
            logger.info("点云缺少法向量，正在估计法向量")
            pcd.estimate_normals()
            pcd.orient_normals_consistent_tangent_plane(100)
            
        # 根据点云密度自适应调整球半径
        points = np.asarray(pcd.points)
        if len(points) > 100:
            # 估计平均点间距
            kdtree = cKDTree(points)
            distances, _ = kdtree.query(points, k=2)
            avg_distance = np.mean(distances[:, 1])  # 到最近邻的平均距离
            
            # 基于平均距离调整球半径
            adaptive_radii = [avg_distance * scale for scale in [0.5, 1.0, 2.0, 4.0]]
        else:
            adaptive_radii = self.ball_radii
            
        try:
            # 执行Ball Pivoting重建
            radii = o3d.utility.DoubleVector(adaptive_radii)
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                pcd, radii
            )
            
            # 清理网格
            mesh.remove_duplicated_triangles()
            mesh.remove_duplicated_vertices()
            mesh.remove_non_manifold_edges()
            mesh.remove_degenerate_triangles()
            mesh.remove_unreferenced_vertices()
            
            # 计算质量指标
            quality_metrics = self._compute_mesh_quality(mesh, pcd)
            
            parameters = {
                'radii': adaptive_radii
            }
            
            result = SurfaceReconstructionResult(mesh, 'ball_pivoting', parameters, quality_metrics)
            
            logger.info(f"Ball Pivoting重建完成: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 三角面")
            
            return result
            
        except Exception as e:
            logger.error(f"Ball Pivoting重建失败: {str(e)}")
            empty_mesh = o3d.geometry.TriangleMesh()
            return SurfaceReconstructionResult(empty_mesh, 'ball_pivoting', {}, {})
            
    def alpha_shape_reconstruction(self, pcd: o3d.geometry.PointCloud) -> SurfaceReconstructionResult:
        """
        Alpha Shape重建
        
        Args:
            pcd: 输入点云
            
        Returns:
            重建结果
        """
        logger.info("开始Alpha Shape重建")
        
        try:
            # 执行Alpha Shape重建
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                pcd, self.alpha_value
            )
            
            # 清理网格
            mesh.remove_duplicated_triangles()
            mesh.remove_duplicated_vertices()
            mesh.remove_unreferenced_vertices()
            
            # 计算质量指标
            quality_metrics = self._compute_mesh_quality(mesh, pcd)
            
            parameters = {
                'alpha': self.alpha_value
            }
            
            result = SurfaceReconstructionResult(mesh, 'alpha_shape', parameters, quality_metrics)
            
            logger.info(f"Alpha Shape重建完成: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 三角面")
            
            return result
            
        except Exception as e:
            logger.error(f"Alpha Shape重建失败: {str(e)}")
            empty_mesh = o3d.geometry.TriangleMesh()
            return SurfaceReconstructionResult(empty_mesh, 'alpha_shape', {}, {})
            
    def adaptive_reconstruction(self, pcd: o3d.geometry.PointCloud) -> SurfaceReconstructionResult:
        """
        自适应表面重建：根据点云特性选择最佳方法
        
        Args:
            pcd: 输入点云
            
        Returns:
            最佳重建结果
        """
        logger.info("开始自适应表面重建")
        
        # 分析点云特性
        analysis = self._analyze_point_cloud(pcd)
        
        # 根据分析结果选择重建方法
        methods_to_try = []
        
        if analysis['density'] > 1000 and analysis['noise_level'] < 0.1:
            # 高密度低噪声：优先使用Poisson
            methods_to_try = ['poisson', 'ball_pivoting', 'alpha_shape']
        elif analysis['density'] > 500:
            # 中等密度：优先使用Ball Pivoting
            methods_to_try = ['ball_pivoting', 'alpha_shape', 'poisson']
        else:
            # 低密度：使用Alpha Shape
            methods_to_try = ['alpha_shape', 'ball_pivoting']
            
        logger.info(f"根据点云分析选择重建方法优先级: {' -> '.join(methods_to_try)}")
        
        best_result = None
        best_score = -1
        
        for method in methods_to_try:
            try:
                if method == 'poisson':
                    result = self.poisson_reconstruction(pcd)
                elif method == 'ball_pivoting':
                    result = self.ball_pivoting_reconstruction(pcd)
                elif method == 'alpha_shape':
                    result = self.alpha_shape_reconstruction(pcd)
                else:
                    continue
                    
                # 评估重建质量
                if len(result.mesh.triangles) > 0:
                    score = self._evaluate_reconstruction_quality(result, analysis)
                    
                    if score > best_score:
                        best_score = score
                        best_result = result
                        
                    logger.info(f"{method} 重建质量评分: {score:.3f}")
                    
            except Exception as e:
                logger.warning(f"{method} 重建失败: {str(e)}")
                continue
                
        if best_result is None:
            logger.warning("所有重建方法都失败，返回空网格")
            empty_mesh = o3d.geometry.TriangleMesh()
            return SurfaceReconstructionResult(empty_mesh, 'adaptive', {}, {})
            
        # 更新结果类型
        best_result.method = f"adaptive_{best_result.method}"
        
        logger.info(f"自适应重建完成，选择方法: {best_result.method}")
        
        return best_result
        
    def multi_resolution_reconstruction(self, pcd: o3d.geometry.PointCloud,
                                      levels: int = 3) -> List[SurfaceReconstructionResult]:
        """
        多分辨率表面重建
        
        Args:
            pcd: 输入点云
            levels: 分辨率级别数
            
        Returns:
            不同分辨率的重建结果列表
        """
        logger.info(f"开始 {levels} 级多分辨率表面重建")
        
        results = []
        
        for level in range(levels):
            # 计算当前级别的体素大小
            voxel_size = 0.01 * (2 ** level)  # 从1cm开始，每级别翻倍
            
            logger.info(f"重建级别 {level+1}/{levels}, 体素大小: {voxel_size:.3f}m")
            
            # 下采样点云
            downsampled_pcd = pcd.voxel_down_sample(voxel_size)
            
            # 确保有足够的点进行重建
            if len(downsampled_pcd.points) < 100:
                logger.warning(f"级别 {level+1} 点数不足 ({len(downsampled_pcd.points)})")
                continue
                
            # 执行重建
            result = self.adaptive_reconstruction(downsampled_pcd)
            
            if len(result.mesh.triangles) > 0:
                # 添加分辨率信息
                result.parameters['resolution_level'] = level + 1
                result.parameters['voxel_size'] = voxel_size
                result.parameters['input_points'] = len(downsampled_pcd.points)
                
                results.append(result)
                
                logger.info(f"级别 {level+1} 重建完成: {len(result.mesh.vertices)} 顶点")
            else:
                logger.warning(f"级别 {level+1} 重建失败")
                
        logger.info(f"多分辨率重建完成，成功重建 {len(results)} 个级别")
        
        return results
        
    def _analyze_point_cloud(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """分析点云特性"""
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return {'density': 0, 'noise_level': 1.0, 'uniformity': 0}
            
        # 计算包围盒体积
        bbox = pcd.get_axis_aligned_bounding_box()
        bbox_volume = bbox.volume()
        
        # 密度估计
        density = n_points / max(bbox_volume, 1e-6)
        
        # 噪声水平估计
        if not pcd.has_normals():
            pcd.estimate_normals()
            
        # 使用统计异常值检测估计噪声
        _, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        noise_level = 1.0 - len(inliers) / n_points
        
        # 均匀性估计
        if n_points > 100:
            # 计算最近邻距离的变异系数
            kdtree = cKDTree(points)
            distances, _ = kdtree.query(points, k=2)
            nearest_distances = distances[:, 1]
            uniformity = 1.0 / (1.0 + np.std(nearest_distances) / np.mean(nearest_distances))
        else:
            uniformity = 0.5
            
        return {
            'density': density,
            'noise_level': noise_level,
            'uniformity': uniformity,
            'point_count': n_points,
            'bbox_volume': bbox_volume
        }
        
    def _compute_mesh_quality(self, mesh: o3d.geometry.TriangleMesh, 
                            original_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算网格质量指标"""
        if len(mesh.triangles) == 0:
            return {'valid': False}
            
        # 基本几何指标
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()
        
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        
        # 三角形质量
        triangle_areas = []
        triangle_aspect_ratios = []
        
        for triangle in triangles:
            v0, v1, v2 = vertices[triangle]
            
            # 计算面积
            edge1 = v1 - v0
            edge2 = v2 - v0
            area = 0.5 * np.linalg.norm(np.cross(edge1, edge2))
            triangle_areas.append(area)
            
            # 计算长宽比
            edge_lengths = [
                np.linalg.norm(v1 - v0),
                np.linalg.norm(v2 - v1),
                np.linalg.norm(v0 - v2)
            ]
            max_edge = max(edge_lengths)
            min_edge = min(edge_lengths)
            aspect_ratio = max_edge / max(min_edge, 1e-8)
            triangle_aspect_ratios.append(aspect_ratio)
            
        # 与原始点云的拟合度
        original_points = np.asarray(original_pcd.points)
        mesh_sample = mesh.sample_points_uniformly(number_of_points=min(len(original_points), 10000))
        sampled_points = np.asarray(mesh_sample.points)
        
        # 计算点到网格的距离
        if len(sampled_points) > 0 and len(original_points) > 0:
            kdtree = cKDTree(sampled_points)
            distances, _ = kdtree.query(original_points)
            fitting_error = np.mean(distances)
        else:
            fitting_error = float('inf')
            
        quality_metrics = {
            'valid': True,
            'vertex_count': len(vertices),
            'triangle_count': len(triangles),
            'mean_triangle_area': np.mean(triangle_areas),
            'std_triangle_area': np.std(triangle_areas),
            'mean_aspect_ratio': np.mean(triangle_aspect_ratios),
            'max_aspect_ratio': np.max(triangle_aspect_ratios),
            'fitting_error': fitting_error,
            'is_manifold': mesh.is_edge_manifold(),
            'is_watertight': mesh.is_watertight()
        }
        
        return quality_metrics
        
    def _evaluate_reconstruction_quality(self, result: SurfaceReconstructionResult,
                                       analysis: Dict) -> float:
        """评估重建质量并返回综合评分"""
        if not result.quality_metrics.get('valid', False):
            return 0.0
            
        metrics = result.quality_metrics
        
        # 基础评分组件
        scores = []
        
        # 1. 三角形质量评分 (0-1)
        if metrics.get('mean_aspect_ratio', float('inf')) < 10:
            triangle_score = 1.0 / (1.0 + metrics.get('mean_aspect_ratio', 10) / 10)
        else:
            triangle_score = 0.1
        scores.append(triangle_score * 0.2)
        
        # 2. 拟合度评分 (0-1)
        fitting_error = metrics.get('fitting_error', float('inf'))
        if fitting_error < float('inf'):
            fitting_score = 1.0 / (1.0 + fitting_error * 100)
        else:
            fitting_score = 0.0
        scores.append(fitting_score * 0.4)
        
        # 3. 网格完整性评分 (0-1)
        manifold_score = 1.0 if metrics.get('is_manifold', False) else 0.5
        watertight_score = 1.0 if metrics.get('is_watertight', False) else 0.8
        integrity_score = (manifold_score + watertight_score) / 2
        scores.append(integrity_score * 0.2)
        
        # 4. 细节保持评分 (0-1)
        triangle_count = metrics.get('triangle_count', 0)
        expected_triangles = analysis.get('point_count', 1000) * 2  # 经验值
        detail_score = min(triangle_count / expected_triangles, 1.0)
        scores.append(detail_score * 0.2)
        
        return sum(scores)
        
    def reconstruct_surface(self, pcd: o3d.geometry.PointCloud,
                          method: str = None) -> SurfaceReconstructionResult:
        """
        执行表面重建
        
        Args:
            pcd: 输入点云
            method: 重建方法，可选值：'poisson', 'ball_pivoting', 'alpha_shape', 'adaptive'
            
        Returns:
            重建结果
        """
        if method is None:
            method = self.method
            
        logger.info(f"开始表面重建，方法: {method}")
        
        if len(pcd.points) < 3:
            logger.warning("点云点数不足，无法进行表面重建")
            empty_mesh = o3d.geometry.TriangleMesh()
            return SurfaceReconstructionResult(empty_mesh, method, {}, {})
            
        if method == 'poisson':
            return self.poisson_reconstruction(pcd)
        elif method == 'ball_pivoting':
            return self.ball_pivoting_reconstruction(pcd)
        elif method == 'alpha_shape':
            return self.alpha_shape_reconstruction(pcd)
        elif method == 'adaptive':
            return self.adaptive_reconstruction(pcd)
        else:
            logger.error(f"未知的重建方法: {method}")
            return self.adaptive_reconstruction(pcd)  # 默认使用自适应方法