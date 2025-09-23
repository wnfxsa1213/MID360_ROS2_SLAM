"""
边缘特征强化算法

检测和增强点云中的边缘特征，保持建筑物的棱角和细节特征。
主要功能：
- 基于曲率的边缘检测
- Harris3D角点检测
- 线特征提取
- 边缘权重增强
- 边缘连续性约束
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional
import logging
import multiprocessing
import time
import sys
import os
from concurrent.futures import ThreadPoolExecutor
from sklearn.neighbors import NearestNeighbors

# 添加父目录到路径以导入utils模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.progress_manager import progress_bar, get_progress_manager
from scipy.spatial.distance import cdist

logger = logging.getLogger(__name__)

class EdgeInfo:
    """边缘信息类"""
    
    def __init__(self, points: np.ndarray, indices: np.ndarray, 
                 curvatures: np.ndarray, edge_type: str):
        self.points = points
        self.indices = indices
        self.curvatures = curvatures
        self.edge_type = edge_type  # 'curvature', 'harris', 'line'
        self.strength = np.mean(curvatures)
        
    def __repr__(self):
        return f"Edge(type={self.edge_type}, points={len(self.points)}, strength={self.strength:.3f})"

class EdgeEnhancer:
    """边缘特征强化算法类"""
    
    def __init__(self, config: dict):
        """
        初始化边缘特征强化算法
        
        Args:
            config: 算法配置字典
        """
        self.curvature_threshold = config.get('curvature_threshold', 0.1)
        self.edge_search_radius = config.get('edge_search_radius', 0.05)
        self.edge_weight_multiplier = config.get('edge_weight_multiplier', 2.0)
        self.line_feature_min_points = config.get('line_feature_min_points', 20)
        
        # 性能配置
        performance_config = config.get('performance', {})
        self.num_threads = performance_config.get('num_threads', min(8, multiprocessing.cpu_count()))
        self.num_threads = max(1, min(self.num_threads, multiprocessing.cpu_count()))

        # 内部状态
        self.detected_edges = []
        self.edge_weights = None

        logger.info(f"边缘特征强化算法初始化完成 - 线程数: {self.num_threads}")
        
    def compute_curvature(self, pcd: o3d.geometry.PointCloud, 
                         radius: Optional[float] = None) -> np.ndarray:
        """
        计算点云的曲率
        
        Args:
            pcd: 输入点云
            radius: 邻域搜索半径
            
        Returns:
            曲率数组 (N,)
        """
        if radius is None:
            radius = self.edge_search_radius
            
        points = np.asarray(pcd.points)
        n_points = len(points)
        curvatures = np.zeros(n_points)
        
        # 确保点云有法向量
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(radius)
            )
            
        normals = np.asarray(pcd.normals)
        
        # 构建KD树
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        logger.info(f"开始计算曲率 - 点数: {n_points}, 线程数: {self.num_threads}")
        start_time = time.time()

        # 根据点数选择处理策略
        if n_points < 1000 or self.num_threads == 1:
            curvatures = self._compute_curvature_single_thread(pcd, kdtree, points, normals, radius)
        else:
            curvatures = self._compute_curvature_parallel(pcd, points, normals, radius)

        elapsed_time = time.time() - start_time
        logger.info(f"曲率计算完成，耗时: {elapsed_time:.2f}秒")
        
        return curvatures
        
    def detect_curvature_edges(self, pcd: o3d.geometry.PointCloud) -> EdgeInfo:
        """
        基于曲率检测边缘特征
        
        Args:
            pcd: 输入点云
            
        Returns:
            边缘信息
        """
        logger.info("开始基于曲率的边缘检测")
        
        # 计算曲率
        curvatures = self.compute_curvature(pcd)
        
        # 找到高曲率点
        edge_mask = curvatures > self.curvature_threshold
        edge_indices = np.where(edge_mask)[0]
        
        if len(edge_indices) == 0:
            logger.warning("未检测到高曲率边缘点")
            return EdgeInfo(np.array([]), np.array([]), np.array([]), 'curvature')
            
        points = np.asarray(pcd.points)
        edge_points = points[edge_indices]
        edge_curvatures = curvatures[edge_indices]
        
        edge_info = EdgeInfo(edge_points, edge_indices, edge_curvatures, 'curvature')
        
        logger.info(f"检测到 {len(edge_indices)} 个高曲率边缘点")
        
        return edge_info
        
    def detect_harris_corners(self, pcd: o3d.geometry.PointCloud) -> EdgeInfo:
        """
        Harris3D角点检测
        
        Args:
            pcd: 输入点云
            
        Returns:
            角点信息
        """
        logger.info("开始Harris3D角点检测")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        # 确保有法向量
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(self.edge_search_radius)
            )
            
        normals = np.asarray(pcd.normals)
        
        # 构建KD树
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        harris_responses = np.zeros(n_points)
        
        for i in range(n_points):
            # 搜索邻域
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.edge_search_radius)
            
            if k < 6:  # 需要足够的邻域点
                continue
                
            neighbor_points = points[idx[1:]]  # 排除自身
            center_point = points[i]
            
            # 计算协方差矩阵
            centered_points = neighbor_points - center_point
            cov_matrix = np.cov(centered_points.T)
            
            # 计算特征值
            eigenvalues = np.linalg.eigvals(cov_matrix)
            eigenvalues = np.sort(eigenvalues)[::-1]  # 降序排列
            
            # Harris响应函数
            if len(eigenvalues) >= 3:
                # 3D Harris响应：检测三个方向都有变化的点（角点）
                det_cov = np.prod(eigenvalues)
                trace_cov = np.sum(eigenvalues)
                
                k = 0.04  # Harris参数
                harris_responses[i] = det_cov - k * (trace_cov ** 3)
                
        # 非极大值抑制
        corner_threshold = np.percentile(harris_responses, 95)  # 取前5%
        corner_candidates = harris_responses > corner_threshold
        
        # 进一步过滤：确保局部极大值
        corner_indices = []
        for i in range(n_points):
            if not corner_candidates[i]:
                continue
                
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], self.edge_search_radius)
            
            # 检查是否为局部最大值
            neighbor_responses = harris_responses[idx]
            if harris_responses[i] == np.max(neighbor_responses):
                corner_indices.append(i)
                
        corner_indices = np.array(corner_indices)
        
        if len(corner_indices) == 0:
            logger.warning("未检测到Harris角点")
            return EdgeInfo(np.array([]), np.array([]), np.array([]), 'harris')
            
        corner_points = points[corner_indices]
        corner_responses = harris_responses[corner_indices]
        
        corner_info = EdgeInfo(corner_points, corner_indices, corner_responses, 'harris')
        
        logger.info(f"检测到 {len(corner_indices)} 个Harris角点")
        
        return corner_info
        
    def detect_line_features(self, pcd: o3d.geometry.PointCloud) -> List[EdgeInfo]:
        """
        检测线特征
        
        Args:
            pcd: 输入点云
            
        Returns:
            线特征列表
        """
        logger.info("开始线特征检测")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points < self.line_feature_min_points:
            logger.warning(f"点云点数不足以检测线特征 ({n_points} < {self.line_feature_min_points})")
            return []
            
        # 计算曲率
        curvatures = self.compute_curvature(pcd)
        
        # 找到高曲率点作为候选
        high_curvature_mask = curvatures > self.curvature_threshold
        candidate_indices = np.where(high_curvature_mask)[0]
        
        if len(candidate_indices) < self.line_feature_min_points:
            logger.warning("高曲率点数不足以形成线特征")
            return []
            
        candidate_points = points[candidate_indices]
        
        # 使用RANSAC检测线段
        line_features = []
        remaining_indices = set(candidate_indices)
        
        max_lines = 20  # 最大线特征数
        for line_id in range(max_lines):
            if len(remaining_indices) < self.line_feature_min_points:
                break
                
            remaining_points = points[list(remaining_indices)]
            
            # RANSAC线拟合
            best_inliers = []
            best_score = 0
            
            for _ in range(100):  # RANSAC迭代次数
                if len(remaining_points) < 2:
                    break
                    
                # 随机选择两个点定义直线
                sample_indices = np.random.choice(len(remaining_points), 2, replace=False)
                p1, p2 = remaining_points[sample_indices]
                
                # 直线方向向量
                line_direction = p2 - p1
                line_direction_norm = np.linalg.norm(line_direction)
                
                if line_direction_norm < 1e-6:
                    continue
                    
                line_direction = line_direction / line_direction_norm
                
                # 计算所有点到直线的距离
                point_to_line = remaining_points - p1
                cross_products = np.cross(point_to_line, line_direction)
                distances = np.linalg.norm(cross_products, axis=1)
                
                # 找到内点
                inliers = distances < self.edge_search_radius
                inlier_count = np.sum(inliers)
                
                if inlier_count > best_score and inlier_count >= self.line_feature_min_points:
                    best_score = inlier_count
                    best_inliers = np.where(inliers)[0]
                    
            if len(best_inliers) >= self.line_feature_min_points:
                # 创建线特征
                line_point_indices = [list(remaining_indices)[i] for i in best_inliers]
                line_points = points[line_point_indices]
                line_curvatures = curvatures[line_point_indices]
                
                line_info = EdgeInfo(line_points, np.array(line_point_indices), 
                                   line_curvatures, 'line')
                line_features.append(line_info)
                
                # 从候选点中移除已使用的点
                for idx in best_inliers:
                    remaining_indices.discard(list(remaining_indices)[idx])
                    
                logger.debug(f"检测到线特征 {line_id+1}: {len(best_inliers)} 个点")
            else:
                break
                
        logger.info(f"总共检测到 {len(line_features)} 个线特征")
        
        return line_features
        
    def compute_edge_weights(self, pcd: o3d.geometry.PointCloud, 
                           edges: List[EdgeInfo]) -> np.ndarray:
        """
        计算边缘点的权重
        
        Args:
            pcd: 输入点云
            edges: 边缘特征列表
            
        Returns:
            点权重数组 (N,)
        """
        points = np.asarray(pcd.points)
        n_points = len(points)
        weights = np.ones(n_points)  # 默认权重为1
        
        # 为不同类型的边缘特征分配权重
        type_weights = {
            'curvature': 1.5,
            'harris': 2.0,
            'line': 1.8
        }
        
        for edge in edges:
            edge_weight = type_weights.get(edge.edge_type, 1.0)
            strength_weight = 1.0 + edge.strength  # 基于特征强度的权重
            
            final_weight = edge_weight * strength_weight * self.edge_weight_multiplier
            
            # 应用权重到边缘点
            weights[edge.indices] = np.maximum(weights[edge.indices], final_weight)
            
        self.edge_weights = weights
        
        logger.info(f"计算边缘权重完成，权重范围: [{np.min(weights):.2f}, {np.max(weights):.2f}]")
        
        return weights
        
    def enhance_edge_density(self, pcd: o3d.geometry.PointCloud,
                           edges: List[EdgeInfo]) -> o3d.geometry.PointCloud:
        """
        在边缘区域增加点密度
        
        Args:
            pcd: 输入点云
            edges: 边缘特征列表
            
        Returns:
            密度增强后的点云
        """
        logger.info("开始边缘密度增强")
        
        original_points = np.asarray(pcd.points)
        enhanced_points = [original_points]
        
        # 为每个边缘特征添加插值点
        for edge in edges:
            if edge.edge_type == 'line' and len(edge.points) >= 2:
                # 对线特征进行插值
                sorted_indices = self._sort_line_points(edge.points)
                sorted_points = edge.points[sorted_indices]
                
                # 在相邻点之间插值
                for i in range(len(sorted_points) - 1):
                    p1, p2 = sorted_points[i], sorted_points[i + 1]
                    distance = np.linalg.norm(p2 - p1)
                    
                    # 如果距离较大，添加插值点
                    if distance > self.edge_search_radius * 2:
                        num_interpolated = int(distance / self.edge_search_radius) - 1
                        for j in range(1, num_interpolated + 1):
                            alpha = j / (num_interpolated + 1)
                            interpolated_point = p1 + alpha * (p2 - p1)
                            enhanced_points.append(interpolated_point.reshape(1, -1))
                            
        if len(enhanced_points) > 1:
            all_points = np.vstack(enhanced_points)
            enhanced_pcd = o3d.geometry.PointCloud()
            enhanced_pcd.points = o3d.utility.Vector3dVector(all_points)
            
            # 复制原始颜色和法向量（如果有）
            if pcd.has_colors():
                original_colors = np.asarray(pcd.colors)
                # 为新点分配默认颜色
                new_colors = np.tile([1.0, 0.0, 0.0], (len(all_points) - len(original_points), 1))
                all_colors = np.vstack([original_colors, new_colors])
                enhanced_pcd.colors = o3d.utility.Vector3dVector(all_colors)
                
            logger.info(f"边缘密度增强: {len(original_points)} -> {len(all_points)} 点")
            
            return enhanced_pcd
        else:
            return pcd
            
    def _sort_line_points(self, points: np.ndarray) -> np.ndarray:
        """
        对线上的点进行排序
        
        Args:
            points: 线上的点 (N, 3)
            
        Returns:
            排序后的索引
        """
        if len(points) <= 2:
            return np.arange(len(points))
            
        # 使用最近邻连接策略排序
        sorted_indices = [0]  # 从第一个点开始
        remaining = set(range(1, len(points)))
        
        while remaining:
            current_point = points[sorted_indices[-1]]
            distances = [np.linalg.norm(points[i] - current_point) for i in remaining]
            nearest_idx = list(remaining)[np.argmin(distances)]
            sorted_indices.append(nearest_idx)
            remaining.remove(nearest_idx)
            
        return np.array(sorted_indices)
        
    def enhance_edges(self, pcd: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.PointCloud, Dict]:
        """
        执行完整的边缘特征增强流程

        Args:
            pcd: 输入点云

        Returns:
            (增强后的点云, 处理信息)
        """
        logger.info("开始边缘特征增强流程")

        pm = get_progress_manager()
        with pm.progress_bar(5, "边缘特征增强", unit="步骤") as main_pbar:
            # 1. 检测不同类型的边缘特征
            logger.info("步骤1/5: 检测曲率边缘")
            curvature_edges = self.detect_curvature_edges(pcd)
            main_pbar.update(1)

            logger.info("步骤2/5: 检测Harris角点")
            harris_corners = self.detect_harris_corners(pcd)
            main_pbar.update(1)

            logger.info("步骤3/5: 检测线特征")
            line_features = self.detect_line_features(pcd)

            # 合并所有边缘特征
            all_edges = [curvature_edges, harris_corners] + line_features
            self.detected_edges = [edge for edge in all_edges if len(edge.points) > 0]
            main_pbar.update(1)

            # 2. 计算边缘权重
            logger.info("步骤4/5: 计算边缘权重")
            edge_weights = self.compute_edge_weights(pcd, self.detected_edges)
            main_pbar.update(1)

            # 3. 增强边缘密度
            logger.info("步骤5/5: 增强边缘密度")
            enhanced_pcd = self.enhance_edge_density(pcd, self.detected_edges)
            main_pbar.update(1)
        
        # 处理信息
        processing_info = {
            'curvature_edges': len(curvature_edges.points),
            'harris_corners': len(harris_corners.points),
            'line_features': len(line_features),
            'total_edge_points': sum(len(edge.points) for edge in self.detected_edges),
            'edge_weights': edge_weights,
            'original_points': len(pcd.points),
            'enhanced_points': len(enhanced_pcd.points)
        }
        
        logger.info(f"边缘特征增强完成: 曲率边缘={processing_info['curvature_edges']}, "
                   f"Harris角点={processing_info['harris_corners']}, "
                   f"线特征={processing_info['line_features']}")

        return enhanced_pcd, processing_info

    def _compute_curvature_single_thread(self, pcd: o3d.geometry.PointCloud, kdtree: o3d.geometry.KDTreeFlann,
                                       points: np.ndarray, normals: np.ndarray, radius: float) -> np.ndarray:
        """单线程曲率计算"""
        n_points = len(points)
        curvatures = np.zeros(n_points)

        for i in range(n_points):
            # 搜索邻域点
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

            if k < 4:  # 需要至少4个点来计算曲率
                continue

            # 获取邻域点和法向量
            neighbor_points = points[idx[1:]]  # 排除自身
            neighbor_normals = normals[idx[1:]]

            # 计算法向量变化率（曲率的近似）
            center_normal = normals[i]
            normal_differences = neighbor_normals - center_normal
            normal_variations = np.linalg.norm(normal_differences, axis=1)

            # 计算距离权重
            center_point = points[i]
            distances = np.linalg.norm(neighbor_points - center_point, axis=1)
            weights = np.exp(-distances / radius)  # 高斯权重

            # 加权平均曲率
            curvatures[i] = np.average(normal_variations, weights=weights)

        return curvatures

    def _compute_curvature_parallel(self, pcd: o3d.geometry.PointCloud, points: np.ndarray,
                                  normals: np.ndarray, radius: float) -> np.ndarray:
        """多线程曲率计算"""
        n_points = len(points)
        chunk_size = max(50, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        curvatures = np.zeros(n_points)

        def compute_chunk_curvature(chunk_info):
            start_idx, end_idx = chunk_info
            chunk_curvatures = np.zeros(end_idx - start_idx)

            # 每个线程创建自己的KD树
            kdtree = o3d.geometry.KDTreeFlann(pcd)

            for i in range(start_idx, end_idx):
                local_idx = i - start_idx
                # 搜索邻域点
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

                if k < 4:
                    continue

                # 获取邻域点和法向量
                neighbor_points = points[idx[1:]]  # 排除自身
                neighbor_normals = normals[idx[1:]]

                # 计算法向量变化率（曲率的近似）
                center_normal = normals[i]
                normal_differences = neighbor_normals - center_normal
                normal_variations = np.linalg.norm(normal_differences, axis=1)

                # 计算距离权重
                center_point = points[i]
                distances = np.linalg.norm(neighbor_points - center_point, axis=1)
                weights = np.exp(-distances / radius)  # 高斯权重

                # 加权平均曲率
                chunk_curvatures[local_idx] = np.average(normal_variations, weights=weights)

            return start_idx, end_idx, chunk_curvatures

        # 执行并行计算
        with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
            futures = [executor.submit(compute_chunk_curvature, chunk) for chunk in chunks]

            with progress_bar(len(chunks), "曲率计算", unit="块") as pbar:
                for future in futures:
                    start_idx, end_idx, chunk_curvatures = future.result()
                    curvatures[start_idx:end_idx] = chunk_curvatures
                    pbar.update(1)

        return curvatures