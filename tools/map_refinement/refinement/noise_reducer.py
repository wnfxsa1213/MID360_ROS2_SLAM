import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional
import logging
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.distance import cdist
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from functools import partial
import multiprocessing
import os
import sys
import time

# 添加父目录到路径以导入utils模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.progress_manager import progress_bar, get_progress_manager
"""
点云去噪功能模块

实现多种点云去噪算法：
- 统计异常值去除（Statistical Outlier Removal）
- 半径异常值去除（Radius Outlier Removal）
- 密度based去噪（移除稀疏区域）
- 时间一致性去噪（利用多帧信息）
- 自适应去噪（基于局部几何特性）
"""


logger = logging.getLogger(__name__)

class NoiseReductionResult:
    """去噪结果类"""
    
    def __init__(self, cleaned_pcd: o3d.geometry.PointCloud, 
                 noise_pcd: o3d.geometry.PointCloud,
                 noise_indices: np.ndarray,
                 method: str):
        self.cleaned_pcd = cleaned_pcd
        self.noise_pcd = noise_pcd
        self.noise_indices = noise_indices
        self.method = method
        self.noise_ratio = len(noise_indices) / (len(cleaned_pcd.points) + len(noise_indices))
        
    def __repr__(self):
        return f"NoiseReduction(method={self.method}, noise_ratio={self.noise_ratio:.3f})"

class NoiseReducer:
    """点云去噪算法类"""
    
    def __init__(self, config: dict):
        """
        初始化点云去噪器
        
        Args:
            config: 算法配置字典
        """
        # 统计异常值去除配置
        self.stat_mean_k = config.get('statistical_outlier', {}).get('mean_k', 50)
        self.stat_std_dev_mul_thresh = config.get('statistical_outlier', {}).get('std_dev_mul_thresh', 1.0)
        
        # 半径异常值去除配置
        self.radius_outlier_radius = config.get('radius_outlier', {}).get('radius', 0.05)
        self.radius_outlier_min_neighbors = config.get('radius_outlier', {}).get('min_neighbors', 10)
        
        # 密度过滤配置
        self.min_density = config.get('density_filter', {}).get('min_density', 50)  # points per cubic meter

        # 自适应去噪配置
        adaptive_config = config.get('adaptive_denoising', {})
        self.denoising_mode = adaptive_config.get('mode', 'conservative')
        self.threshold_method = adaptive_config.get('threshold_method', 'hybrid')
        self.noise_percentile = adaptive_config.get('noise_percentile', 85)

        # 特征权重配置
        weights = adaptive_config.get('feature_weights', {})
        self.curvature_weight = weights.get('curvature', 0.2)
        self.normal_variation_weight = weights.get('normal_variation', 0.3)
        self.density_weight = weights.get('density', 0.2)

        # 根据模式调整权重
        if self.denoising_mode == 'conservative':
            # 保守模式：减少权重，更宽松
            self.curvature_weight *= 0.7
            self.normal_variation_weight *= 0.7
            self.density_weight *= 0.5
            self.noise_percentile = max(self.noise_percentile, 90)
        elif self.denoising_mode == 'aggressive':
            # 激进模式：增加权重，更严格
            self.curvature_weight *= 1.3
            self.normal_variation_weight *= 1.3
            self.density_weight *= 1.5
            self.noise_percentile = min(self.noise_percentile, 75)
        # balanced模式使用默认权重

        # 性能配置
        performance_config = config.get('performance', {})
        self.num_threads = performance_config.get('num_threads', min(8, multiprocessing.cpu_count()))
        self.memory_limit_gb = performance_config.get('memory_limit_gb', 16)

        # 确保线程数合理
        self.num_threads = max(1, min(self.num_threads, multiprocessing.cpu_count()))

        logger.info(f"点云去噪器初始化完成 - 模式: {self.denoising_mode}, "
                   f"阈值方法: {self.threshold_method}, "
                   f"百分位数: {self.noise_percentile}")
        logger.info(f"特征权重 - 曲率: {self.curvature_weight:.2f}, "
                   f"法向量变化: {self.normal_variation_weight:.2f}, "
                   f"密度: {self.density_weight:.2f}")
        logger.info(f"性能配置 - 线程数: {self.num_threads}, "
                   f"内存限制: {self.memory_limit_gb}GB")
        
    def statistical_outlier_removal(self, pcd: o3d.geometry.PointCloud) -> NoiseReductionResult:
        """
        统计异常值去除

        Args:
            pcd: 输入点云

        Returns:
            去噪结果
        """
        logger.info("开始统计异常值去除")

        # 检查输入点云是否为空
        if len(pcd.points) == 0:
            logger.warning("输入点云为空，跳过统计去噪")
            return NoiseReductionResult(pcd, o3d.geometry.PointCloud(), np.array([]), "statistical")

        # 使用Open3D的统计异常值去除
        cleaned_pcd, inlier_indices = pcd.remove_statistical_outlier(
            nb_neighbors=self.stat_mean_k,
            std_ratio=self.stat_std_dev_mul_thresh
        )
        
        # 获取噪声点
        all_indices = set(range(len(pcd.points)))
        inlier_set = set(inlier_indices)
        noise_indices = np.array(list(all_indices - inlier_set))
        
        noise_pcd = pcd.select_by_index(noise_indices) if len(noise_indices) > 0 else o3d.geometry.PointCloud()
        
        result = NoiseReductionResult(cleaned_pcd, noise_pcd, noise_indices, "statistical")
        
        logger.info(f"统计异常值去除完成: {len(pcd.points)} -> {len(cleaned_pcd.points)} 点 "
                   f"(移除 {len(noise_indices)} 个异常点)")
                   
        return result
        
    def radius_outlier_removal(self, pcd: o3d.geometry.PointCloud) -> NoiseReductionResult:
        """
        半径异常值去除
        
        Args:
            pcd: 输入点云
            
        Returns:
            去噪结果
        """
        logger.info("开始半径异常值去除")
        
        # 使用Open3D的半径异常值去除
        cleaned_pcd, inlier_indices = pcd.remove_radius_outlier(
            nb_points=self.radius_outlier_min_neighbors,
            radius=self.radius_outlier_radius
        )
        
        # 获取噪声点
        all_indices = set(range(len(pcd.points)))
        inlier_set = set(inlier_indices)
        noise_indices = np.array(list(all_indices - inlier_set))
        
        noise_pcd = pcd.select_by_index(noise_indices) if len(noise_indices) > 0 else o3d.geometry.PointCloud()
        
        result = NoiseReductionResult(cleaned_pcd, noise_pcd, noise_indices, "radius")
        
        logger.info(f"半径异常值去除完成: {len(pcd.points)} -> {len(cleaned_pcd.points)} 点 "
                   f"(移除 {len(noise_indices)} 个异常点)")
                   
        return result
        
    def density_based_filtering(self, pcd: o3d.geometry.PointCloud, 
                               voxel_size: float = 0.1) -> NoiseReductionResult:
        """
        基于密度的过滤
        
        Args:
            pcd: 输入点云
            voxel_size: 体素大小用于密度计算
            
        Returns:
            去噪结果
        """
        logger.info("开始密度based过滤")

        points = np.asarray(pcd.points)
        n_points = len(points)

        if n_points == 0:
            return NoiseReductionResult(pcd, o3d.geometry.PointCloud(), np.array([]), "density")

        with progress_bar(3, "密度过滤", unit="步骤") as pbar:
            # 计算每个点的局部密度
            logger.info("步骤1/3: 计算局部密度")
            densities = self._compute_local_density(points, voxel_size)
            pbar.update(1)

            # 过滤低密度点
            logger.info("步骤2/3: 应用密度阈值")
            density_threshold = self.min_density * (voxel_size ** 3)  # 转换为体素密度
            high_density_mask = densities >= density_threshold

            inlier_indices = np.where(high_density_mask)[0]
            noise_indices = np.where(~high_density_mask)[0]
            pbar.update(1)

            # 创建结果点云
            logger.info("步骤3/3: 生成结果点云")
            cleaned_pcd = pcd.select_by_index(inlier_indices.tolist())
            noise_pcd = pcd.select_by_index(noise_indices.tolist()) if len(noise_indices) > 0 else o3d.geometry.PointCloud()
            pbar.update(1)
        
        result = NoiseReductionResult(cleaned_pcd, noise_pcd, noise_indices, "density")
        
        logger.info(f"密度过滤完成: {len(pcd.points)} -> {len(cleaned_pcd.points)} 点 "
                   f"(移除 {len(noise_indices)} 个低密度点)")
                   
        return result
        
    def _compute_local_density(self, points: np.ndarray, radius: float) -> np.ndarray:
        """
        计算每个点的局部密度（多线程优化版本）

        Args:
            points: 点坐标 (N, 3)
            radius: 邻域半径

        Returns:
            密度数组 (N,)
        """
        n_points = len(points)

        logger.info(f"开始密度计算 - 点数: {n_points}")

        # 如果点数较少，使用单线程
        if n_points < 1000 or self.num_threads == 1:
            return self._compute_local_density_single_thread(points, radius)

        # 多线程计算
        return self._compute_local_density_parallel(points, radius)

    def _compute_local_density_single_thread(self, points: np.ndarray, radius: float) -> np.ndarray:
        """单线程密度计算"""
        n_points = len(points)
        densities = np.zeros(n_points)

        # 构建KD树用于邻域搜索
        nbrs = NearestNeighbors(radius=radius, algorithm='kd_tree').fit(points)

        for i in range(n_points):
            # 搜索邻域点
            distances, indices = nbrs.radius_neighbors([points[i]])
            neighbor_count = len(indices[0]) - 1  # 排除自身

            # 计算密度 (点数/体积)
            volume = (4/3) * np.pi * (radius ** 3)
            densities[i] = neighbor_count / volume

        return densities

    def _compute_local_density_parallel(self, points: np.ndarray, radius: float) -> np.ndarray:
        """并行密度计算"""
        n_points = len(points)

        # 将点分块进行并行处理
        chunk_size = max(100, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        densities = np.zeros(n_points)

        def compute_chunk_density(chunk_info):
            start_idx, end_idx = chunk_info
            chunk_densities = np.zeros(end_idx - start_idx)

            # 每个线程创建自己的KD树
            nbrs = NearestNeighbors(radius=radius, algorithm='kd_tree').fit(points)

            for i in range(start_idx, end_idx):
                local_idx = i - start_idx
                # 搜索邻域点
                distances, indices = nbrs.radius_neighbors([points[i]])
                neighbor_count = len(indices[0]) - 1  # 排除自身

                # 计算密度 (点数/体积)
                volume = (4/3) * np.pi * (radius ** 3)
                chunk_densities[local_idx] = neighbor_count / volume

            return start_idx, end_idx, chunk_densities

        # 执行并行计算
        with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
            futures = [executor.submit(compute_chunk_density, chunk) for chunk in chunks]

            for future in futures:
                start_idx, end_idx, chunk_densities = future.result()
                densities[start_idx:end_idx] = chunk_densities

        return densities
        
    def adaptive_noise_removal(self, pcd: o3d.geometry.PointCloud) -> NoiseReductionResult:
        """
        自适应去噪：基于局部几何特性
        
        Args:
            pcd: 输入点云
            
        Returns:
            去噪结果
        """
        logger.info("开始自适应去噪")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points < 10:
            return NoiseReductionResult(pcd, o3d.geometry.PointCloud(), np.array([]), "adaptive")
            
        # 确保有法向量
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(0.1)
            )
            
        normals = np.asarray(pcd.normals)
        
        # 计算多个几何特征（多线程优化）
        pm = get_progress_manager()
        with pm.progress_bar(6, "自适应去噪", unit="步骤") as main_pbar:
            logger.info("步骤1/6: 计算曲率特征")
            start_time = time.time()
            curvatures = self._compute_curvature(pcd)
            curvature_time = time.time() - start_time
            logger.info(f"曲率计算完成，耗时: {curvature_time:.2f}秒")
            main_pbar.update(1)

            logger.info("步骤2/6: 计算法向量变化")
            normal_variations = self._compute_normal_variation(pcd)
            normal_time = time.time() - start_time - curvature_time
            logger.info(f"法向量变化计算完成，耗时: {normal_time:.2f}秒")
            main_pbar.update(1)

            logger.info("步骤3/6: 计算局部密度")
            local_densities = self._compute_local_density(points, 0.05)
            density_time = time.time() - start_time - curvature_time - normal_time
            logger.info(f"密度计算完成，耗时: {density_time:.2f}秒")
            main_pbar.update(1)

            total_feature_time = time.time() - start_time
            logger.info(f"所有几何特征计算完成，总耗时: {total_feature_time:.2f}秒")

            # 归一化特征
            logger.info("步骤4/6: 特征归一化")
            curvatures_norm = self._normalize_feature(curvatures)
            normal_variations_norm = self._normalize_feature(normal_variations)
            densities_norm = self._normalize_feature(local_densities)
            main_pbar.update(1)

            # 计算综合噪声分数 - 使用配置的权重
            logger.info("步骤5/6: 计算噪声分数")
            noise_scores = (curvatures_norm * self.curvature_weight +
                           normal_variations_norm * self.normal_variation_weight +
                           (1 - densities_norm) * self.density_weight)

            # 输出调试信息
            logger.info(f"噪声分数统计: min={np.min(noise_scores):.4f}, "
                       f"max={np.max(noise_scores):.4f}, "
                       f"mean={np.mean(noise_scores):.4f}, "
                       f"std={np.std(noise_scores):.4f}")
            logger.info(f"使用权重 - 曲率: {self.curvature_weight:.2f}, "
                       f"法向量变化: {self.normal_variation_weight:.2f}, "
                       f"密度: {self.density_weight:.2f}")

            # 根据配置选择阈值计算方法
            if self.threshold_method == 'percentile':
                threshold = np.percentile(noise_scores, self.noise_percentile)
                logger.info(f"使用百分位数阈值: {threshold:.4f} (第{self.noise_percentile}百分位)")

            elif self.threshold_method == 'otsu':
                threshold = self._compute_otsu_threshold(noise_scores)
                logger.info(f"使用OTSU阈值: {threshold:.4f}")

            else:  # hybrid方法
                threshold_percentile = np.percentile(noise_scores, self.noise_percentile)
                threshold_otsu = self._compute_otsu_threshold(noise_scores)
                threshold = max(threshold_percentile, threshold_otsu)  # 选择更保守的
                logger.info(f"混合阈值计算: 百分位数={threshold_percentile:.4f}, "
                           f"OTSU={threshold_otsu:.4f}, 最终={threshold:.4f}")
            main_pbar.update(1)

            # 分类点
            logger.info("步骤6/6: 生成清洁点云")
            clean_mask = noise_scores <= threshold
            inlier_indices = np.where(clean_mask)[0]
            noise_indices = np.where(~clean_mask)[0]

            # 创建结果点云
            cleaned_pcd = pcd.select_by_index(inlier_indices.tolist())
            noise_pcd = pcd.select_by_index(noise_indices.tolist()) if len(noise_indices) > 0 else o3d.geometry.PointCloud()
            main_pbar.update(1)
        
        result = NoiseReductionResult(cleaned_pcd, noise_pcd, noise_indices, "adaptive")

        # 计算详细统计信息
        original_count = len(pcd.points)
        remaining_count = len(cleaned_pcd.points)
        removed_count = len(noise_indices)
        removal_percentage = (removed_count / original_count) * 100 if original_count > 0 else 0

        logger.info(f"自适应去噪完成:")
        logger.info(f"  原始点数: {original_count}")
        logger.info(f"  保留点数: {remaining_count}")
        logger.info(f"  移除点数: {removed_count}")
        logger.info(f"  移除比例: {removal_percentage:.2f}%")
        logger.info(f"  使用模式: {self.denoising_mode}")

        # 如果移除比例过高，给出警告
        if removal_percentage > 50:
            logger.warning(f"⚠️  移除了{removal_percentage:.1f}%的点云，这可能过于激进")
            logger.warning("   建议调整为conservative模式或增大noise_percentile参数")

        return result
        
    def _compute_curvature(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """计算曲率（多线程优化版本）"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)

        logger.info(f"开始曲率计算 - 点数: {n_points}, 使用线程数: {self.num_threads}")
        start_time = time.time()

        # 如果点数较少，使用单线程
        if n_points < 1000 or self.num_threads == 1:
            return self._compute_curvature_single_thread(pcd, radius)

        # 多线程计算
        return self._compute_curvature_parallel(pcd, radius)

    def _compute_curvature_single_thread(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """单线程曲率计算"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)
        curvatures = np.zeros(n_points)

        # 构建KD树
        kdtree = o3d.geometry.KDTreeFlann(pcd)

        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

            if k < 4:
                continue

            neighbor_normals = normals[idx[1:]]  # 排除自身
            center_normal = normals[i]

            # 计算法向量变化率
            normal_differences = neighbor_normals - center_normal
            normal_variations = np.linalg.norm(normal_differences, axis=1)
            curvatures[i] = np.mean(normal_variations)

        return curvatures

    def _compute_curvature_parallel(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """并行曲率计算"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)

        # 将点分块进行并行处理
        chunk_size = max(100, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        # 使用ThreadPoolExecutor进行并行计算
        curvatures = np.zeros(n_points)

        def compute_chunk_curvature(chunk_info):
            start_idx, end_idx = chunk_info
            chunk_curvatures = np.zeros(end_idx - start_idx)

            # 每个线程创建自己的KD树（因为Open3D的KD树不是线程安全的）
            kdtree = o3d.geometry.KDTreeFlann(pcd)

            for i in range(start_idx, end_idx):
                local_idx = i - start_idx
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

                if k < 4:
                    continue

                neighbor_normals = normals[idx[1:]]  # 排除自身
                center_normal = normals[i]

                # 计算法向量变化率
                normal_differences = neighbor_normals - center_normal
                normal_variations = np.linalg.norm(normal_differences, axis=1)
                chunk_curvatures[local_idx] = np.mean(normal_variations)

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
        
    def _compute_normal_variation(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """计算法向量变化（多线程优化版本）"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)

        logger.info(f"开始法向量变化计算 - 点数: {n_points}")

        # 如果点数较少，使用单线程
        if n_points < 1000 or self.num_threads == 1:
            return self._compute_normal_variation_single_thread(pcd, radius)

        # 多线程计算
        return self._compute_normal_variation_parallel(pcd, radius)

    def _compute_normal_variation_single_thread(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """单线程法向量变化计算"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)
        variations = np.zeros(n_points)

        kdtree = o3d.geometry.KDTreeFlann(pcd)

        for i in range(n_points):
            [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

            if k < 3:
                continue

            neighbor_normals = normals[idx]

            # 计算法向量标准差
            variations[i] = np.std(neighbor_normals.flatten())

        return variations

    def _compute_normal_variation_parallel(self, pcd: o3d.geometry.PointCloud, radius: float = 0.05) -> np.ndarray:
        """并行法向量变化计算"""
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        n_points = len(points)

        # 将点分块进行并行处理
        chunk_size = max(100, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        variations = np.zeros(n_points)

        def compute_chunk_normal_variation(chunk_info):
            start_idx, end_idx = chunk_info
            chunk_variations = np.zeros(end_idx - start_idx)

            # 每个线程创建自己的KD树
            kdtree = o3d.geometry.KDTreeFlann(pcd)

            for i in range(start_idx, end_idx):
                local_idx = i - start_idx
                [k, idx, _] = kdtree.search_radius_vector_3d(pcd.points[i], radius)

                if k < 3:
                    continue

                neighbor_normals = normals[idx]

                # 计算法向量标准差
                chunk_variations[local_idx] = np.std(neighbor_normals.flatten())

            return start_idx, end_idx, chunk_variations

        # 执行并行计算
        with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
            futures = [executor.submit(compute_chunk_normal_variation, chunk) for chunk in chunks]

            for future in futures:
                start_idx, end_idx, chunk_variations = future.result()
                variations[start_idx:end_idx] = chunk_variations

        return variations
        
    def _normalize_feature(self, feature: np.ndarray) -> np.ndarray:
        """特征归一化"""
        min_val = np.min(feature)
        max_val = np.max(feature)
        
        if max_val - min_val < 1e-8:
            return np.zeros_like(feature)
            
        return (feature - min_val) / (max_val - min_val)
        
    def _compute_otsu_threshold(self, values: np.ndarray) -> float:
        """计算OTSU阈值"""
        # 简化的OTSU实现
        histogram, bins = np.histogram(values, bins=256)
        bin_centers = (bins[:-1] + bins[1:]) / 2
        
        best_threshold = 0
        best_variance = 0
        
        total_pixels = len(values)
        total_weight = np.sum(histogram * bin_centers)
        
        sum_background = 0
        weight_background = 0
        
        for i in range(len(histogram)):
            weight_background += histogram[i]
            if weight_background == 0:
                continue
                
            weight_foreground = total_pixels - weight_background
            if weight_foreground == 0:
                break
                
            sum_background += histogram[i] * bin_centers[i]
            
            mean_background = sum_background / weight_background
            mean_foreground = (total_weight - sum_background) / weight_foreground
            
            # 计算类间方差
            variance_between = weight_background * weight_foreground * (mean_background - mean_foreground) ** 2
            
            if variance_between > best_variance:
                best_variance = variance_between
                best_threshold = bin_centers[i]
                
        return best_threshold
        
    def multi_stage_denoising(self, pcd: o3d.geometry.PointCloud,
                            stages: List[str] = None) -> Tuple[o3d.geometry.PointCloud, List[NoiseReductionResult]]:
        """
        多阶段去噪流程
        
        Args:
            pcd: 输入点云
            stages: 去噪阶段列表，默认为['radius', 'statistical', 'density', 'adaptive']
            
        Returns:
            (最终清理的点云, 各阶段结果列表)
        """
        if stages is None:
            stages = ['radius', 'statistical', 'density', 'adaptive']
            
        logger.info(f"开始多阶段去噪: {' -> '.join(stages)}")

        current_pcd = o3d.geometry.PointCloud(pcd)
        results = []

        with progress_bar(len(stages), "多阶段去噪", unit="阶段") as pbar:
            for stage in stages:
                logger.info(f"执行 {stage} 去噪阶段")

                if stage == 'statistical':
                    result = self.statistical_outlier_removal(current_pcd)
                elif stage == 'radius':
                    result = self.radius_outlier_removal(current_pcd)
                elif stage == 'density':
                    result = self.density_based_filtering(current_pcd)
                elif stage == 'adaptive':
                    result = self.adaptive_noise_removal(current_pcd)
                else:
                    logger.warning(f"未知的去噪阶段: {stage}")
                    pbar.update(1)
                    continue

                results.append(result)
                current_pcd = result.cleaned_pcd

                logger.info(f"{stage} 阶段完成: 剩余 {len(current_pcd.points)} 点")

                # 如果点云已经为空，停止后续处理
                if len(current_pcd.points) == 0:
                    logger.warning(f"在{stage}阶段后点云已为空，停止后续去噪处理")
                    pbar.update(len(stages) - len(results))  # 更新剩余进度
                    break

                pbar.update(1)
            
        total_removed = len(pcd.points) - len(current_pcd.points)
        removal_ratio = total_removed / len(pcd.points) if len(pcd.points) > 0 else 0
        
        logger.info(f"多阶段去噪完成: {len(pcd.points)} -> {len(current_pcd.points)} 点 "
                   f"(总共移除 {total_removed} 点, {removal_ratio:.2%})")
                   
        return current_pcd, results
        
    def preserve_structure_denoising(self, pcd: o3d.geometry.PointCloud,
                                   planes: List = None) -> NoiseReductionResult:
        """
        结构保护去噪：在去噪过程中保护重要的几何结构
        
        Args:
            pcd: 输入点云
            planes: 检测到的平面列表（可选）
            
        Returns:
            去噪结果
        """
        logger.info("开始结构保护去噪")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        # 标记结构点
        structure_mask = np.zeros(n_points, dtype=bool)
        
        if planes:
            # 标记属于平面的点为结构点
            for plane in planes:
                if hasattr(plane, 'indices'):
                    structure_mask[plane.indices] = True
                    
        # 计算基础噪声分数
        result_adaptive = self.adaptive_noise_removal(pcd)
        noise_scores = np.ones(n_points)
        
        # 重新计算噪声分数，考虑结构保护
        if len(result_adaptive.noise_indices) > 0:
            noise_scores[result_adaptive.noise_indices] = 2.0  # 高噪声分数
            
        # 降低结构点的噪声分数
        noise_scores[structure_mask] *= 0.1  # 显著降低结构点被移除的概率
        
        # 基于调整后的分数分类
        threshold = self._compute_otsu_threshold(noise_scores)
        clean_mask = noise_scores <= threshold
        
        inlier_indices = np.where(clean_mask)[0]
        noise_indices = np.where(~clean_mask)[0]
        
        # 创建结果点云
        cleaned_pcd = pcd.select_by_index(inlier_indices.tolist())
        noise_pcd = pcd.select_by_index(noise_indices.tolist()) if len(noise_indices) > 0 else o3d.geometry.PointCloud()
        
        result = NoiseReductionResult(cleaned_pcd, noise_pcd, noise_indices, "structure_preserving")
        
        structure_preserved = np.sum(structure_mask[inlier_indices])
        total_structure = np.sum(structure_mask)
        preservation_ratio = structure_preserved / total_structure if total_structure > 0 else 0
        
        logger.info(f"结构保护去噪完成: {len(pcd.points)} -> {len(cleaned_pcd.points)} 点 "
                   f"(结构点保护率: {preservation_ratio:.2%})")
                   
        return result