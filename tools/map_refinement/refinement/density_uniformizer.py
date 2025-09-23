import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional, Union
import logging
import multiprocessing
import time
import sys
import os
from concurrent.futures import ThreadPoolExecutor
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import cKDTree, Voronoi
from scipy.spatial.distance import cdist
from scipy.interpolate import griddata

# 添加父目录到路径以导入utils模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.progress_manager import progress_bar, get_progress_manager
"""
密度均匀化处理模块

实现点云密度分布的均匀化处理：
- 自适应采样
- 网格uniform采样
- 密度感知下采样
- 空洞填充和插值
- 分层密度处理
- 结构感知密度调整
"""


logger = logging.getLogger(__name__)

class DensityUniformizationResult:
    """密度均匀化结果类"""
    
    def __init__(self, uniformized_pcd: o3d.geometry.PointCloud,
                 density_map: np.ndarray,
                 uniformity_map: np.ndarray,
                 method: str,
                 parameters: Dict):
        self.uniformized_pcd = uniformized_pcd
        self.density_map = density_map  # 原始密度分布
        self.uniformity_map = uniformity_map  # 均匀化后的密度分布
        self.method = method
        self.parameters = parameters
        
        # 计算均匀化指标
        if len(density_map) > 0 and len(uniformity_map) > 0:
            self.density_variance_reduction = (
                np.var(density_map) - np.var(uniformity_map)
            ) / np.var(density_map) if np.var(density_map) > 0 else 0
        else:
            self.density_variance_reduction = 0
            
    def __repr__(self):
        return f"DensityUniformization(method={self.method}, points={len(self.uniformized_pcd.points)}, variance_reduction={self.density_variance_reduction:.3f})"

class DensityUniformizer:
    """密度均匀化处理器类"""
    
    def __init__(self, config: dict):
        """
        初始化密度均匀化处理器
        
        Args:
            config: 算法配置字典
        """
        self.target_density = config.get('target_density', 100)  # points per cubic meter
        self.adaptive_sampling = config.get('adaptive_sampling', True)
        self.hole_filling = config.get('hole_filling', True)
        self.interpolation_method = config.get('interpolation_method', 'linear')
        
        # 处理参数
        self.voxel_size = 0.05  # 默认体素大小
        self.min_points_per_voxel = 5
        self.max_points_per_voxel = 50
        self.interpolation_radius = 0.1

        # 性能配置
        performance_config = config.get('performance', {})
        self.num_threads = performance_config.get('num_threads', min(8, multiprocessing.cpu_count()))
        self.num_threads = max(1, min(self.num_threads, multiprocessing.cpu_count()))

        logger.info(f"密度均匀化处理器初始化完成 - 线程数: {self.num_threads}")
        
    def compute_density_map(self, pcd: o3d.geometry.PointCloud, 
                          radius: float = None) -> np.ndarray:
        """
        计算点云密度分布
        
        Args:
            pcd: 输入点云
            radius: 密度计算半径
            
        Returns:
            密度数组 (N,)
        """
        if radius is None:
            radius = self.voxel_size * 2
            
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return np.array([])
            
        logger.info(f"开始计算密度分布 - 点数: {n_points}, 线程数: {self.num_threads}")
        start_time = time.time()

        # 根据点数选择处理策略
        if n_points < 1000 or self.num_threads == 1:
            densities = self._compute_density_single_thread(points, radius)
        else:
            densities = self._compute_density_parallel(points, radius)

        elapsed_time = time.time() - start_time
        logger.info(f"密度分布计算完成，耗时: {elapsed_time:.2f}秒，"
                   f"密度范围: [{np.min(densities):.1f}, {np.max(densities):.1f}]")
        
        return densities
        
    def voxel_based_uniformization(self, pcd: o3d.geometry.PointCloud) -> DensityUniformizationResult:
        """
        基于体素的密度均匀化
        
        Args:
            pcd: 输入点云
            
        Returns:
            均匀化结果
        """
        logger.info("开始基于体素的密度均匀化")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return DensityUniformizationResult(
                pcd, np.array([]), np.array([]),
                "voxel_based", {}
            )
            
        # 计算原始密度分布
        original_densities = self.compute_density_map(pcd)
        
        # 创建体素网格
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd, voxel_size=self.voxel_size
        )
        
        # 获取体素信息
        voxels = voxel_grid.get_voxels()
        
        if len(voxels) == 0:
            logger.warning("未能创建体素网格")
            return DensityUniformizationResult(
                pcd, original_densities, original_densities,
                "voxel_based", {}
            )
            
        # 分析每个体素的点密度
        voxel_point_counts = {}
        voxel_points = {}
        
        # 将点分配到体素
        for i, point in enumerate(points):
            voxel_coord = voxel_grid.get_voxel(point)
            voxel_key = tuple(voxel_coord)
            
            if voxel_key not in voxel_point_counts:
                voxel_point_counts[voxel_key] = 0
                voxel_points[voxel_key] = []
                
            voxel_point_counts[voxel_key] += 1
            voxel_points[voxel_key].append(i)
            
        # 计算目标点数
        voxel_volume = self.voxel_size ** 3
        target_points_per_voxel = int(self.target_density * voxel_volume)
        target_points_per_voxel = max(self.min_points_per_voxel, 
                                     min(target_points_per_voxel, self.max_points_per_voxel))
        
        logger.info(f"目标每体素点数: {target_points_per_voxel}")
        
        # 处理每个体素
        uniformized_points = []
        processed_indices = set()
        
        for voxel_key, point_indices in voxel_points.items():
            current_count = len(point_indices)
            voxel_points_coords = points[point_indices]
            
            if current_count == target_points_per_voxel:
                # 密度已经合适，直接保留
                uniformized_points.extend(voxel_points_coords)
                processed_indices.update(point_indices)
                
            elif current_count > target_points_per_voxel:
                # 密度过高，进行下采样
                sampled_points = self._downsample_voxel(
                    voxel_points_coords, target_points_per_voxel
                )
                uniformized_points.extend(sampled_points)
                processed_indices.update(point_indices)
                
            else:
                # 密度过低，进行上采样
                if self.adaptive_sampling:
                    upsampled_points = self._upsample_voxel(
                        voxel_points_coords, target_points_per_voxel
                    )
                    uniformized_points.extend(upsampled_points)
                    processed_indices.update(point_indices)
                else:
                    # 不进行上采样，保留原始点
                    uniformized_points.extend(voxel_points_coords)
                    processed_indices.update(point_indices)
                    
        # 添加未处理的点
        for i in range(n_points):
            if i not in processed_indices:
                uniformized_points.append(points[i])
                
        # 创建结果点云
        if len(uniformized_points) > 0:
            uniformized_points = np.array(uniformized_points)
            result_pcd = o3d.geometry.PointCloud()
            result_pcd.points = o3d.utility.Vector3dVector(uniformized_points)
            
            # 复制属性
            if pcd.has_colors() and len(uniformized_points) <= len(points):
                # 简化：只有下采样时保留颜色
                result_pcd.colors = pcd.colors
            if pcd.has_normals() and len(uniformized_points) <= len(points):
                result_pcd.normals = pcd.normals
                
        else:
            result_pcd = o3d.geometry.PointCloud(pcd)
            uniformized_points = points
            
        # 计算新的密度分布
        new_densities = self.compute_density_map(result_pcd)
        
        parameters = {
            'voxel_size': self.voxel_size,
            'target_points_per_voxel': target_points_per_voxel,
            'original_points': n_points,
            'uniformized_points': len(uniformized_points)
        }
        
        result = DensityUniformizationResult(
            result_pcd, original_densities, new_densities,
            "voxel_based", parameters
        )
        
        logger.info(f"体素密度均匀化完成: {n_points} -> {len(uniformized_points)} 点")

        return result

    def adaptive_density_uniformization(self, pcd: o3d.geometry.PointCloud) -> DensityUniformizationResult:
        """
        自适应密度均匀化

        Args:
            pcd: 输入点云

        Returns:
            均匀化结果
        """
        logger.info("开始自适应密度均匀化")

        points = np.asarray(pcd.points)
        n_points = len(points)

        if n_points == 0:
            return DensityUniformizationResult(
                pcd, np.array([]), np.array([]),
                "adaptive", {}
            )

        # 计算原始密度分布
        original_densities = self.compute_density_map(pcd)

        # 对于自适应方法，我们直接使用体素方法的简化版本
        # 这里可以根据需要实现更复杂的自适应算法
        logger.info("使用体素下采样进行自适应密度均匀化")
        result_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # 计算新的密度分布
        new_densities = self.compute_density_map(result_pcd)

        parameters = {
            'voxel_size': self.voxel_size,
            'original_points': n_points,
            'uniformized_points': len(result_pcd.points)
        }

        result = DensityUniformizationResult(
            result_pcd, original_densities, new_densities,
            "adaptive", parameters
        )

        logger.info(f"自适应密度均匀化完成: {n_points} -> {len(result_pcd.points)} 点")

        return result

    def _downsample_voxel(self, voxel_points: np.ndarray, target_count: int) -> List[np.ndarray]:
        """体素内下采样"""
        if len(voxel_points) <= target_count:
            return voxel_points.tolist()
            
        # 使用泊松盘采样进行均匀下采样
        # 简化版本：随机采样 + 最远点采样
        
        # 首先随机选择一个点作为种子
        seed_idx = np.random.randint(len(voxel_points))
        selected_points = [voxel_points[seed_idx]]
        remaining_indices = list(range(len(voxel_points)))
        remaining_indices.remove(seed_idx)
        
        # 迭代选择距离已选点最远的点
        for _ in range(target_count - 1):
            if not remaining_indices:
                break
                
            max_min_distance = -1
            best_idx = -1
            
            for idx in remaining_indices:
                candidate_point = voxel_points[idx]
                
                # 计算到所有已选点的最小距离
                min_distance = min([
                    np.linalg.norm(candidate_point - selected_point)
                    for selected_point in selected_points
                ])
                
                if min_distance > max_min_distance:
                    max_min_distance = min_distance
                    best_idx = idx
                    
            if best_idx != -1:
                selected_points.append(voxel_points[best_idx])
                remaining_indices.remove(best_idx)
                
        return selected_points
        
    def _upsample_voxel(self, voxel_points: np.ndarray, target_count: int) -> List[np.ndarray]:
        """体素内上采样"""
        if len(voxel_points) >= target_count:
            return voxel_points.tolist()
            
        if len(voxel_points) == 0:
            return []
            
        current_points = voxel_points.tolist()
        points_to_add = target_count - len(voxel_points)
        
        # 基于现有点的插值生成新点
        for _ in range(points_to_add):
            if len(current_points) < 2:
                # 如果点太少，在现有点附近随机生成
                base_point = current_points[0]
                noise = np.random.normal(0, self.voxel_size * 0.1, 3)
                new_point = base_point + noise
            else:
                # 在现有点之间插值
                idx1, idx2 = np.random.choice(len(current_points), 2, replace=False)
                alpha = np.random.uniform(0.2, 0.8)  # 避免过于接近端点
                new_point = alpha * current_points[idx1] + (1 - alpha) * current_points[idx2]
                
                # 添加小量噪声避免共线
                noise = np.random.normal(0, self.voxel_size * 0.05, 3)
                new_point = new_point + noise
                
            current_points.append(new_point)
            
        return current_points
        
    def adaptive_density_uniformization(self, pcd: o3d.geometry.PointCloud) -> DensityUniformizationResult:
        """
        自适应密度均匀化
        
        Args:
            pcd: 输入点云
            
        Returns:
            均匀化结果
        """
        logger.info("开始自适应密度均匀化")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points == 0:
            return DensityUniformizationResult(
                pcd, np.array([]), np.array([]),
                "adaptive", {}
            )
            
        # 计算原始密度分布
        original_densities = self.compute_density_map(pcd)
        
        # 分析密度分布特征
        density_stats = {
            'mean': np.mean(original_densities),
            'std': np.std(original_densities),
            'min': np.min(original_densities),
            'max': np.max(original_densities),
            'median': np.median(original_densities)
        }
        
        logger.info(f"密度统计: 均值={density_stats['mean']:.1f}, "
                   f"标准差={density_stats['std']:.1f}, "
                   f"范围=[{density_stats['min']:.1f}, {density_stats['max']:.1f}]")
        
        # 根据局部密度特征进行自适应处理
        uniformized_points = []
        processed_mask = np.zeros(n_points, dtype=bool)
        
        # 分类处理不同密度区域
        high_density_threshold = density_stats['mean'] + density_stats['std']
        low_density_threshold = density_stats['mean'] - density_stats['std']
        
        high_density_indices = np.where(original_densities > high_density_threshold)[0]
        low_density_indices = np.where(original_densities < low_density_threshold)[0]
        normal_density_indices = np.where(
            (original_densities >= low_density_threshold) & 
            (original_densities <= high_density_threshold)
        )[0]
        
        logger.info(f"密度区域分类: 高密度={len(high_density_indices)}, "
                   f"正常密度={len(normal_density_indices)}, "
                   f"低密度={len(low_density_indices)}")
        
        # 处理正常密度区域（直接保留）
        uniformized_points.extend(points[normal_density_indices])
        processed_mask[normal_density_indices] = True
        
        # 处理高密度区域（下采样）
        if len(high_density_indices) > 0:
            downsampled_points = self._adaptive_downsample(
                points[high_density_indices], 
                original_densities[high_density_indices],
                target_density=density_stats['mean']
            )
            uniformized_points.extend(downsampled_points)
            processed_mask[high_density_indices] = True
            
        # 处理低密度区域（选择性上采样）
        if len(low_density_indices) > 0 and self.adaptive_sampling:
            upsampled_points = self._adaptive_upsample(
                points[low_density_indices],
                original_densities[low_density_indices],
                target_density=density_stats['mean']
            )
            uniformized_points.extend(upsampled_points)
            processed_mask[low_density_indices] = True
            
        # 添加未处理的点
        unprocessed_indices = np.where(~processed_mask)[0]
        if len(unprocessed_indices) > 0:
            uniformized_points.extend(points[unprocessed_indices])
            
        # 创建结果点云
        if len(uniformized_points) > 0:
            uniformized_points = np.array(uniformized_points)
            result_pcd = o3d.geometry.PointCloud()
            result_pcd.points = o3d.utility.Vector3dVector(uniformized_points)
        else:
            result_pcd = o3d.geometry.PointCloud(pcd)
            uniformized_points = points
            
        # 计算新的密度分布
        new_densities = self.compute_density_map(result_pcd)
        
        parameters = {
            'original_density_stats': density_stats,
            'high_density_threshold': high_density_threshold,
            'low_density_threshold': low_density_threshold,
            'processed_regions': {
                'high_density': len(high_density_indices),
                'normal_density': len(normal_density_indices),
                'low_density': len(low_density_indices)
            }
        }
        
        result = DensityUniformizationResult(
            result_pcd, original_densities, new_densities,
            "adaptive", parameters
        )
        
        logger.info(f"自适应密度均匀化完成: {n_points} -> {len(uniformized_points)} 点")
        
        return result
        
    def _adaptive_downsample(self, points: np.ndarray, densities: np.ndarray,
                           target_density: float) -> List[np.ndarray]:
        """自适应下采样"""
        n_points = len(points)
        if n_points == 0:
            return []
            
        # 计算采样比例
        mean_density = np.mean(densities)
        sampling_ratio = target_density / mean_density if mean_density > 0 else 1.0
        sampling_ratio = min(sampling_ratio, 1.0)  # 不进行上采样
        
        target_count = max(1, int(n_points * sampling_ratio))
        
        # 使用重要性采样：密度越高的点被选中的概率越低
        weights = 1.0 / (densities + 1e-6)  # 避免除零
        weights = weights / np.sum(weights)
        
        # 随机采样
        selected_indices = np.random.choice(
            n_points, size=target_count, replace=False, p=weights
        )
        
        return points[selected_indices].tolist()
        
    def _adaptive_upsample(self, points: np.ndarray, densities: np.ndarray,
                         target_density: float) -> List[np.ndarray]:
        """自适应上采样"""
        n_points = len(points)
        if n_points == 0:
            return []
            
        # 计算需要增加的点数
        mean_density = np.mean(densities)
        density_ratio = target_density / mean_density if mean_density > 0 else 1.0
        
        if density_ratio <= 1.0:
            return points.tolist()  # 不需要上采样
            
        target_count = int(n_points * density_ratio)
        points_to_add = target_count - n_points
        
        current_points = points.tolist()
        
        # 在低密度区域生成新点
        for _ in range(points_to_add):
            if len(current_points) >= 2:
                # 在现有点之间插值
                idx1, idx2 = np.random.choice(len(points), 2, replace=False)
                alpha = np.random.uniform(0.3, 0.7)
                new_point = alpha * points[idx1] + (1 - alpha) * points[idx2]
                
                # 添加噪声
                noise = np.random.normal(0, self.interpolation_radius * 0.1, 3)
                new_point = new_point + noise
                
                current_points.append(new_point)
            else:
                # 在单点附近生成
                base_point = points[0] if len(points) > 0 else np.zeros(3)
                noise = np.random.normal(0, self.interpolation_radius, 3)
                new_point = base_point + noise
                current_points.append(new_point)
                
        return current_points
        
    def hole_filling(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        空洞填充
        
        Args:
            pcd: 输入点云
            
        Returns:
            填充后的点云
        """
        if not self.hole_filling:
            return pcd
            
        logger.info("开始空洞填充")
        
        points = np.asarray(pcd.points)
        n_points = len(points)
        
        if n_points < 10:
            return pcd
            
        # 检测空洞（低密度区域）
        densities = self.compute_density_map(pcd)
        density_threshold = np.percentile(densities, 20)  # 最低20%密度为空洞
        
        hole_candidates = np.where(densities < density_threshold)[0]
        
        if len(hole_candidates) == 0:
            logger.info("未检测到需要填充的空洞")
            return pcd
            
        logger.info(f"检测到 {len(hole_candidates)} 个空洞候选点")
        
        # 为空洞区域生成填充点
        filled_points = points.tolist()
        kdtree = cKDTree(points)
        
        for hole_idx in hole_candidates:
            hole_point = points[hole_idx]
            
            # 搜索邻域
            neighbor_indices = kdtree.query_ball_point(hole_point, self.interpolation_radius)
            neighbor_points = points[neighbor_indices]
            
            if len(neighbor_points) >= 3:
                # 在邻域内插值生成新点
                centroid = np.mean(neighbor_points, axis=0)
                
                # 在空洞点和邻域质心之间插值
                for alpha in [0.3, 0.7]:
                    interpolated_point = alpha * hole_point + (1 - alpha) * centroid
                    filled_points.append(interpolated_point)
                    
        # 创建填充后的点云
        if len(filled_points) > n_points:
            filled_pcd = o3d.geometry.PointCloud()
            filled_pcd.points = o3d.utility.Vector3dVector(np.array(filled_points))
            
            logger.info(f"空洞填充完成: {n_points} -> {len(filled_points)} 点")
            return filled_pcd
        else:
            return pcd
            
    def uniformize_density(self, pcd: o3d.geometry.PointCloud,
                          method: str = "adaptive") -> DensityUniformizationResult:
        """
        执行密度均匀化
        
        Args:
            pcd: 输入点云
            method: 均匀化方法，可选值：'voxel_based', 'adaptive'
            
        Returns:
            均匀化结果
        """
        logger.info(f"开始密度均匀化，方法: {method}")

        if len(pcd.points) < 3:
            logger.warning("点云点数不足，跳过密度均匀化")
            return DensityUniformizationResult(
                pcd, np.array([]), np.array([]),
                method, {}
            )

        pm = get_progress_manager()
        with pm.progress_bar(3, "密度均匀化", unit="步骤") as main_pbar:
            # 1. 空洞填充（如果启用）
            logger.info("步骤1/3: 空洞填充处理")
            if self.hole_filling:
                filled_pcd = self.perform_hole_filling(pcd)
            else:
                filled_pcd = pcd
            main_pbar.update(1)

            # 2. 执行密度均匀化
            logger.info(f"步骤2/3: 执行{method}均匀化")
            if method == "voxel_based":
                result = self.voxel_based_uniformization(filled_pcd)
            elif method == "adaptive":
                result = self.adaptive_density_uniformization(filled_pcd)
            else:
                logger.error(f"未知的密度均匀化方法: {method}")
                result = self.adaptive_density_uniformization(filled_pcd)  # 默认方法
            main_pbar.update(1)

            # 3. 后处理
            logger.info("步骤3/3: 结果处理")
            main_pbar.update(1)
            
        logger.info(f"密度均匀化完成，密度方差减少: {result.density_variance_reduction:.2%}")

        return result

    def perform_hole_filling(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        执行空洞填充处理

        Args:
            pcd: 输入点云

        Returns:
            填充后的点云
        """
        logger.info("开始空洞填充处理")

        # 简单的空洞填充实现
        # 对于当前版本，我们使用基本的体素下采样来填充稀疏区域
        filled_pcd = o3d.geometry.PointCloud(pcd)

        # 如果点云过于稀疏，进行轻微的上采样
        if len(filled_pcd.points) < 1000:
            logger.info("点云过于稀疏，跳过空洞填充")
            return filled_pcd

        # 使用较小的体素来保持细节
        fill_voxel_size = self.voxel_size * 0.5
        filled_pcd = filled_pcd.voxel_down_sample(voxel_size=fill_voxel_size)

        logger.info(f"空洞填充完成: {len(pcd.points)} -> {len(filled_pcd.points)} 点")

        return filled_pcd

    def _compute_density_single_thread(self, points: np.ndarray, radius: float) -> np.ndarray:
        """单线程密度计算"""
        import time
        start_time = time.time()

        logger.debug("使用单线程计算点密度")
        n_points = len(points)
        densities = np.zeros(n_points)

        # 构建KD树
        kdtree = cKDTree(points)

        # 逐点计算密度
        for i in range(n_points):
            neighbor_indices = kdtree.query_ball_point(points[i], radius)
            densities[i] = len(neighbor_indices)

        elapsed_time = time.time() - start_time
        logger.debug(f"单线程密度计算完成，耗时: {elapsed_time:.2f}秒")

        return densities

    def _compute_density_parallel(self, points: np.ndarray, radius: float) -> np.ndarray:
        """多线程密度计算"""
        import time
        start_time = time.time()

        logger.debug(f"使用{self.num_threads}线程并行计算点密度")
        n_points = len(points)
        chunk_size = max(10, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        densities = np.zeros(n_points)

        def compute_chunk_density(chunk_info):
            """计算数据块的密度"""
            start_idx, end_idx = chunk_info
            chunk_densities = []

            # 每个线程创建自己的KD树
            kdtree = cKDTree(points)

            for i in range(start_idx, end_idx):
                neighbor_indices = kdtree.query_ball_point(points[i], radius)
                chunk_densities.append(len(neighbor_indices))

            return start_idx, end_idx, chunk_densities

        # 执行并行计算
        with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
            futures = [executor.submit(compute_chunk_density, chunk) for chunk in chunks]

            with progress_bar(len(chunks), "密度计算", unit="块") as pbar:
                for future in futures:
                    start_idx, end_idx, chunk_densities = future.result()
                    densities[start_idx:end_idx] = chunk_densities
                    pbar.update(1)

        elapsed_time = time.time() - start_time
        logger.debug(f"并行密度计算完成: {n_points}个点，{self.num_threads}线程，耗时{elapsed_time:.2f}秒")

        return densities