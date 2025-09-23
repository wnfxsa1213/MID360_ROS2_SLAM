import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional
import logging
from sklearn.cluster import DBSCAN
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from scipy.spatial import cKDTree
import sys
import os

# 添加父目录到路径以导入utils模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.progress_manager import progress_bar, get_progress_manager
"""
平面结构保护算法

检测和保护点云中的平面结构，确保在精细化过程中保持建筑物的几何特征。
主要功能：
- RANSAC平面检测
- 平面约束优化
- 平行/垂直关系保持
- 平面内点保护
"""


logger = logging.getLogger(__name__)

class PlaneInfo:
    """平面信息类"""
    
    def __init__(self, normal: np.ndarray, distance: float, points: np.ndarray, indices: np.ndarray):
        self.normal = normal / np.linalg.norm(normal)  # 单位法向量
        self.distance = distance  # 到原点距离
        self.points = points  # 平面上的点
        self.indices = indices  # 点在原始点云中的索引
        self.area = len(points)  # 平面面积（点数近似）
        
    def __repr__(self):
        return f"Plane(normal={self.normal}, distance={self.distance:.3f}, points={len(self.points)})"

class PlanePreserver:
    """平面结构保护算法类"""
    
    def __init__(self, config: dict):
        """
        初始化平面保护算法
        
        Args:
            config: 算法配置字典
        """
        self.distance_threshold = config.get('ransac_distance_threshold', 0.02)
        self.max_iterations = config.get('ransac_max_iterations', 1000)
        self.min_plane_points = config.get('min_plane_points', 100)
        self.parallel_threshold = np.radians(config.get('parallel_threshold', 5.0))
        self.perpendicular_threshold = np.radians(config.get('perpendicular_threshold', 85.0))
        
        # 性能配置
        performance_config = config.get('performance', {})
        self.num_threads = performance_config.get('num_threads', min(8, multiprocessing.cpu_count()))
        self.num_threads = max(1, min(self.num_threads, multiprocessing.cpu_count()))

        # 内部状态
        self.detected_planes = []
        self.plane_relationships = {}

        logger.info(f"平面结构保护算法初始化完成 - 线程数: {self.num_threads}")
        
    def detect_planes(self, pcd: o3d.geometry.PointCloud,
                     max_planes: int = 10) -> List[PlaneInfo]:
        """
        检测点云中的平面结构

        Args:
            pcd: 输入点云
            max_planes: 最大平面数量

        Returns:
            检测到的平面列表
        """
        logger.info("开始检测平面结构")

        planes = []
        remaining_pcd = o3d.geometry.PointCloud(pcd)
        original_points = np.asarray(pcd.points)

        with progress_bar(max_planes, "平面检测", unit="平面") as pbar:
            for i in range(max_planes):
                if len(remaining_pcd.points) < self.min_plane_points:
                    pbar.update(max_planes - i)  # 更新剩余进度
                    break

                # RANSAC平面拟合
                plane_model, inliers = remaining_pcd.segment_plane(
                    distance_threshold=self.distance_threshold,
                    ransac_n=3,
                    num_iterations=self.max_iterations
                )

                if len(inliers) < self.min_plane_points:
                    logger.debug(f"平面 {i+1}: 内点数量不足 ({len(inliers)})")
                    pbar.update(1)
                    continue

                # 提取平面参数
                a, b, c, d = plane_model
                normal = np.array([a, b, c])
                distance = abs(d) / np.linalg.norm(normal)
                normal = normal / np.linalg.norm(normal)

                # 确保法向量指向一致（朝上为正）
                if normal[2] < 0:
                    normal = -normal
                    distance = -distance

                # 获取平面上的点
                inlier_cloud = remaining_pcd.select_by_index(inliers)
                plane_points = np.asarray(inlier_cloud.points)

                # 在原始点云中找到对应的索引 (多线程优化)
                if len(plane_points) < 100 or self.num_threads == 1:
                    # 小数据集使用单线程
                    original_indices = self._find_original_indices_single_thread(plane_points, original_points)
                else:
                    # 大数据集使用多线程
                    original_indices = self._find_original_indices_parallel(plane_points, original_points)

                plane_info = PlaneInfo(normal, distance, plane_points, original_indices)
                planes.append(plane_info)

                logger.info(f"检测到平面 {i+1}: 法向量={normal}, "
                           f"距离={distance:.3f}, 点数={len(inliers)}")

                # 移除已检测的平面点
                remaining_indices = [idx for idx in range(len(remaining_pcd.points))
                                   if idx not in inliers]
                remaining_pcd = remaining_pcd.select_by_index(remaining_indices)

                # 更新进度
                pbar.update(1)
            
        self.detected_planes = planes
        logger.info(f"总共检测到 {len(planes)} 个平面")
        
        return planes
        
    def analyze_plane_relationships(self, planes: List[PlaneInfo]) -> Dict:
        """
        分析平面间的几何关系
        
        Args:
            planes: 平面列表
            
        Returns:
            平面关系字典
        """
        relationships = {
            'parallel_pairs': [],
            'perpendicular_pairs': [],
            'dominant_directions': []
        }
        
        # 分析两两平面关系
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                plane1, plane2 = planes[i], planes[j]
                
                # 计算法向量夹角
                dot_product = np.dot(plane1.normal, plane2.normal)
                angle = np.arccos(np.clip(abs(dot_product), 0, 1))
                
                # 判断平行关系
                if angle < self.parallel_threshold or angle > (np.pi - self.parallel_threshold):
                    relationships['parallel_pairs'].append((i, j, angle))
                    logger.debug(f"平面 {i}-{j} 平行, 角度={np.degrees(angle):.1f}°")
                    
                # 判断垂直关系
                elif abs(angle - np.pi/2) < self.perpendicular_threshold:
                    relationships['perpendicular_pairs'].append((i, j, angle))
                    logger.debug(f"平面 {i}-{j} 垂直, 角度={np.degrees(angle):.1f}°")
                    
        # 识别主导方向
        normals = np.array([plane.normal for plane in planes])
        
        # 聚类法向量以找到主导方向
        if len(normals) > 0:
            # 使用球面坐标聚类
            phi = np.arctan2(normals[:, 1], normals[:, 0])  # 方位角
            theta = np.arccos(normals[:, 2])  # 仰角
            
            spherical_coords = np.column_stack([phi, theta])
            
            if len(spherical_coords) >= 2:
                clustering = DBSCAN(eps=0.2, min_samples=1).fit(spherical_coords)
                
                for label in set(clustering.labels_):
                    if label == -1:  # 噪声点
                        continue
                    cluster_indices = np.where(clustering.labels_ == label)[0]
                    cluster_normals = normals[cluster_indices]
                    mean_normal = np.mean(cluster_normals, axis=0)
                    mean_normal = mean_normal / np.linalg.norm(mean_normal)
                    
                    relationships['dominant_directions'].append({
                        'direction': mean_normal,
                        'planes': cluster_indices.tolist(),
                        'count': len(cluster_indices)
                    })
                    
        self.plane_relationships = relationships
        
        logger.info(f"平面关系分析完成: 平行对={len(relationships['parallel_pairs'])}, "
                   f"垂直对={len(relationships['perpendicular_pairs'])}, "
                   f"主导方向={len(relationships['dominant_directions'])}")
                   
        return relationships
        
    def create_plane_constraints(self, planes: List[PlaneInfo]) -> List[Dict]:
        """
        创建平面约束条件
        
        Args:
            planes: 平面列表
            
        Returns:
            约束条件列表
        """
        constraints = []
        
        # 平面完整性约束
        for i, plane in enumerate(planes):
            constraint = {
                'type': 'plane_integrity',
                'plane_id': i,
                'normal': plane.normal,
                'distance': plane.distance,
                'points': plane.indices,
                'weight': 1.0
            }
            constraints.append(constraint)
            
        # 平行约束
        for i, j, angle in self.plane_relationships.get('parallel_pairs', []):
            constraint = {
                'type': 'parallel',
                'plane_ids': [i, j],
                'target_angle': 0.0,
                'weight': 0.8
            }
            constraints.append(constraint)
            
        # 垂直约束
        for i, j, angle in self.plane_relationships.get('perpendicular_pairs', []):
            constraint = {
                'type': 'perpendicular', 
                'plane_ids': [i, j],
                'target_angle': np.pi / 2,
                'weight': 0.8
            }
            constraints.append(constraint)
            
        logger.info(f"创建了 {len(constraints)} 个平面约束")
        return constraints
        
    def apply_plane_constraints(self, pcd: o3d.geometry.PointCloud,
                              constraints: List[Dict]) -> o3d.geometry.PointCloud:
        """
        应用平面约束优化点云
        
        Args:
            pcd: 输入点云
            constraints: 约束条件列表
            
        Returns:
            优化后的点云
        """
        logger.info("开始应用平面约束")
        
        optimized_pcd = o3d.geometry.PointCloud(pcd)
        points = np.asarray(optimized_pcd.points)
        
        # 迭代优化
        for iteration in range(10):  # 最多10次迭代
            total_adjustment = 0.0
            
            for constraint in constraints:
                if constraint['type'] == 'plane_integrity':
                    # 平面完整性约束：将点投影到平面上
                    plane_id = constraint['plane_id']
                    normal = constraint['normal']
                    distance = constraint['distance']
                    point_indices = constraint['points']
                    weight = constraint['weight']
                    
                    if plane_id < len(self.detected_planes):
                        plane_points = points[point_indices]
                        
                        # 计算点到平面的距离
                        distances_to_plane = np.dot(plane_points, normal) + distance
                        
                        # 投影到平面（加权调整）
                        adjustment = -distances_to_plane.reshape(-1, 1) * normal.reshape(1, -1) * weight
                        points[point_indices] += adjustment
                        
                        total_adjustment += np.sum(np.abs(adjustment))
                        
            logger.debug(f"迭代 {iteration+1}: 总调整量={total_adjustment:.6f}")
            
            if total_adjustment < 1e-6:
                logger.info(f"约束优化在第 {iteration+1} 次迭代收敛")
                break
                
        # 更新点云
        optimized_pcd.points = o3d.utility.Vector3dVector(points)
        
        logger.info("平面约束应用完成")
        return optimized_pcd
        
    def filter_non_planar_noise(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        过滤非平面区域的噪声点
        
        Args:
            pcd: 输入点云
            
        Returns:
            过滤后的点云
        """
        if not self.detected_planes:
            logger.warning("未检测到平面，跳过非平面噪声过滤")
            return pcd
            
        points = np.asarray(pcd.points)
        keep_indices = []
        
        # 标记属于平面的点
        planar_points = set()
        for plane in self.detected_planes:
            planar_points.update(plane.indices)
            
        # 对非平面点进行噪声过滤
        non_planar_indices = [i for i in range(len(points)) if i not in planar_points]
        
        if non_planar_indices:
            non_planar_pcd = pcd.select_by_index(non_planar_indices)
            
            # 统计异常值去除
            filtered_pcd, _ = non_planar_pcd.remove_statistical_outlier(
                nb_neighbors=20, std_ratio=2.0
            )
            
            # 获取保留的非平面点索引 - 优化版本
            filtered_points = np.asarray(filtered_pcd.points)

            # 使用KD树进行高效最近邻搜索
            kdtree = cKDTree(points)

            with progress_bar(len(filtered_points), "非平面点索引匹配", unit="点") as pbar:
                for point in filtered_points:
                    # 找到最近邻点
                    distance, closest_idx = kdtree.query(point)
                    if distance < 1e-6:  # 确保是同一个点
                        keep_indices.append(closest_idx)
                    pbar.update(1)
                    
        # 添加所有平面点
        keep_indices.extend(list(planar_points))
        keep_indices = sorted(set(keep_indices))
        
        filtered_pcd = pcd.select_by_index(keep_indices)
        
        logger.info(f"非平面噪声过滤: {len(points)} -> {len(filtered_pcd.points)} 点")
        
        return filtered_pcd
        
    def preserve_planes(self, pcd: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.PointCloud, Dict]:
        """
        执行完整的平面结构保护流程

        Args:
            pcd: 输入点云

        Returns:
            (保护后的点云, 处理信息)
        """
        logger.info("开始平面结构保护流程")

        pm = get_progress_manager()
        with pm.progress_bar(5, "平面结构保护", unit="步骤") as main_pbar:
            # 1. 检测平面
            logger.info("步骤1/5: 检测平面")
            planes = self.detect_planes(pcd)
            main_pbar.update(1)

            if not planes:
                logger.warning("未检测到平面结构")
                main_pbar.update(4)  # 跳过剩余步骤
                return pcd, {'planes': [], 'constraints': []}

            # 2. 分析平面关系
            logger.info("步骤2/5: 分析平面关系")
            relationships = self.analyze_plane_relationships(planes)
            main_pbar.update(1)

            # 3. 创建约束条件
            logger.info("步骤3/5: 创建约束条件")
            constraints = self.create_plane_constraints(planes)
            main_pbar.update(1)

            # 4. 应用约束优化
            logger.info("步骤4/5: 应用约束优化")
            optimized_pcd = self.apply_plane_constraints(pcd, constraints)
            main_pbar.update(1)

            # 5. 过滤非平面噪声
            logger.info("步骤5/5: 过滤非平面噪声")
            final_pcd = self.filter_non_planar_noise(optimized_pcd)
            main_pbar.update(1)
        
        processing_info = {
            'planes': planes,
            'relationships': relationships,
            'constraints': constraints,
            'original_points': len(pcd.points),
            'final_points': len(final_pcd.points)
        }
        
        logger.info("平面结构保护流程完成")

        return final_pcd, processing_info

    def _find_original_indices_single_thread(self, plane_points: np.ndarray, original_points: np.ndarray) -> np.ndarray:
        """单线程最近邻搜索 - 优化版本使用KD树"""
        original_indices = []

        # 构建KD树进行高效搜索
        kdtree = cKDTree(original_points)

        for point in plane_points:
            _, closest_idx = kdtree.query(point)
            original_indices.append(closest_idx)
        return np.array(original_indices)

    def _find_original_indices_parallel(self, plane_points: np.ndarray, original_points: np.ndarray) -> np.ndarray:
        """多线程最近邻搜索 - 优化版本使用KD树"""
        import time
        start_time = time.time()

        n_points = len(plane_points)
        chunk_size = max(10, n_points // self.num_threads)
        chunks = [(i, min(i + chunk_size, n_points)) for i in range(0, n_points, chunk_size)]

        original_indices = np.zeros(n_points, dtype=int)

        def find_chunk_indices(chunk_info):
            start_idx, end_idx = chunk_info
            chunk_indices = []

            # 每个线程创建自己的KD树
            kdtree = cKDTree(original_points)

            for i in range(start_idx, end_idx):
                point = plane_points[i]
                # 使用KD树进行高效最近邻搜索
                _, closest_idx = kdtree.query(point)
                chunk_indices.append(closest_idx)

            return start_idx, end_idx, chunk_indices

        # 执行并行搜索
        with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
            futures = [executor.submit(find_chunk_indices, chunk) for chunk in chunks]

            for future in futures:
                start_idx, end_idx, chunk_indices = future.result()
                original_indices[start_idx:end_idx] = chunk_indices

        elapsed_time = time.time() - start_time
        logger.debug(f"并行最近邻搜索完成: {n_points}个点，耗时{elapsed_time:.3f}秒")

        return original_indices