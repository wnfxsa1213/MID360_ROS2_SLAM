import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional
import logging
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
from .edge_enhancer import EdgeEnhancer

"""
几何约束优化算法

利用建筑物的几何规律性进行约束优化，包括：
- 平行约束（墙面平行）
- 垂直约束（墙面垂直于地面）
- 距离约束（标准房间尺寸）
- 角度约束（直角约束）
- 对称约束
- 共面约束
"""


logger = logging.getLogger(__name__)

class GeometryConstraint:
    """几何约束基类"""
    
    def __init__(self, constraint_type: str, weight: float = 1.0):
        self.constraint_type = constraint_type
        self.weight = weight
        
    def compute_error(self, points: np.ndarray) -> float:
        """计算约束误差"""
        raise NotImplementedError
        
    def compute_gradient(self, points: np.ndarray) -> np.ndarray:
        """计算约束梯度"""
        raise NotImplementedError

class ParallelConstraint(GeometryConstraint):
    """平行约束"""
    
    def __init__(self, plane_indices1: np.ndarray, plane_indices2: np.ndarray, 
                 target_angle: float = 0.0, weight: float = 1.0):
        super().__init__("parallel", weight)
        self.plane_indices1 = plane_indices1
        self.plane_indices2 = plane_indices2
        self.target_angle = target_angle
        
    def compute_error(self, points: np.ndarray) -> float:
        """计算平行约束误差"""
        # 过滤有效索引
        valid_indices1 = self.plane_indices1[self.plane_indices1 < len(points)]
        valid_indices2 = self.plane_indices2[self.plane_indices2 < len(points)]

        if len(valid_indices1) == 0 or len(valid_indices2) == 0:
            return 0.0

        # 计算两个平面的法向量
        points1 = points[valid_indices1]
        points2 = points[valid_indices2]
        
        if len(points1) < 3 or len(points2) < 3:
            return 0.0
            
        # 使用SVD计算平面法向量
        normal1 = self._compute_plane_normal(points1)
        normal2 = self._compute_plane_normal(points2)
        
        # 计算角度差异
        dot_product = np.clip(np.abs(np.dot(normal1, normal2)), 0, 1)
        angle = np.arccos(dot_product)
        
        error = (angle - self.target_angle) ** 2
        return error * self.weight
        
    def _compute_plane_normal(self, points: np.ndarray) -> np.ndarray:
        """计算平面法向量"""
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        
        # SVD分解
        U, S, Vt = np.linalg.svd(centered_points)
        normal = Vt[-1]  # 最小奇异值对应的向量
        
        return normal / np.linalg.norm(normal)

class PerpendicularConstraint(GeometryConstraint):
    """垂直约束"""
    
    def __init__(self, plane_indices1: np.ndarray, plane_indices2: np.ndarray, 
                 weight: float = 1.0):
        super().__init__("perpendicular", weight)
        self.plane_indices1 = plane_indices1
        self.plane_indices2 = plane_indices2
        
    def compute_error(self, points: np.ndarray) -> float:
        """计算垂直约束误差"""
        # 过滤有效索引
        valid_indices1 = self.plane_indices1[self.plane_indices1 < len(points)]
        valid_indices2 = self.plane_indices2[self.plane_indices2 < len(points)]

        if len(valid_indices1) == 0 or len(valid_indices2) == 0:
            return 0.0

        points1 = points[valid_indices1]
        points2 = points[valid_indices2]
        
        if len(points1) < 3 or len(points2) < 3:
            return 0.0
            
        normal1 = self._compute_plane_normal(points1)
        normal2 = self._compute_plane_normal(points2)
        
        # 垂直时点积应该为0
        dot_product = np.dot(normal1, normal2)
        error = dot_product ** 2
        
        return error * self.weight
        
    def _compute_plane_normal(self, points: np.ndarray) -> np.ndarray:
        """计算平面法向量"""
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        
        U, S, Vt = np.linalg.svd(centered_points)
        normal = Vt[-1]
        
        return normal / np.linalg.norm(normal)

class DistanceConstraint(GeometryConstraint):
    """距离约束"""
    
    def __init__(self, point_indices1: np.ndarray, point_indices2: np.ndarray,
                 target_distance: float, weight: float = 1.0):
        super().__init__("distance", weight)
        self.point_indices1 = point_indices1
        self.point_indices2 = point_indices2
        self.target_distance = target_distance
        
    def compute_error(self, points: np.ndarray) -> float:
        """计算距离约束误差"""
        # 过滤有效索引
        valid_indices1 = self.point_indices1[self.point_indices1 < len(points)]
        valid_indices2 = self.point_indices2[self.point_indices2 < len(points)]

        if len(valid_indices1) == 0 or len(valid_indices2) == 0:
            return 0.0

        points1 = points[valid_indices1]
        points2 = points[valid_indices2]
        
        # 计算两个点集的质心距离
        centroid1 = np.mean(points1, axis=0)
        centroid2 = np.mean(points2, axis=0)
        
        actual_distance = np.linalg.norm(centroid2 - centroid1)
        error = (actual_distance - self.target_distance) ** 2
        
        return error * self.weight

class RightAngleConstraint(GeometryConstraint):
    """直角约束"""
    
    def __init__(self, point_indices: np.ndarray, vertex_index: int, weight: float = 1.0):
        super().__init__("right_angle", weight)
        self.point_indices = point_indices
        self.vertex_index = vertex_index
        
    def compute_error(self, points: np.ndarray) -> float:
        """计算直角约束误差"""
        # 检查顶点索引有效性
        if self.vertex_index >= len(points):
            return 0.0

        # 过滤有效索引
        valid_point_indices = self.point_indices[self.point_indices < len(points)]

        if len(valid_point_indices) < 3:
            return 0.0

        # 获取顶点和相邻点
        vertex = points[self.vertex_index]
        adjacent_points = points[valid_point_indices]

        # 移除顶点自身
        adjacent_points = adjacent_points[valid_point_indices != self.vertex_index]
        
        if len(adjacent_points) < 2:
            return 0.0
            
        # 找到最近的两个点形成直角
        distances = np.linalg.norm(adjacent_points - vertex, axis=1)
        closest_indices = np.argsort(distances)[:2]
        
        p1, p2 = adjacent_points[closest_indices]
        
        # 计算两个向量
        v1 = p1 - vertex
        v2 = p2 - vertex
        
        # 计算夹角
        dot_product = np.dot(v1, v2)
        norms = np.linalg.norm(v1) * np.linalg.norm(v2)
        
        if norms < 1e-8:
            return 0.0
            
        cos_angle = dot_product / norms
        cos_angle = np.clip(cos_angle, -1, 1)
        
        # 直角的余弦值为0
        error = cos_angle ** 2
        
        return error * self.weight

class GeometryOptimizer:
    """几何约束优化器"""
    
    def __init__(self, config: dict):
        """
        初始化几何约束优化器
        
        Args:
            config: 算法配置字典
        """
        self.enable_parallel_constraint = config.get('enable_parallel_constraint', True)
        self.enable_perpendicular_constraint = config.get('enable_perpendicular_constraint', True)
        self.enable_distance_constraint = config.get('enable_distance_constraint', True)
        self.enable_angle_constraint = config.get('enable_angle_constraint', True)
        self.constraint_weight = config.get('constraint_weight', 0.5)
        
        # 内部状态
        self.constraints = []
        self.original_points = None
        
        logger.info("几何约束优化器初始化完成")
        
    def detect_geometric_relationships(self, pcd: o3d.geometry.PointCloud,
                                     planes: List) -> List[GeometryConstraint]:
        """
        检测几何关系并创建约束
        
        Args:
            pcd: 输入点云
            planes: 检测到的平面列表
            
        Returns:
            约束列表
        """
        logger.info("开始检测几何关系")
        
        constraints = []
        points = np.asarray(pcd.points)
        
        # 1. 创建平行约束
        if self.enable_parallel_constraint:
            parallel_constraints = self._create_parallel_constraints(planes)
            constraints.extend(parallel_constraints)
            
        # 2. 创建垂直约束
        if self.enable_perpendicular_constraint:
            perpendicular_constraints = self._create_perpendicular_constraints(planes)
            constraints.extend(perpendicular_constraints)
            
        # 3. 创建距离约束
        if self.enable_distance_constraint:
            distance_constraints = self._create_distance_constraints(planes)
            constraints.extend(distance_constraints)
            
        # 4. 创建直角约束
        if self.enable_angle_constraint:
            angle_constraints = self._create_right_angle_constraints(points)
            constraints.extend(angle_constraints)
            
        logger.info(f"检测到 {len(constraints)} 个几何约束")
        
        return constraints
        
    def _create_parallel_constraints(self, planes: List) -> List[ParallelConstraint]:
        """创建平行约束"""
        constraints = []
        
        # 分析平面法向量的相似性
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                plane1, plane2 = planes[i], planes[j]
                
                # 计算法向量角度差
                dot_product = np.abs(np.dot(plane1.normal, plane2.normal))
                angle = np.arccos(np.clip(dot_product, 0, 1))
                
                # 如果角度小于5度，认为应该平行
                if angle < np.radians(5.0):
                    constraint = ParallelConstraint(
                        plane1.indices, plane2.indices, 
                        target_angle=0.0, weight=self.constraint_weight
                    )
                    constraints.append(constraint)
                    
        logger.debug(f"创建了 {len(constraints)} 个平行约束")
        return constraints
        
    def _create_perpendicular_constraints(self, planes: List) -> List[PerpendicularConstraint]:
        """创建垂直约束"""
        constraints = []
        
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                plane1, plane2 = planes[i], planes[j]
                
                # 计算法向量角度
                dot_product = np.abs(np.dot(plane1.normal, plane2.normal))
                angle = np.arccos(np.clip(dot_product, 0, 1))
                
                # 如果角度接近90度，认为应该垂直
                if abs(angle - np.pi/2) < np.radians(10.0):
                    constraint = PerpendicularConstraint(
                        plane1.indices, plane2.indices,
                        weight=self.constraint_weight
                    )
                    constraints.append(constraint)
                    
        logger.debug(f"创建了 {len(constraints)} 个垂直约束")
        return constraints
        
    def _create_distance_constraints(self, planes: List) -> List[DistanceConstraint]:
        """创建距离约束"""
        constraints = []
        
        # 识别标准房间尺寸
        standard_distances = [2.4, 2.8, 3.0, 3.6, 4.0, 4.8, 6.0]  # 常见房间尺寸(米)
        
        for i in range(len(planes)):
            for j in range(i + 1, len(planes)):
                plane1, plane2 = planes[i], planes[j]
                
                # 计算平面间距离
                centroid1 = np.mean(plane1.points, axis=0)
                centroid2 = np.mean(plane2.points, axis=0)
                actual_distance = np.linalg.norm(centroid2 - centroid1)
                
                # 找到最接近的标准距离
                closest_std_distance = min(standard_distances, 
                                         key=lambda x: abs(x - actual_distance))
                
                # 如果差异不大，应用距离约束
                if abs(actual_distance - closest_std_distance) < 0.5:
                    constraint = DistanceConstraint(
                        plane1.indices, plane2.indices,
                        target_distance=closest_std_distance,
                        weight=self.constraint_weight * 0.5  # 距离约束权重较小
                    )
                    constraints.append(constraint)
                    
        logger.debug(f"创建了 {len(constraints)} 个距离约束")
        return constraints
        
    def _create_right_angle_constraints(self, points: np.ndarray) -> List[RightAngleConstraint]:
        """创建直角约束"""
        constraints = []
        
        # 检测潜在的角点
        # 这里简化为使用曲率检测
        
        # 使用默认配置检测边缘
        edge_config = {'curvature_threshold': 0.1, 'edge_search_radius': 0.05}
        edge_enhancer = EdgeEnhancer(edge_config)
        
        # 创建临时点云用于检测
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(points)
        
        # 检测高曲率点（潜在角点）
        curvatures = edge_enhancer.compute_curvature(temp_pcd)
        corner_candidates = np.where(curvatures > edge_config['curvature_threshold'])[0]

        # 确保索引在有效范围内
        max_valid_index = len(points) - 1
        corner_candidates = corner_candidates[corner_candidates <= max_valid_index]

        # 为每个候选角点创建直角约束
        for corner_idx in corner_candidates[:20]:  # 限制数量避免过多约束
            # 确保corner_idx在有效范围内
            if corner_idx >= len(points):
                continue

            # 找到邻近点
            corner_point = points[corner_idx]
            distances = np.linalg.norm(points - corner_point, axis=1)
            neighbor_indices = np.where(distances < 0.2)[0]  # 20cm邻域

            # 确保邻居索引也在有效范围内
            neighbor_indices = neighbor_indices[neighbor_indices < len(points)]
            
            if len(neighbor_indices) >= 3:  # 需要足够的邻近点
                constraint = RightAngleConstraint(
                    neighbor_indices, corner_idx,
                    weight=self.constraint_weight * 0.3  # 角度约束权重较小
                )
                constraints.append(constraint)
                
        logger.debug(f"创建了 {len(constraints)} 个直角约束")
        return constraints
        
    def compute_total_error(self, points: np.ndarray) -> float:
        """计算总约束误差"""
        total_error = 0.0
        
        for constraint in self.constraints:
            error = constraint.compute_error(points)
            total_error += error
            
        return total_error
        
    def optimize_geometry(self, pcd: o3d.geometry.PointCloud,
                         planes: List, max_iterations: int = 20) -> Tuple[o3d.geometry.PointCloud, Dict]:
        """
        执行几何约束优化
        
        Args:
            pcd: 输入点云
            planes: 检测到的平面列表
            max_iterations: 最大迭代次数
            
        Returns:
            (优化后的点云, 优化信息)
        """
        logger.info("开始几何约束优化")
        
        self.original_points = np.asarray(pcd.points).copy()
        
        # 1. 检测几何关系并创建约束
        self.constraints = self.detect_geometric_relationships(pcd, planes)
        
        if not self.constraints:
            logger.warning("未检测到几何约束，跳过优化")
            return pcd, {'constraints': 0, 'iterations': 0, 'initial_error': 0, 'final_error': 0}
            
        # 2. 计算初始误差
        initial_error = self.compute_total_error(self.original_points)
        
        # 3. 迭代优化
        current_points = self.original_points.copy()
        previous_error = initial_error

        logger.info(f"开始迭代优化，约束数量: {len(self.constraints)}, 初始误差: {initial_error:.6f}")

        for iteration in range(max_iterations):
            # 计算约束力
            total_force = np.zeros_like(current_points)

            logger.debug(f"迭代 {iteration+1}: 计算约束力...")

            for i, constraint in enumerate(self.constraints):
                if (i + 1) % 20 == 0:  # 每20个约束报告一次进度
                    logger.debug(f"  处理约束 {i+1}/{len(self.constraints)}")

                force = self._compute_constraint_force(constraint, current_points)
                total_force += force

            # 应用约束力（小步长）
            step_size = 0.001  # 1mm步长
            current_points += step_size * total_force

            # 检查收敛性
            current_error = self.compute_total_error(current_points)

            logger.info(f"迭代 {iteration+1}: 约束误差 = {current_error:.6f}")

            # 如果误差增加或变化很小，停止优化
            if iteration > 0 and (current_error > previous_error * 1.01 or
                                 abs(current_error - previous_error) < 1e-8):
                logger.info(f"优化在第 {iteration+1} 次迭代收敛")
                break

            previous_error = current_error
            
        # 4. 创建优化后的点云
        optimized_pcd = o3d.geometry.PointCloud()
        optimized_pcd.points = o3d.utility.Vector3dVector(current_points)
        
        # 复制原始属性
        if pcd.has_colors():
            optimized_pcd.colors = pcd.colors
        if pcd.has_normals():
            optimized_pcd.normals = pcd.normals
            
        final_error = self.compute_total_error(current_points)
        
        optimization_info = {
            'constraints': len(self.constraints),
            'iterations': iteration + 1,
            'initial_error': initial_error,
            'final_error': final_error,
            'error_reduction': (initial_error - final_error) / initial_error if initial_error > 0 else 0,
            'max_displacement': np.max(np.linalg.norm(current_points - self.original_points, axis=1))
        }
        
        logger.info(f"几何约束优化完成: 约束数={optimization_info['constraints']}, "
                   f"迭代次数={optimization_info['iterations']}, "
                   f"误差减少={optimization_info['error_reduction']:.2%}, "
                   f"最大位移={optimization_info['max_displacement']:.3f}m")
                   
        return optimized_pcd, optimization_info
        
    def _compute_constraint_force(self, constraint: GeometryConstraint, 
                                points: np.ndarray) -> np.ndarray:
        """
        计算约束力（简化的数值梯度）
        
        Args:
            constraint: 约束对象
            points: 当前点坐标
            
        Returns:
            约束力向量
        """
        force = np.zeros_like(points)
        epsilon = 1e-4  # 数值微分步长
        
        # 根据约束类型计算力
        if hasattr(constraint, 'plane_indices1') and hasattr(constraint, 'plane_indices2'):
            # 平面约束
            affected_indices = np.concatenate([constraint.plane_indices1, constraint.plane_indices2])
        elif hasattr(constraint, 'point_indices1') and hasattr(constraint, 'point_indices2'):
            # 距离约束
            affected_indices = np.concatenate([constraint.point_indices1, constraint.point_indices2])
        elif hasattr(constraint, 'point_indices'):
            # 角度约束
            affected_indices = constraint.point_indices
        else:
            return force

        # 确保所有索引都在有效范围内
        affected_indices = affected_indices[affected_indices < len(points)]
            
        # 限制受影响点的数量以提高性能
        max_points_per_constraint = 100  # 限制每个约束最多影响100个点
        if len(affected_indices) > max_points_per_constraint:
            # 随机采样或取前N个点
            np.random.seed(42)  # 固定随机种子保证可重复性
            affected_indices = np.random.choice(affected_indices, max_points_per_constraint, replace=False)

        # 只计算一次当前误差
        current_error = constraint.compute_error(points)

        # 对受影响的点计算数值梯度（优化版本）
        for i in affected_indices:
            if i >= len(points):
                continue

            # 计算每个坐标方向的梯度
            for dim in range(3):
                # 只修改单个点而不是复制整个数组
                original_value = points[i, dim]
                points[i, dim] += epsilon
                error_plus = constraint.compute_error(points)
                points[i, dim] = original_value  # 恢复原值

                gradient = (error_plus - current_error) / epsilon
                force[i, dim] -= gradient * constraint.weight  # 负梯度方向
                
        return force