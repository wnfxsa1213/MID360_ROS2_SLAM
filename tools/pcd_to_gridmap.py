#!/usr/bin/env python3
"""
PCD地图转栅格地图工具
将保存的PCD点云地图转换为2D栅格地图（占用栅格地图）
增强版本：充分利用MID360全景扫描特性
"""

import numpy as np
import open3d as o3d
import cv2
import yaml
import argparse
from pathlib import Path
from collections import defaultdict

class PCDToGridMap:
    def __init__(self, resolution=0.05, ground_height=(-0.3, 0.3),
                 ceiling_height=(1.8, 3.0), robot_height=0.8,
                 obstacle_threshold=3, vertical_continuity_threshold=0.8,
                 panoramic_analysis=True,
                 inflate_radius=0.0,
                 use_vertical_structures=False,
                 require_vertical_for_obstacle=True,
                 ground_dominates_ratio=2.0):
        """
        初始化PCD到栅格地图转换器（增强版本：利用MID360全景扫描）

        Args:
            resolution: 栅格分辨率 (米/像素)
            ground_height: 地面高度范围 (min_height, max_height)
            ceiling_height: 天花板高度范围 (min_height, max_height)
            robot_height: 机器人通行高度
            obstacle_threshold: 障碍物点云密度阈值
            vertical_continuity_threshold: 垂直连续性阈值
            panoramic_analysis: 是否启用全景分析
        """
        self.resolution = resolution
        self.ground_height = ground_height
        self.ceiling_height = ceiling_height
        self.robot_height = robot_height
        self.obstacle_threshold = obstacle_threshold
        self.vertical_continuity_threshold = vertical_continuity_threshold
        self.panoramic_analysis = panoramic_analysis
        self.inflate_radius = inflate_radius
        self.use_vertical_structures = use_vertical_structures
        self.require_vertical_for_obstacle = require_vertical_for_obstacle
        self.ground_dominates_ratio = ground_dominates_ratio

        # 多层高度分析参数
        self.height_layers = {
            'ground': ground_height,
            'robot': (-0.1, robot_height + 0.2),
            'overhead': (robot_height + 0.2, ceiling_height[0]),
            'ceiling': ceiling_height
        }

    def load_pcd(self, pcd_file):
        """加载PCD文件"""
        print(f"正在加载PCD文件: {pcd_file}")
        try:
            pcd = o3d.io.read_point_cloud(str(pcd_file))
            points = np.asarray(pcd.points)
            print(f"加载了 {len(points)} 个点")
            return points
        except Exception as e:
            print(f"加载PCD文件失败: {e}")
            return None

    def preprocess_points(self, points):
        """可选的点云预处理：体素下采样 + 统计离群去噪"""
        if not getattr(self, 'enable_pre_filter', False):
            return points

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 体素下采样
        voxel = float(getattr(self, 'voxel_size', 0.0) or 0.0)
        if voxel > 0.0:
            pcd = pcd.voxel_down_sample(voxel)

        # 统计离群去噪
        mean_k = int(getattr(self, 'sor_mean_k', 0) or 0)
        if mean_k > 0:
            std_ratio = float(getattr(self, 'sor_std_ratio', 1.0) or 1.0)
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=mean_k, std_ratio=std_ratio)
            pcd = pcd.select_by_index(ind)

        filtered = np.asarray(pcd.points)
        print(f"预处理后剩余 {len(filtered)} 个点 (原始 {len(points)})")
        return filtered

    @staticmethod
    def apply_yaw(points: np.ndarray, yaw_deg: float) -> np.ndarray:
        """绕Z轴旋转点云 yaw_deg（度）"""
        if yaw_deg is None:
            return points
        yaw = np.deg2rad(float(yaw_deg))
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        return points @ R.T

    @staticmethod
    def estimate_yaw_pca(points_xy: np.ndarray) -> float:
        """基于XY PCA估计主方向，与x轴对齐所需的旋转角度（度）"""
        if points_xy.shape[0] < 10:
            return 0.0
        # 去均值
        mu = np.mean(points_xy, axis=0)
        X = points_xy - mu
        C = np.cov(X.T)
        eigvals, eigvecs = np.linalg.eigh(C)
        v = eigvecs[:, np.argmax(eigvals)]  # 主方向
        angle = np.arctan2(v[1], v[0])  # 向x轴的夹角
        return -np.rad2deg(angle)  # 需要旋转的角度（度）

    @staticmethod
    def _bresenham_line(x0, y0, x1, y1):
        """整数栅格的Bresenham直线生成器"""
        x0 = int(x0); y0 = int(y0); x1 = int(x1); y1 = int(y1)
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def ray_carve_free(self, occupancy_grid: np.ndarray, origin_x: float, origin_y: float,
                        poses_xy: np.ndarray, max_range: float = 30.0,
                        angle_step_deg: float = 2.0, pose_stride: int = 10):
        """从位姿发射射线，将命中前的未知/自由格强制设为自由，遇障碍即停止"""
        if poses_xy is None or poses_xy.shape[0] == 0:
            return occupancy_grid
        h, w = occupancy_grid.shape
        step_pix = 1
        res = self.resolution
        max_pix = int(max_range / res) if max_range > 0 else max(h, w)
        angles = np.deg2rad(np.arange(0.0, 360.0, max(0.1, angle_step_deg)))
        for i in range(0, poses_xy.shape[0], max(1, pose_stride)):
            px = (poses_xy[i, 0] - origin_x) / res
            py = (poses_xy[i, 1] - origin_y) / res
            if not (0 <= px < w and 0 <= py < h):
                continue
            for ang in angles:
                dx = np.cos(ang)
                dy = np.sin(ang)
                # 目标点
                tx = int(px + dx * max_pix)
                ty = int(py + dy * max_pix)
                for x, y in self._bresenham_line(px, py, tx, ty):
                    if not (0 <= x < w and 0 <= y < h):
                        break
                    if occupancy_grid[y, x] == 100:
                        break
                    # 清理未知/自由为自由
                    occupancy_grid[y, x] = 0
        return occupancy_grid

    def ray_carve_by_returns(self, occupancy_grid: np.ndarray, poses_xy: np.ndarray,
                              max_range: float, angle_step_deg: float,
                              pose_stride: int) -> np.ndarray:
        """按观测回波进行雕刻：以有观测的格作为目标端点，沿线清理"""
        if poses_xy is None or poses_xy.shape[0] == 0:
            return occupancy_grid
        if not hasattr(self, '_last_density_maps') or not hasattr(self, '_last_grid_meta'):
            return occupancy_grid
        dm = self._last_density_maps
        meta = self._last_grid_meta
        h, w = occupancy_grid.shape
        res = self.resolution
        ox, oy = meta['origin_x'], meta['origin_y']

        targets = np.zeros((h, w), dtype=np.uint8)
        if 'robot' in dm:
            targets |= (dm['robot'] > 0).astype(np.uint8)
        if 'ground' in dm:
            targets |= (dm['ground'] > 0).astype(np.uint8)
        ty, tx = np.where(targets > 0)
        if ty.size == 0:
            return occupancy_grid
        tgt_coords = np.stack([tx, ty], axis=1)

        max_pix = int(max_range / res) if max_range > 0 else max(h, w)
        bin_deg = max(0.1, angle_step_deg)
        for i in range(0, poses_xy.shape[0], max(1, pose_stride)):
            px = (poses_xy[i, 0] - ox) / res
            py = (poses_xy[i, 1] - oy) / res
            if not (0 <= px < w and 0 <= py < h):
                continue
            used_bins = set()
            dx = tgt_coords[:, 0] - px
            dy = tgt_coords[:, 1] - py
            dist2 = dx*dx + dy*dy
            order = np.argsort(dist2)
            limit = min(len(order), int(360.0 / bin_deg) + 4)
            for idx in order[:limit]:
                tx1, ty1 = int(tgt_coords[idx, 0]), int(tgt_coords[idx, 1])
                ang = np.rad2deg(np.arctan2(ty1 - py, tx1 - px)) % 360.0
                b = int(ang // bin_deg)
                if b in used_bins:
                    continue
                used_bins.add(b)
                for x, y in self._bresenham_line(px, py, tx1, ty1):
                    if not (0 <= x < w and 0 <= y < h):
                        break
                    if occupancy_grid[y, x] == 100:
                        break
                    occupancy_grid[y, x] = 0
        return occupancy_grid

    @staticmethod
    def _quat_to_rot(qw, qx, qy, qz):
        """四元数转旋转矩阵 (w, x, y, z)"""
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm == 0:
            return np.eye(3)
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)
        return R

    def assemble_from_patches(self, poses_file: Path, patches_dir: Path):
        """从 PGO patches + poses.txt 组装全局点云

        poses.txt 每行: patch_name.pcd tx ty tz qw qx qy qz
        patches 为 PCL PointXYZI，仅使用 x,y,z
        """
        if not poses_file.exists():
            print(f"错误: poses文件不存在: {poses_file}")
            return None
        if not patches_dir.exists():
            print(f"错误: patches目录不存在: {patches_dir}")
            return None

        all_pts = []
        loaded = 0
        with open(poses_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) != 8:
                    print(f"警告: poses行格式不正确，跳过: {line}")
                    continue
                name, tx, ty, tz, qw, qx, qy, qz = parts[0], *map(float, parts[1:])
                patch_path = patches_dir / name
                if not patch_path.exists():
                    print(f"警告: 未找到patch: {patch_path}")
                    continue
                try:
                    p = o3d.io.read_point_cloud(str(patch_path))
                    pts = np.asarray(p.points)
                    if pts.size == 0:
                        continue
                    R = self._quat_to_rot(qw, qx, qy, qz)
                    t = np.array([tx, ty, tz], dtype=np.float64)
                    pts_w = (pts @ R.T) + t
                    all_pts.append(pts_w)
                    loaded += 1
                except Exception as e:
                    print(f"警告: 加载或变换patch失败 {patch_path}: {e}")
                    continue

        if not all_pts:
            print("错误: 未能从patches组装出点云")
            return None

        cat = np.vstack(all_pts)
        print(f"已从 {loaded} 个patch组装点云，共 {len(cat)} 点")
        return cat

    @staticmethod
    def load_hba_poses(hba_file: Path):
        """加载HBA轨迹: 每行 tx ty tz qw qx qy qz"""
        if not hba_file or not hba_file.exists():
            return None
        arr = []
        with open(hba_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) != 7:
                    print(f"警告: HBA行格式不正确，跳过: {line}")
                    continue
                tx, ty, tz, qw, qx, qy, qz = map(float, parts)
                arr.append((tx, ty, tz, qw, qx, qy, qz))
        if not arr:
            return None
        return np.array(arr, dtype=np.float64)

    def analyze_height_layers(self, points):
        """多层高度分析：利用MID360全景扫描特性"""
        layer_points = {}
        layer_stats = {}

        for layer_name, (min_h, max_h) in self.height_layers.items():
            layer_filter = (points[:, 2] >= min_h) & (points[:, 2] <= max_h)
            layer_points[layer_name] = points[layer_filter]
            layer_stats[layer_name] = {
                'count': len(layer_points[layer_name]),
                'height_range': (min_h, max_h),
                'percentage': len(layer_points[layer_name]) / len(points) * 100
            }

        print(f"多层高度分析结果:")
        for layer_name, stats in layer_stats.items():
            print(f"  {layer_name}: {stats['count']} 点 ({stats['percentage']:.1f}%) [{stats['height_range'][0]:.1f}m ~ {stats['height_range'][1]:.1f}m]")

        return layer_points, layer_stats

    @staticmethod
    def _skeletonize_binary(binary_img: np.ndarray) -> np.ndarray:
        """形态学骨架提取（binary为0/1）"""
        img = (binary_img.copy() * 255).astype(np.uint8)
        skel = np.zeros_like(img)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while True:
            opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
            temp = cv2.subtract(img, opened)
            eroded = cv2.erode(img, element)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded
            if cv2.countNonZero(img) == 0:
                break
        return (skel > 0).astype(np.uint8)

    def detect_vertical_structures(self, points):
        """检测垂直结构（墙面、柱子等）"""
        if not self.use_vertical_structures:
            return np.array([])

        # 计算每个点在XY平面上的栅格坐标
        grid_x = ((points[:, 0] - np.min(points[:, 0])) / self.resolution).astype(int)
        grid_y = ((points[:, 1] - np.min(points[:, 1])) / self.resolution).astype(int)

        # 统计每个栅格的高度分布
        grid_height_data = defaultdict(list)
        for i in range(len(points)):
            grid_key = (grid_x[i], grid_y[i])
            grid_height_data[grid_key].append(points[i, 2])

        vertical_structure_points = []

        for grid_key, heights in grid_height_data.items():
            if len(heights) < self.obstacle_threshold:
                continue

            heights = np.array(heights)
            height_span = np.max(heights) - np.min(heights)

            # 判断是否为垂直结构（高度跨度大于机器人高度）
            if height_span > self.robot_height * self.vertical_continuity_threshold:
                # 这个栅格包含垂直结构
                grid_indices = (grid_x == grid_key[0]) & (grid_y == grid_key[1])
                vertical_structure_points.extend(points[grid_indices])

        return np.array(vertical_structure_points) if vertical_structure_points else np.array([])

    def filter_navigable_points(self, points):
        """过滤出影响机器人导航的点云"""
        # 机器人高度范围内的点
        robot_level_filter = (points[:, 2] >= self.height_layers['robot'][0]) & \
                           (points[:, 2] <= self.height_layers['robot'][1])

        # 地面附近的点（用于判断可通行性）
        ground_level_filter = (points[:, 2] >= self.ground_height[0]) & \
                             (points[:, 2] <= self.ground_height[1])

        navigable_points = points[robot_level_filter | ground_level_filter]
        print(f"导航相关点云: {len(navigable_points)} 个点")

        return navigable_points

    def points_to_grid(self, points):
        """将点云转换为栅格坐标"""
        # 计算边界
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])

        # 添加边界缓冲
        buffer = 2.0  # 2米缓冲
        min_x -= buffer
        max_x += buffer
        min_y -= buffer
        max_y += buffer

        # 计算栅格尺寸（向上取整，避免裁剪边界）
        grid_width = int(np.ceil((max_x - min_x) / self.resolution))
        grid_height = int(np.ceil((max_y - min_y) / self.resolution))

        print(f"地图边界: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}]")
        print(f"栅格尺寸: {grid_width} x {grid_height}")

        # 转换点云到栅格坐标
        grid_x = ((points[:, 0] - min_x) / self.resolution).astype(int)
        grid_y = ((points[:, 1] - min_y) / self.resolution).astype(int)

        # 确保坐标在有效范围内
        valid_indices = (grid_x >= 0) & (grid_x < grid_width) & (grid_y >= 0) & (grid_y < grid_height)
        grid_x = grid_x[valid_indices]
        grid_y = grid_y[valid_indices]

        return grid_x, grid_y, grid_width, grid_height, min_x, min_y

    def create_occupancy_grid(self, points):
        """创建增强占用栅格地图（利用MID360全景扫描特性）"""
        # 多层高度分析
        layer_points, layer_stats = self.analyze_height_layers(points)

        # 检测垂直结构
        vertical_structures = self.detect_vertical_structures(points)

        # 过滤导航相关点云
        navigable_points = self.filter_navigable_points(points)
        if len(navigable_points) == 0:
            print("警告: 没有找到导航相关点云")
            return None, None

        # 转换到栅格坐标
        grid_x, grid_y, grid_width, grid_height, origin_x, origin_y = self.points_to_grid(navigable_points)

        # 计算全高度跨度图（使用同一原点/尺寸）
        gx_all = ((points[:, 0] - origin_x) / self.resolution).astype(int)
        gy_all = ((points[:, 1] - origin_y) / self.resolution).astype(int)
        valid_all = (gx_all >= 0) & (gx_all < grid_width) & (gy_all >= 0) & (gy_all < grid_height)
        gx_all = gx_all[valid_all]
        gy_all = gy_all[valid_all]
        z_all = points[:, 2][valid_all]

        height_min = np.full((grid_height, grid_width), np.inf, dtype=np.float32)
        height_max = np.full((grid_height, grid_width), -np.inf, dtype=np.float32)
        for x, y, z in zip(gx_all, gy_all, z_all):
            if z < height_min[y, x]:
                height_min[y, x] = z
            if z > height_max[y, x]:
                height_max[y, x] = z
        height_span_map = height_max - height_min
        height_span_map[~np.isfinite(height_span_map)] = 0.0

        # 创建多层密度地图（统一使用导航网格的原点与尺寸，避免坐标错位）
        density_maps = {}
        for layer_name, points_layer in layer_points.items():
            if len(points_layer) == 0:
                continue

            # 使用与导航网格相同的原点进行索引换算
            layer_grid_x = ((points_layer[:, 0] - origin_x) / self.resolution).astype(int)
            layer_grid_y = ((points_layer[:, 1] - origin_y) / self.resolution).astype(int)
            density_map = np.zeros((grid_height, grid_width), dtype=np.int32)

            # 统计每个栅格的点云数量（向量化累加）
            valid_indices = (layer_grid_x >= 0) & (layer_grid_x < grid_width) & \
                           (layer_grid_y >= 0) & (layer_grid_y < grid_height)
            lx = layer_grid_x[valid_indices]
            ly = layer_grid_y[valid_indices]
            if lx.size > 0:
                np.add.at(density_map, (ly, lx), 1)

            density_maps[layer_name] = density_map

        # 创建增强占用栅格地图
        occupancy_grid = self.create_enhanced_occupancy_grid(density_maps, grid_width, grid_height, height_span_map)

        # 保存统计与网格元数据，供射线雕刻按回波使用
        self._last_density_maps = density_maps
        self._last_grid_meta = {
            'origin_x': origin_x,
            'origin_y': origin_y,
            'width': grid_width,
            'height': grid_height
        }

        # 处理垂直结构
        if len(vertical_structures) > 0:
            occupancy_grid = self.apply_vertical_structure_constraints(occupancy_grid, vertical_structures, origin_x, origin_y)

        print(f"增强占用栅格统计:")
        print(f"  障碍物栅格: {np.sum(occupancy_grid == 100)}")
        print(f"  自由空间栅格: {np.sum(occupancy_grid == 0)}")
        print(f"  未知区域栅格: {np.sum((occupancy_grid != 0) & (occupancy_grid != 100))}")

        # 创建元数据
        metadata = {
            'resolution': self.resolution,
            'origin': [origin_x, origin_y, 0.0],
            'width': grid_width,
            'height': grid_height,
            'ground_height': self.ground_height,
            'ceiling_height': self.ceiling_height,
            'robot_height': self.robot_height,
            'obstacle_threshold': self.obstacle_threshold,
            'vertical_continuity_threshold': self.vertical_continuity_threshold,
            'panoramic_analysis': self.panoramic_analysis,
            'layer_stats': layer_stats,
            'vertical_structures_count': len(vertical_structures)
        }

        return occupancy_grid, metadata

    def create_enhanced_occupancy_grid(self, density_maps, grid_width, grid_height, height_span_map):
        """创建增强占用栅格地图（多层分析）"""
        occupancy_grid = np.full((grid_height, grid_width), -1, dtype=np.int8)  # 默认未知

        # 地面层分析：判断可通行性
        if 'ground' in density_maps:
            ground_density = density_maps['ground']
            # 地面点云较少的区域认为可通行
            free_ground_mask = (ground_density > 0) & (ground_density < self.obstacle_threshold)
            occupancy_grid[free_ground_mask] = 0  # 自由空间

        # 机器人高度层分析：主要障碍物检测
        if 'robot' in density_maps:
            robot_density = density_maps['robot']
            # 约束：需要有明显垂直高度跨度，避免走廊中线被误判
            if self.require_vertical_for_obstacle:
                vertical_mask = height_span_map > (self.robot_height * self.vertical_continuity_threshold)
                obstacle_mask = (robot_density >= self.obstacle_threshold) & vertical_mask
            else:
                obstacle_mask = (robot_density >= self.obstacle_threshold)
            occupancy_grid[obstacle_mask] = 100  # 障碍物

        # 顶部和天花板层分析：辅助判断
        overhead_obstacle_mask = np.zeros((grid_height, grid_width), dtype=bool)
        for layer_name in ['overhead', 'ceiling']:
            if layer_name in density_maps:
                layer_density = density_maps[layer_name]
                overhead_obstacle_mask |= (layer_density >= self.obstacle_threshold)

        # 在机器人高度层没有障碍物但上方有障碍物的区域，保留为未知（避免误判遮挡物）
        # 可选：如果需要，可在此处标注低置信障碍（值50），但默认保持未知
        # potential_low_obstacle = overhead_obstacle_mask & (occupancy_grid == -1)
        # occupancy_grid[potential_low_obstacle] = 50

        # 增强自由空间推理：利用全景扫描特性
        if self.panoramic_analysis:
            occupancy_grid = self.enhance_free_space_inference(occupancy_grid, density_maps)

        # 地面主导时强制自由（缓解走廊内部变黑）
        if 'ground' in density_maps and 'robot' in density_maps:
            ground_density = density_maps['ground']
            robot_density = density_maps['robot']
            dominance = (ground_density >= (self.ground_dominates_ratio * (robot_density + 1))) & (occupancy_grid != 100)
            occupancy_grid[dominance] = 0

        # 可选：根据车辆尺寸对障碍物进行膨胀，提升导航安全裕度
        if self.inflate_radius and self.inflate_radius > 0.0:
            pix_radius = max(1, int(np.round(self.inflate_radius / self.resolution)))
            kernel_size = 2 * pix_radius + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            obstacle_binary = (occupancy_grid == 100).astype(np.uint8)

            # 门洞保护：在膨胀前计算到原始障碍的距离变换（像素）
            min_door_width = float(getattr(self, 'min_door_width', 0.0) or 0.0)
            dist_map = None
            free_pre = (occupancy_grid == 0).astype(np.uint8)
            skel_pre = None
            if min_door_width > 0.0:
                # 前景为非障碍(1)，背景为障碍(0)，计算到障碍的距离
                dt_input = np.where(obstacle_binary == 1, 0, 1).astype(np.uint8)
                dist_map = cv2.distanceTransform(dt_input, cv2.DIST_L2, 3)
                min_half_pix = max(1, int(np.ceil((min_door_width / 2.0) / self.resolution)))
                # 连通性约束：仅保护自由空间骨架附近，避免打开薄墙缝隙
                skel_pre = self._skeletonize_binary(free_pre)

            inflated = cv2.dilate(obstacle_binary, kernel, iterations=1)

            if dist_map is not None:
                # 保护宽于阈值的通道：在这些区域不允许膨胀侵占
                protect_mask = dist_map >= min_half_pix
                # 仅恢复被膨胀新置为障碍且属于保护区域的像素
                newly_inflated = (inflated == 1) & (obstacle_binary == 0)
                to_restore = newly_inflated & protect_mask
                if skel_pre is not None:
                    k = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
                    skel_near = cv2.dilate(skel_pre, k, iterations=1).astype(bool)
                    to_restore = to_restore & skel_near
                inflated[to_restore] = 0

            occupancy_grid[inflated == 1] = 100

        # 可选：障碍开运算与小连通域清理，减少走廊内部黑点/毛刺
        obstacle_open_iters = getattr(self, 'obstacle_open_iters', 0)
        min_obstacle_area = getattr(self, 'min_obstacle_area', 0)
        if obstacle_open_iters > 0 or (min_obstacle_area and min_obstacle_area > 0):
            obs = (occupancy_grid == 100).astype(np.uint8)
            if obstacle_open_iters > 0:
                k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                obs = cv2.morphologyEx(obs, cv2.MORPH_OPEN, k, iterations=int(obstacle_open_iters))

            if min_obstacle_area and min_obstacle_area > 0:
                num, labels = cv2.connectedComponents(obs, connectivity=8)
                areas = np.bincount(labels.ravel())
                keep = np.zeros(num, dtype=bool)  # 0: 背景，默认不保留
                for i in range(1, num):
                    if areas[i] >= int(min_obstacle_area):
                        keep[i] = True
                obs = np.where(labels > 0, keep[labels].astype(np.uint8), 0)

            # 将清理后的障碍写回
            prev_obstacles = (occupancy_grid == 100)
            occupancy_grid[obs == 1] = 100
            # 对被清理掉的原障碍点设为未知（更保守），由后续策略决定是否变为自由
            cleared = (obs == 0) & prev_obstacles
            occupancy_grid[cleared] = -1

        return occupancy_grid

    def enhance_free_space_inference(self, occupancy_grid, density_maps):
        """增强自由空间推理：利用MID360全景扫描特性"""
        grid_height, grid_width = occupancy_grid.shape

        # 基于地面层和机器人层的空间推理
        if 'ground' in density_maps and 'robot' in density_maps:
            ground_density = density_maps['ground']
            robot_density = density_maps['robot']

            # 地面和机器人高度都没有障碍物的区域，很可能是自由空间
            potential_free = (ground_density < self.obstacle_threshold) & \
                           (robot_density < self.obstacle_threshold) & \
                           (occupancy_grid == -1)

            # 但需要有一些点云数据支持（说明扫描到了这个区域）
            has_scan_data = (ground_density > 0) | (robot_density > 0)
            confident_free = potential_free & has_scan_data

            occupancy_grid[confident_free] = 0  # 设为自由空间

        # 对自由空间进行形态学处理，填补小空洞
        free_space_mask = (occupancy_grid == 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # 闭运算：填补自由空间中的小空洞
        free_space_closed = cv2.morphologyEx(free_space_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 更新自由空间
        occupancy_grid[free_space_closed == 1] = 0

        return occupancy_grid

    def apply_vertical_structure_constraints(self, occupancy_grid, vertical_structures, origin_x, origin_y):
        """应用垂直结构约束"""
        if len(vertical_structures) == 0:
            return occupancy_grid

        # 将垂直结构点转换为栅格坐标
        vs_grid_x = ((vertical_structures[:, 0] - origin_x) / self.resolution).astype(int)
        vs_grid_y = ((vertical_structures[:, 1] - origin_y) / self.resolution).astype(int)

        # 确保坐标在有效范围内
        valid_indices = (vs_grid_x >= 0) & (vs_grid_x < occupancy_grid.shape[1]) & \
                       (vs_grid_y >= 0) & (vs_grid_y < occupancy_grid.shape[0])

        vs_grid_x = vs_grid_x[valid_indices]
        vs_grid_y = vs_grid_y[valid_indices]

        # 将垂直结构位置标记为障碍物
        for i in range(len(vs_grid_x)):
            x, y = vs_grid_x[i], vs_grid_y[i]
            occupancy_grid[y, x] = 100

        print(f"应用垂直结构约束: {len(vs_grid_x)} 个栅格")
        return occupancy_grid

    def save_occupancy_grid(self, occupancy_grid, metadata, output_file):
        """保存占用栅格地图为PGM格式"""
        if occupancy_grid is None:
            print("错误: 占用栅格地图为空，无法保存")
            return False

        # 转换为PGM格式的图像数据
        # PGM格式: 0=黑色(障碍物), 255=白色(自由空间), 127=灰色(未知)
        pgm_data = np.full_like(occupancy_grid, 127, dtype=np.uint8)  # 默认灰色(未知)
        pgm_data[occupancy_grid == 0] = 255     # 自由空间 -> 白色
        # 将低置信障碍（50）视为未知，避免与trinary阈值冲突
        pgm_data[occupancy_grid == 100] = 0     # 障碍物 -> 黑色

        # 为兼容ROS map_server 的原点定义，垂直翻转图像（使行0对应地图底部）
        pgm_to_save = np.flipud(pgm_data)

        # 保存PGM文件
        pgm_file = output_file.with_suffix('.pgm')
        success = cv2.imwrite(str(pgm_file), pgm_to_save)

        if success:
            print(f"栅格地图已保存为: {pgm_file}")

            # 保存YAML元数据文件
            yaml_file = output_file.with_suffix('.yaml')
            yaml_data = {
                'image': pgm_file.name,
                'resolution': float(metadata['resolution']),
                'origin': [float(x) for x in metadata['origin']],
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'negate': 0,
                'width': int(metadata['width']),
                'height': int(metadata['height']),
                'mode': 'trinary',
                'enhanced_panoramic': True,
                'mid360_optimized': True
            }

            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)

            print(f"地图元数据已保存为: {yaml_file}")

            # 保存详细信息
            info_file = output_file.with_suffix('.info')
            with open(info_file, 'w', encoding='utf-8') as f:
                f.write("PCD到栅格地图转换信息（增强版本）\n")
                f.write("=" * 50 + "\n")
                f.write(f"分辨率: {metadata['resolution']} 米/像素\n")
                f.write(f"地图尺寸: {metadata['width']} x {metadata['height']} 像素\n")
                f.write(f"地图原点: ({metadata['origin'][0]:.2f}, {metadata['origin'][1]:.2f})\n")
                f.write(f"地面高度范围: {metadata['ground_height'][0]:.1f} ~ {metadata['ground_height'][1]:.1f} 米\n")
                f.write(f"天花板高度范围: {metadata['ceiling_height'][0]:.1f} ~ {metadata['ceiling_height'][1]:.1f} 米\n")
                f.write(f"机器人通行高度: {metadata['robot_height']:.1f} 米\n")
                f.write(f"障碍物阈值: {metadata['obstacle_threshold']} 点/栅格\n")
                f.write(f"垂直连续性阈值: {metadata['vertical_continuity_threshold']:.2f}\n")
                f.write(f"全景分析: {'启用' if metadata['panoramic_analysis'] else '禁用'}\n")
                f.write(f"垂直结构数量: {metadata['vertical_structures_count']}\n")
                f.write(f"\n多层高度分析统计:\n")
                for layer_name, stats in metadata['layer_stats'].items():
                    f.write(f"  {layer_name}: {stats['count']} 点 ({stats['percentage']:.1f}%)\n")
                f.write(f"\n栅格统计:\n")
                f.write(f"  障碍物栅格: {np.sum(occupancy_grid == 100)}\n")
                f.write(f"  自由空间栅格: {np.sum(occupancy_grid == 0)}\n")
                f.write(f"  低信心度障碍物: {np.sum(occupancy_grid == 50)}\n")
                f.write(f"  未知区域栅格: {np.sum(occupancy_grid == -1)}\n")

            # 若存在雕刻前指标，输出到info并写JSON（便于GUI读取）
            try:
                pre = metadata.get('metrics_pre_carve')
                metrics_json = output_file.with_name(output_file.stem + '_metrics.json')
                post = {
                    'obstacles': int(np.sum(occupancy_grid == 100)),
                    'free': int(np.sum(occupancy_grid == 0)),
                    'unknown': int(np.sum(occupancy_grid == -1))
                }
                if pre is not None:
                    with open(info_file, 'a', encoding='utf-8') as f:
                        f.write("\n射线雕刻前后统计对比:\n")
                        f.write(f"  之前 - 障碍:{pre['obstacles']}, 自由:{pre['free']}, 未知:{pre['unknown']}\n")
                        f.write(f"  之后 - 障碍:{post['obstacles']}, 自由:{post['free']}, 未知:{post['unknown']}\n")
                # JSON保存
                import json as _json
                data_json = {
                    'resolution': float(metadata['resolution']),
                    'width': int(metadata['width']),
                    'height': int(metadata['height']),
                    'origin': [float(x) for x in metadata['origin']],
                    'post': post
                }
                if pre is not None:
                    data_json['pre'] = pre
                with open(metrics_json, 'w', encoding='utf-8') as jf:
                    _json.dump(data_json, jf, ensure_ascii=False)
            except Exception as e:
                print(f"写入指标JSON失败: {e}")

            print(f"转换信息已保存为: {info_file}")
            return True
        else:
            print(f"保存栅格地图失败: {pgm_file}")
            return False

    def convert(self, pcd_file, output_file):
        """执行PCD到栅格地图的完整转换流程"""
        print(f"开始转换: {pcd_file} -> {output_file}")

        # 选择点云来源：优先使用 patches+poses 组装，其次直接加载PCD
        points = None
        if getattr(self, 'patches_dir', None) and getattr(self, 'poses_file', None):
            points = self.assemble_from_patches(self.poses_file, self.patches_dir)
        if points is None:
            # 回退为直接加载PCD
            points = self.load_pcd(pcd_file)
        if points is None or len(points) == 0:
            print("错误: 无法加载PCD文件或文件为空")
            return False

        # 根据HBA轨迹裁剪ROI（可选）
        if getattr(self, 'hba_file', None) and getattr(self, 'roi_from_hba', False):
            hba = self.load_hba_poses(self.hba_file)
            if hba is not None:
                tx = hba[:, 0]; ty = hba[:, 1]
                buf = float(getattr(self, 'roi_buffer', 2.0) or 2.0)
                xmin, xmax = np.min(tx) - buf, np.max(tx) + buf
                ymin, ymax = np.min(ty) - buf, np.max(ty) + buf
                m = (points[:, 0] >= xmin) & (points[:, 0] <= xmax) & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
                kept = int(np.count_nonzero(m))
                print(f"ROI裁剪: [{xmin:.2f},{xmax:.2f}]x[{ymin:.2f},{ymax:.2f}], 保留 {kept}/{len(points)} 点")
                if kept > 0:
                    points = points[m]

        # 点云预处理（可选）
        points = self.preprocess_points(points)

        # 主方向对齐（可选）
        yaw_deg = getattr(self, 'yaw_deg', None)
        auto_pca = getattr(self, 'auto_yaw_pca', False)
        if yaw_deg is None and auto_pca:
            try:
                yaw_deg = self.estimate_yaw_pca(points[:, :2])
                print(f"PCA估计主方向旋转: {yaw_deg:.2f} 度")
            except Exception as e:
                print(f"PCA主方向估计失败: {e}")
                yaw_deg = None
        if yaw_deg is not None:
            points = self.apply_yaw(points, yaw_deg)

        # 创建占用栅格地图
        occupancy_grid, metadata = self.create_occupancy_grid(points)
        if occupancy_grid is None:
            print("错误: 创建占用栅格地图失败")
            return False

        # 在射线雕刻前记录一次指标（供对比）
        pre_metrics = {
            'obstacles': int(np.sum(occupancy_grid == 100)),
            'free': int(np.sum(occupancy_grid == 0)),
            'unknown': int(np.sum(occupancy_grid == -1))
        }

        # 射线雕刻（可选）
        if getattr(self, 'ray_carve', False):
            # 优先使用HBA轨迹，否则尝试从poses.txt提取平移
            poses_xy = None
            if getattr(self, 'hba_file', None):
                hba = self.load_hba_poses(self.hba_file)
                if hba is not None and hba.shape[0] > 0:
                    poses_xy = hba[:, :2]
            if poses_xy is None and getattr(self, 'poses_file', None):
                # 解析poses.txt
                arr = []
                try:
                    with open(self.poses_file, 'r', encoding='utf-8') as f:
                        for line in f:
                            line = line.strip()
                            if not line or line.startswith('#'):
                                continue
                            ps = line.split()
                            if len(ps) != 8:
                                continue
                            tx, ty, tz = map(float, ps[1:4])
                            arr.append((tx, ty))
                    if arr:
                        poses_xy = np.array(arr, dtype=np.float64)
                except Exception as e:
                    print(f"读取poses.txt失败: {e}")
            if poses_xy is not None:
                mode = getattr(self, 'ray_mode', 'returns')
                ox, oy = float(metadata['origin'][0]), float(metadata['origin'][1])
                if mode == 'returns':
                    occupancy_grid = self.ray_carve_by_returns(
                        occupancy_grid, poses_xy,
                        max_range=float(getattr(self, 'ray_max_range', 30.0) or 30.0),
                        angle_step_deg=float(getattr(self, 'ray_angle_step_deg', 2.0) or 2.0),
                        pose_stride=int(getattr(self, 'ray_pose_stride', 10) or 10)
                    )
                else:
                    occupancy_grid = self.ray_carve_free(
                        occupancy_grid, ox, oy, poses_xy,
                        max_range=float(getattr(self, 'ray_max_range', 30.0) or 30.0),
                        angle_step_deg=float(getattr(self, 'ray_angle_step_deg', 2.0) or 2.0),
                        pose_stride=int(getattr(self, 'ray_pose_stride', 10) or 10)
                    )
        if occupancy_grid is None:
            print("错误: 创建占用栅格地图失败")
            return False

        # 保存栅格地图（携带pre指标以便输出对比/JSON）
        metadata['metrics_pre_carve'] = pre_metrics
        return self.save_occupancy_grid(occupancy_grid, metadata, output_file)

def main():
    parser = argparse.ArgumentParser(description='将PCD点云地图转换为栅格地图')
    parser.add_argument('pcd_file', help='输入PCD文件路径（若提供 --patches-dir 与 --poses-file，将优先使用patches组装）')
    parser.add_argument('-o', '--output', help='输出文件路径（不包含扩展名）')
    parser.add_argument('-r', '--resolution', type=float, default=0.05,
                       help='栅格分辨率(米/像素，默认0.05)')
    parser.add_argument('--ground-min', type=float, default=-0.3,
                       help='地面最小高度(米，默认-0.3)')
    parser.add_argument('--ground-max', type=float, default=0.3,
                       help='地面最大高度(米，默认0.3)')
    parser.add_argument('--ceiling-min', type=float, default=1.8,
                       help='天花板最小高度(米，默认1.8)')
    parser.add_argument('--ceiling-max', type=float, default=3.0,
                       help='天花板最大高度(米，默认3.0)')
    parser.add_argument('--robot-height', type=float, default=0.8,
                       help='机器人高度(米)，用于垂直结构与通行性推断（默认0.8）')
    parser.add_argument('--inflate-radius', type=float, default=0.0,
                       help='障碍物膨胀半径(米)，根据底盘/安全裕度设置（默认0.0=不膨胀）')
    parser.add_argument('--obstacle-thresh', type=int, default=3,
                       help='障碍物点云密度阈值(默认3)')
    parser.add_argument('--vertical-thresh', type=float, default=0.8,
                       help='垂直连续性阈值(默认0.8)')
    parser.add_argument('--disable-panoramic', action='store_true',
                       help='禁用全景分析模式')
    parser.add_argument('--enable-vertical-structures', action='store_true',
                       help='启用垂直结构约束（默认关闭，避免室外高空点误阻塞）')
    parser.add_argument('--no-require-vertical', action='store_true',
                       help='不强制要求垂直高度跨度用于障碍判定（更激进，默认需要）')
    parser.add_argument('--ground-dominates-ratio', type=float, default=2.0,
                       help='地面点密度主导比率（地面密度 >= 比率×机器人层密度 则判为自由，默认2.0）')
    parser.add_argument('--enable-pre-filter', action='store_true',
                       help='启用点云预处理（体素下采样+统计离群去噪）')
    parser.add_argument('--voxel-size', type=float, default=0.0,
                       help='体素下采样尺寸(米)，0表示不下采样')
    parser.add_argument('--sor-mean-k', type=int, default=0,
                       help='统计离群均值邻域K，0表示不启用')
    parser.add_argument('--sor-std-ratio', type=float, default=1.0,
                       help='统计离群标准差系数')
    parser.add_argument('--obstacle-open-iters', type=int, default=0,
                       help='障碍图开运算迭代次数(默认0=不启用)')
    parser.add_argument('--min-obstacle-area', type=int, default=0,
                       help='障碍最小连通域像素数，小于则清理(默认0=不启用)')
    # 方向对齐与射线雕刻
    parser.add_argument('--yaw-deg', type=float, default=None, help='绕Z轴旋转点云(度)，正为逆时针')
    parser.add_argument('--auto-yaw-pca', action='store_true', help='自动PCA估计主方向并对齐x轴')
    parser.add_argument('--ray-carve', action='store_true', help='启用射线雕刻：从轨迹向外发射射线，命中前强制自由')
    parser.add_argument('--ray-mode', type=str, default='returns', choices=['returns','uniform'], help='射线模式：returns(按回波) 或 uniform(均匀扇形)')
    parser.add_argument('--ray-angle-step-deg', type=float, default=2.0, help='射线角步长(度)，越小越密集')
    parser.add_argument('--ray-max-range', type=float, default=30.0, help='射线最大范围(米)')
    parser.add_argument('--ray-pose-stride', type=int, default=10, help='轨迹下采样步长（每N个位姿取1）')
    parser.add_argument('--min-door-width', type=float, default=0.0,
                       help='最小门洞净宽(米)。启用后，膨胀不会封闭≥该净宽的通道')
    # PGO patches 组装与 HBA ROI
    parser.add_argument('--patches-dir', type=str, default='', help='PGO输出的patches目录')
    parser.add_argument('--poses-file', type=str, default='', help='poses.txt路径；每行: name.pcd tx ty tz qw qx qy qz')
    parser.add_argument('--hba-poses', type=str, default='', help='HBA轨迹文件；每行: tx ty tz qw qx qy qz')
    parser.add_argument('--roi-from-hba', action='store_true', help='使用HBA轨迹的包围盒作为ROI裁剪点云')
    parser.add_argument('--roi-buffer', type=float, default=2.0, help='HBA ROI缓冲(米)，默认2.0')

    args = parser.parse_args()

    # 检查输入文件
    pcd_file = Path(args.pcd_file)
    if not pcd_file.exists():
        print(f"错误: PCD文件不存在: {pcd_file}")
        return 1

    # 设置输出文件
    if args.output:
        output_file = Path(args.output)
    else:
        output_file = pcd_file.with_name(pcd_file.stem + '_gridmap')

    # 创建输出目录
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # 创建增强转换器
    converter = PCDToGridMap(
        resolution=args.resolution,
        ground_height=(args.ground_min, args.ground_max),
        ceiling_height=(args.ceiling_min, args.ceiling_max),
        robot_height=args.robot_height,
        obstacle_threshold=args.obstacle_thresh,
        vertical_continuity_threshold=args.vertical_thresh,
        panoramic_analysis=not args.disable_panoramic,
        inflate_radius=args.inflate_radius,
        use_vertical_structures=args.enable_vertical_structures
        ,require_vertical_for_obstacle=not args.no_require_vertical
        ,ground_dominates_ratio=args.ground_dominates_ratio
    )

    # 运行时参数（不扩展构造签名，直接设置）
    converter.enable_pre_filter = args.enable_pre_filter
    converter.voxel_size = args.voxel_size
    converter.sor_mean_k = args.sor_mean_k
    converter.sor_std_ratio = args.sor_std_ratio
    converter.obstacle_open_iters = args.obstacle_open_iters
    converter.min_obstacle_area = args.min_obstacle_area
    # 方向/射线参数
    converter.yaw_deg = args.yaw_deg
    converter.auto_yaw_pca = args.auto_yaw_pca
    converter.ray_carve = args.ray_carve
    converter.ray_mode = args.ray_mode
    converter.ray_angle_step_deg = args.ray_angle_step_deg
    converter.ray_max_range = args.ray_max_range
    converter.ray_pose_stride = args.ray_pose_stride
    converter.min_door_width = args.min_door_width
    # patches/HBA 配置
    converter.patches_dir = Path(args.patches_dir) if args.patches_dir else None
    converter.poses_file = Path(args.poses_file) if args.poses_file else None
    converter.hba_file = Path(args.hba_poses) if args.hba_poses else None
    converter.roi_from_hba = bool(args.roi_from_hba)
    converter.roi_buffer = args.roi_buffer

    # 执行转换
    success = converter.convert(pcd_file, output_file)

    if success:
        print("转换完成!")
        return 0
    else:
        print("转换失败!")
        return 1

if __name__ == '__main__':
    import sys
    sys.exit(main())
