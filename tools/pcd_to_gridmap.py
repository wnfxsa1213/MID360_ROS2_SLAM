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
import os
from pathlib import Path
from scipy import ndimage
from collections import defaultdict

class PCDToGridMap:
    def __init__(self, resolution=0.05, ground_height=(-0.3, 0.3),
                 ceiling_height=(1.8, 3.0), robot_height=0.5,
                 obstacle_threshold=3, vertical_continuity_threshold=0.8,
                 panoramic_analysis=True):
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

    def detect_vertical_structures(self, points):
        """检测垂直结构（墙面、柱子等）"""
        if not self.panoramic_analysis:
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

        # 计算栅格尺寸
        grid_width = int((max_x - min_x) / self.resolution)
        grid_height = int((max_y - min_y) / self.resolution)

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

        # 创建多层密度地图
        density_maps = {}
        for layer_name, points_layer in layer_points.items():
            if len(points_layer) == 0:
                continue

            layer_grid_x, layer_grid_y, _, _, _, _ = self.points_to_grid(points_layer)
            density_map = np.zeros((grid_height, grid_width), dtype=np.int32)

            # 统计每个栅格的点云数量
            valid_indices = (layer_grid_x >= 0) & (layer_grid_x < grid_width) & \
                           (layer_grid_y >= 0) & (layer_grid_y < grid_height)
            for i in np.where(valid_indices)[0]:
                x, y = layer_grid_x[i], layer_grid_y[i]
                density_map[y, x] += 1

            density_maps[layer_name] = density_map

        # 创建增强占用栅格地图
        occupancy_grid = self.create_enhanced_occupancy_grid(density_maps, grid_width, grid_height)

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

    def create_enhanced_occupancy_grid(self, density_maps, grid_width, grid_height):
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
            # 机器人高度层的高密度点云为障碍物
            obstacle_mask = robot_density >= self.obstacle_threshold
            occupancy_grid[obstacle_mask] = 100  # 障碍物

        # 顶部和天花板层分析：辅助判断
        overhead_obstacle_mask = np.zeros((grid_height, grid_width), dtype=bool)
        for layer_name in ['overhead', 'ceiling']:
            if layer_name in density_maps:
                layer_density = density_maps[layer_name]
                overhead_obstacle_mask |= (layer_density >= self.obstacle_threshold)

        # 在机器人高度层没有障碍物但上方有障碍物的区域，可能是低矮障碍物
        potential_low_obstacle = overhead_obstacle_mask & (occupancy_grid == -1)
        occupancy_grid[potential_low_obstacle] = 50  # 低信心度障碍物

        # 增强自由空间推理：利用全景扫描特性
        if self.panoramic_analysis:
            occupancy_grid = self.enhance_free_space_inference(occupancy_grid, density_maps)

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
        # PGM格式: 0=黑色(障碍物), 255=白色(自由空间), 127=灰色(未知), 200=浅灰色(低信心度障碍物)
        pgm_data = np.full_like(occupancy_grid, 127, dtype=np.uint8)  # 默认灰色(未知)
        pgm_data[occupancy_grid == 0] = 255     # 自由空间 -> 白色
        pgm_data[occupancy_grid == 50] = 200    # 低信心度障碍物 -> 浅灰色
        pgm_data[occupancy_grid == 100] = 0     # 障碍物 -> 黑色

        # 保存PGM文件
        pgm_file = output_file.with_suffix('.pgm')
        success = cv2.imwrite(str(pgm_file), pgm_data)

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

            print(f"转换信息已保存为: {info_file}")
            return True
        else:
            print(f"保存栅格地图失败: {pgm_file}")
            return False

    def convert(self, pcd_file, output_file):
        """执行PCD到栅格地图的完整转换流程"""
        print(f"开始转换: {pcd_file} -> {output_file}")

        # 加载PCD文件
        points = self.load_pcd(pcd_file)
        if points is None or len(points) == 0:
            print("错误: 无法加载PCD文件或文件为空")
            return False

        # 创建占用栅格地图
        occupancy_grid, metadata = self.create_occupancy_grid(points)
        if occupancy_grid is None:
            print("错误: 创建占用栅格地图失败")
            return False

        # 保存栅格地图
        return self.save_occupancy_grid(occupancy_grid, metadata, output_file)

def main():
    parser = argparse.ArgumentParser(description='将PCD点云地图转换为栅格地图')
    parser.add_argument('pcd_file', help='输入PCD文件路径')
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
    parser.add_argument('--robot-height', type=float, default=0.5,
                       help='机器人高度(米，默认0.5)')
    parser.add_argument('--obstacle-thresh', type=int, default=3,
                       help='障碍物点云密度阈值(默认3)')
    parser.add_argument('--vertical-thresh', type=float, default=0.8,
                       help='垂直连续性阈值(默认0.8)')
    parser.add_argument('--disable-panoramic', action='store_true',
                       help='禁用全景分析模式')

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
        panoramic_analysis=not args.disable_panoramic
    )

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