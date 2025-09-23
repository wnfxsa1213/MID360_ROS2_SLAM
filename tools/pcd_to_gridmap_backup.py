#!/usr/bin/env python3
"""
PCD地图转栅格地图工具
将保存的PCD点云地图转换为2D栅格地图（占用栅格地图）
"""

import numpy as np
import open3d as o3d
import cv2
import yaml
import argparse
import os
from pathlib import Path

class PCDToGridMap:
    def __init__(self, resolution=0.05, height_range=(-0.5, 2.0),
                 obstacle_threshold=5, free_threshold=1):
        """
        初始化PCD到栅格地图转换器

        Args:
            resolution: 栅格分辨率 (米/像素)
            height_range: 有效高度范围 (min_height, max_height)
            obstacle_threshold: 障碍物点云密度阈值
            free_threshold: 自由空间点云密度阈值
        """
        self.resolution = resolution
        self.height_range = height_range
        self.obstacle_threshold = obstacle_threshold
        self.free_threshold = free_threshold

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

    def filter_points_by_height(self, points):
        """根据高度过滤点云"""
        z_filter = (points[:, 2] >= self.height_range[0]) & (points[:, 2] <= self.height_range[1])
        filtered_points = points[z_filter]
        print(f"高度过滤后剩余 {len(filtered_points)} 个点")
        return filtered_points

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
        """创建占用栅格地图"""
        # 过滤高度
        filtered_points = self.filter_points_by_height(points)
        if len(filtered_points) == 0:
            print("警告: 高度过滤后没有剩余点云")
            return None, None

        # 转换到栅格坐标
        grid_x, grid_y, grid_width, grid_height, origin_x, origin_y = self.points_to_grid(filtered_points)

        # 创建点云密度地图
        density_map = np.zeros((grid_height, grid_width), dtype=np.int32)

        # 统计每个栅格的点云数量
        for i in range(len(grid_x)):
            x, y = grid_x[i], grid_y[i]
            density_map[y, x] += 1

        # 创建占用栅格地图
        # 0: 未知区域 (灰色)
        # 100: 障碍物 (黑色)
        # -1: 自由空间 (白色)
        occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.int8)

        # 设置障碍物区域
        obstacle_mask = density_map >= self.obstacle_threshold
        occupancy_grid[obstacle_mask] = 100

        # 设置自由空间区域（密度很低但不为0的区域）
        free_mask = (density_map > 0) & (density_map < self.free_threshold)
        occupancy_grid[free_mask] = 0  # 自由空间设为0

        # 膨胀处理，增强障碍物区域
        kernel = np.ones((3, 3), np.uint8)
        obstacle_binary = (occupancy_grid == 100).astype(np.uint8)
        obstacle_dilated = cv2.dilate(obstacle_binary, kernel, iterations=1)
        occupancy_grid[obstacle_dilated == 1] = 100

        print(f"占用栅格统计:")
        print(f"  障碍物栅格: {np.sum(occupancy_grid == 100)}")
        print(f"  自由空间栅格: {np.sum(occupancy_grid == 0)}")
        print(f"  未知区域栅格: {np.sum(occupancy_grid == -1)}")

        # 创建元数据
        metadata = {
            'resolution': self.resolution,
            'origin': [origin_x, origin_y, 0.0],
            'width': grid_width,
            'height': grid_height,
            'height_range': self.height_range,
            'obstacle_threshold': self.obstacle_threshold,
            'free_threshold': self.free_threshold
        }

        return occupancy_grid, metadata

    def save_occupancy_grid(self, occupancy_grid, metadata, output_file):
        """保存占用栅格地图为PGM格式"""
        if occupancy_grid is None:
            print("错误: 占用栅格地图为空，无法保存")
            return False

        # 转换为PGM格式的图像数据
        # PGM格式: 0=黑色(障碍物), 255=白色(自由空间), 127=灰色(未知)
        pgm_data = np.full_like(occupancy_grid, 127, dtype=np.uint8)  # 默认灰色(未知)
        pgm_data[occupancy_grid == 0] = 255     # 自由空间 -> 白色
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
                'mode': 'trinary'
            }

            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)

            print(f"地图元数据已保存为: {yaml_file}")

            # 保存详细信息
            info_file = output_file.with_suffix('.info')
            with open(info_file, 'w', encoding='utf-8') as f:
                f.write("PCD到栅格地图转换信息\n")
                f.write("=" * 50 + "\n")
                f.write(f"分辨率: {metadata['resolution']} 米/像素\n")
                f.write(f"地图尺寸: {metadata['width']} x {metadata['height']} 像素\n")
                f.write(f"地图原点: ({metadata['origin'][0]:.2f}, {metadata['origin'][1]:.2f})\n")
                f.write(f"高度过滤范围: {metadata['height_range'][0]:.1f} ~ {metadata['height_range'][1]:.1f} 米\n")
                f.write(f"障碍物阈值: {metadata['obstacle_threshold']} 点/栅格\n")
                f.write(f"自由空间阈值: {metadata['free_threshold']} 点/栅格\n")
                f.write(f"\n栅格统计:\n")
                f.write(f"  障碍物栅格: {np.sum(occupancy_grid == 100)}\n")
                f.write(f"  自由空间栅格: {np.sum(occupancy_grid == 0)}\n")
                f.write(f"  未知区域栅格: {np.sum((occupancy_grid != 0) & (occupancy_grid != 100))}\n")

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
    parser.add_argument('--height-min', type=float, default=-0.5,
                       help='最小高度(米，默认-0.5)')
    parser.add_argument('--height-max', type=float, default=2.0,
                       help='最大高度(米，默认2.0)')
    parser.add_argument('--obstacle-thresh', type=int, default=5,
                       help='障碍物点云密度阈值(默认5)')
    parser.add_argument('--free-thresh', type=int, default=1,
                       help='自由空间点云密度阈值(默认1)')

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

    # 创建转换器
    converter = PCDToGridMap(
        resolution=args.resolution,
        height_range=(args.height_min, args.height_max),
        obstacle_threshold=args.obstacle_thresh,
        free_threshold=args.free_thresh
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