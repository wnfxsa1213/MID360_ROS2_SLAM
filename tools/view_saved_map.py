#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MID360 SLAM 保存地图查看工具
用于查看保存的点云地图和轨迹数据
"""

import os
import sys
import argparse
import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

def load_trajectory(trajectory_file):
    """加载轨迹文件"""
    if not os.path.exists(trajectory_file):
        print(f'❌ 轨迹文件不存在: {trajectory_file}')
        return None
    
    print(f'📖 加载轨迹文件: {trajectory_file}')
    
    try:
        # 读取轨迹数据 (时间戳 x y z qx qy qz qw)
        data = []
        with open(trajectory_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split()
                    if len(parts) >= 8:
                        timestamp = float(parts[0])
                        position = [float(parts[1]), float(parts[2]), float(parts[3])]
                        orientation = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
                        data.append((timestamp, position, orientation))
        
        if not data:
            print('❌ 轨迹文件为空或格式不正确')
            return None
        
        print(f'✅ 成功加载 {len(data)} 个轨迹点')
        return data
        
    except Exception as e:
        print(f'❌ 轨迹文件加载失败: {e}')
        return None

def create_trajectory_lineset(trajectory_data):
    """创建轨迹线条用于可视化"""
    if not trajectory_data:
        return None
    
    # 提取位置点
    points = np.array([pose[1] for pose in trajectory_data])
    
    # 创建线条连接
    lines = []
    for i in range(len(points) - 1):
        lines.append([i, i + 1])
    
    # 创建Open3D线条集
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    # 设置轨迹颜色 (红色)
    colors = np.array([[1, 0, 0] for _ in lines])  # 红色
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

def visualize_with_open3d(point_cloud_file, trajectory_file=None):
    """使用Open3D可视化点云和轨迹"""
    print(f'🎨 使用Open3D可视化')
    
    # 加载点云
    print(f'📖 加载点云文件: {point_cloud_file}')
    pcd = o3d.io.read_point_cloud(point_cloud_file)
    
    if len(pcd.points) == 0:
        print(f'❌ 点云文件为空或格式不支持: {point_cloud_file}')
        return False
    
    print(f'✅ 成功加载 {len(pcd.points)} 个点')
    
    # 设置点云颜色
    if not pcd.has_colors():
        # 根据高度着色
        points = np.asarray(pcd.points)
        z_min, z_max = points[:, 2].min(), points[:, 2].max()
        z_range = z_max - z_min
        
        if z_range > 0:
            # 高度着色 (蓝色到红色)
            colors = np.zeros((len(points), 3))
            normalized_z = (points[:, 2] - z_min) / z_range
            colors[:, 0] = normalized_z  # 红色分量
            colors[:, 2] = 1 - normalized_z  # 蓝色分量
            pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 准备可视化对象
    geometries = [pcd]
    
    # 加载轨迹
    trajectory_data = None
    if trajectory_file and os.path.exists(trajectory_file):
        trajectory_data = load_trajectory(trajectory_file)
        if trajectory_data:
            trajectory_lineset = create_trajectory_lineset(trajectory_data)
            if trajectory_lineset:
                geometries.append(trajectory_lineset)
    
    # 打印统计信息
    points = np.asarray(pcd.points)
    print(f'\n📊 点云统计信息:')
    print(f'   点数量: {len(points):,}')
    print(f'   X范围: {points[:, 0].min():.2f} ~ {points[:, 0].max():.2f} ({points[:, 0].max() - points[:, 0].min():.2f}m)')
    print(f'   Y范围: {points[:, 1].min():.2f} ~ {points[:, 1].max():.2f} ({points[:, 1].max() - points[:, 1].min():.2f}m)')
    print(f'   Z范围: {points[:, 2].min():.2f} ~ {points[:, 2].max():.2f} ({points[:, 2].max() - points[:, 2].min():.2f}m)')
    
    if trajectory_data:
        print(f'\n📊 轨迹统计信息:')
        print(f'   轨迹点数: {len(trajectory_data)}')
        positions = np.array([pose[1] for pose in trajectory_data])
        total_distance = 0
        for i in range(1, len(positions)):
            total_distance += np.linalg.norm(positions[i] - positions[i-1])
        print(f'   总行程: {total_distance:.2f}m')
    
    print(f'\n🎮 可视化控制:')
    print(f'   鼠标左键: 旋转视角')
    print(f'   鼠标右键: 平移')
    print(f'   鼠标滚轮: 缩放')
    print(f'   按ESC键: 退出')
    print(f'   按空格键: 重置视角')
    
    # 启动可视化
    o3d.visualization.draw_geometries(
        geometries,
        window_name="MID360 SLAM 地图查看器",
        width=1200,
        height=800,
        left=50,
        top=50
    )
    
    return True

def print_file_info(file_path):
    """打印文件信息"""
    if not os.path.exists(file_path):
        print(f'❌ 文件不存在: {file_path}')
        return False
    
    file_size = os.path.getsize(file_path)
    file_ext = os.path.splitext(file_path)[1].lower()
    
    print(f'📄 文件信息:')
    print(f'   路径: {file_path}')
    print(f'   大小: {file_size / 1024 / 1024:.2f} MB')
    print(f'   格式: {file_ext}')
    
    return True

def main():
    parser = argparse.ArgumentParser(description='MID360 SLAM 保存地图查看工具')
    parser.add_argument('point_cloud', type=str, help='点云文件路径 (.pcd 或 .ply)')
    parser.add_argument('--trajectory', '-t', type=str, help='轨迹文件路径 (可选)')
    parser.add_argument('--info-only', action='store_true', help='仅显示文件信息，不启动可视化')
    
    args = parser.parse_args()
    
    print('🗺️  MID360 SLAM 地图查看器')
    print('=' * 50)
    
    # 检查点云文件
    if not os.path.exists(args.point_cloud):
        print(f'❌ 点云文件不存在: {args.point_cloud}')
        sys.exit(1)
    
    # 打印文件信息
    print_file_info(args.point_cloud)
    
    if args.trajectory:
        print_file_info(args.trajectory)
    
    if args.info_only:
        print('ℹ️  信息模式，不启动可视化')
        return
    
    # 检查Open3D
    if not HAS_OPEN3D:
        print('❌ 未安装Open3D库，无法进行可视化')
        print('   安装命令: pip install open3d')
        
        # 提供基本的统计信息
        print('\n📊 尝试读取基本信息...')
        try:
            if args.point_cloud.lower().endswith('.pcd'):
                # 简单解析PCD头部
                with open(args.point_cloud, 'r') as f:
                    for line in f:
                        if line.startswith('POINTS'):
                            point_count = int(line.split()[1])
                            print(f'   点云数量: {point_count:,}')
                            break
        except:
            pass
        
        sys.exit(1)
    
    # 启动可视化
    try:
        success = visualize_with_open3d(args.point_cloud, args.trajectory)
        if success:
            print('✅ 可视化完成')
        else:
            print('❌ 可视化失败')
            sys.exit(1)
    except KeyboardInterrupt:
        print('\n⚠️  用户中断操作')
    except Exception as e:
        print(f'❌ 可视化错误: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()