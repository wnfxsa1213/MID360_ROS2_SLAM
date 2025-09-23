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

def load_pcd_with_intensity(file_path):
    """
    加载PCD文件并尝试提取强度信息用于着色
    返回: (point_cloud, intensity_values)
    """
    intensity_values = None

    try:
        # 首先尝试读取PCD文件头部（只读文本部分）
        print(f'📖 分析PCD文件结构: {file_path}')

        # 读取文件头部（找到DATA行为止）
        header_lines = []
        has_intensity = False
        intensity_index = -1
        is_binary = False
        point_count = 0
        data_format = None

        with open(file_path, 'rb') as f:
            # 读取头部，逐行解析直到DATA行
            current_line = b""
            while True:
                byte = f.read(1)
                if not byte:
                    break

                if byte == b'\n':
                    try:
                        line = current_line.decode('utf-8').strip()
                        header_lines.append(line)

                        # 检查字段定义
                        if line.startswith('FIELDS'):
                            fields = line.split()[1:]  # 跳过 'FIELDS' 关键字
                            print(f"📊 检测到字段: {fields}")
                            if 'intensity' in fields:
                                has_intensity = True
                                intensity_index = fields.index('intensity')
                                print(f"✅ 发现强度字段，位于索引 {intensity_index}")
                            elif 'i' in fields:
                                has_intensity = True
                                intensity_index = fields.index('i')
                                print(f"✅ 发现强度字段(i)，位于索引 {intensity_index}")

                        # 检查点数量
                        elif line.startswith('POINTS'):
                            point_count = int(line.split()[1])
                            print(f"📊 点云总数: {point_count:,}")

                        # 检查数据格式
                        elif line.startswith('DATA'):
                            data_format = line.split()[1] if len(line.split()) > 1 else 'ascii'
                            is_binary = (data_format.lower() == 'binary')
                            print(f"📊 数据格式: {data_format}")
                            break

                        current_line = b""
                    except UnicodeDecodeError:
                        # 如果遇到二进制数据，停止头部解析
                        break
                else:
                    current_line += byte

        # 如果找到强度字段，尝试使用Open3D读取并从中提取强度
        if has_intensity:
            print(f"🔍 尝试从Open3D读取的点云中提取强度信息...")

            # 使用Open3D加载点云
            pcd = o3d.io.read_point_cloud(file_path)

            # 检查点云是否有强度作为颜色或其他属性存储
            points = np.asarray(pcd.points)
            if len(points) > 0:
                print(f"✅ Open3D成功加载 {len(points)} 个点")

                # 尝试读取原始PCD文件的强度数据（如果是ASCII格式）
                if not is_binary:
                    print("🔍 尝试从ASCII PCD中提取强度...")
                    try:
                        with open(file_path, 'r', encoding='utf-8') as f:
                            lines = f.readlines()

                        # 找到数据开始位置
                        data_start = 0
                        for i, line in enumerate(lines):
                            if line.startswith('DATA'):
                                data_start = i + 1
                                break

                        # 解析强度数据
                        intensity_values = []
                        for line in lines[data_start:]:
                            line = line.strip()
                            if line:
                                parts = line.split()
                                if len(parts) > intensity_index:
                                    try:
                                        intensity = float(parts[intensity_index])
                                        intensity_values.append(intensity)
                                    except (ValueError, IndexError):
                                        intensity_values.append(0.0)

                        if intensity_values and len(intensity_values) == len(points):
                            intensity_values = np.array(intensity_values)
                            print(f"✅ 成功提取 {len(intensity_values)} 个强度值 (ASCII)")
                            print(f"📊 强度范围: {intensity_values.min():.1f} ~ {intensity_values.max():.1f}")
                        else:
                            print("⚠️  ASCII强度数据解析失败或数量不匹配")
                            intensity_values = None

                    except Exception as e:
                        print(f"⚠️  ASCII强度解析失败: {e}")
                        intensity_values = None
                else:
                    print("⚠️  二进制PCD强度解析暂不支持，将使用替代着色方案")
                    intensity_values = None
            else:
                print("❌ Open3D无法加载点云")
                pcd = None
        else:
            print("⚠️  未发现强度字段")
            # 使用Open3D加载点云几何
            pcd = o3d.io.read_point_cloud(file_path)

    except Exception as e:
        print(f"⚠️  强度解析出错: {e}")
        intensity_values = None
        # 使用Open3D加载点云几何
        pcd = o3d.io.read_point_cloud(file_path)

    return pcd, intensity_values

def apply_intensity_coloring(pcd, intensity_values):
    """
    根据强度值为点云着色
    """
    if intensity_values is None or len(intensity_values) == 0:
        return False

    points = np.asarray(pcd.points)
    if len(points) != len(intensity_values):
        print(f"⚠️  点数量({len(points)})与强度数量({len(intensity_values)})不匹配")
        return False

    # 归一化强度值
    intensity_min = intensity_values.min()
    intensity_max = intensity_values.max()
    intensity_range = intensity_max - intensity_min

    print(f"🎨 基于强度着色 (范围: {intensity_min:.1f} ~ {intensity_max:.1f})")

    if intensity_range < 1e-6:
        print("⚠️  强度值无变化，使用统一颜色")
        colors = np.full((len(points), 3), [0.7, 0.7, 0.7])
    else:
        # 归一化到0-1范围
        normalized_intensity = (intensity_values - intensity_min) / intensity_range
        colors = np.zeros((len(points), 3))

        # 强度映射到颜色 - 使用暖色调方案
        for i, intensity in enumerate(normalized_intensity):
            if intensity < 0.2:
                # 低强度: 深蓝色
                colors[i] = [0.1, 0.1, 0.8]
            elif intensity < 0.4:
                # 中低强度: 青色到绿色
                t = (intensity - 0.2) / 0.2
                colors[i] = [0.1, 0.5 + 0.5*t, 0.8 - 0.3*t]
            elif intensity < 0.6:
                # 中等强度: 绿色到黄色
                t = (intensity - 0.4) / 0.2
                colors[i] = [0.8*t, 1.0, 0.5 - 0.5*t]
            elif intensity < 0.8:
                # 中高强度: 黄色到橙色
                t = (intensity - 0.6) / 0.2
                colors[i] = [1.0, 1.0 - 0.3*t, 0.0]
            else:
                # 高强度: 橙色到红色
                t = (intensity - 0.8) / 0.2
                colors[i] = [1.0, 0.7 - 0.7*t, 0.0]

    # 应用颜色
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print(f"✅ 强度着色完成，颜色范围: {colors.min():.3f} ~ {colors.max():.3f}")

    # 显示样本
    sample_indices = [0, len(colors)//4, len(colors)//2, 3*len(colors)//4, -1]
    print("🎨 强度着色样本:")
    for idx in sample_indices:
        intensity = intensity_values[idx]
        color = colors[idx]
        print(f"   点{idx}: 强度={intensity:.1f}, RGB=({color[0]:.2f},{color[1]:.2f},{color[2]:.2f})")

    return True

def visualize_with_open3d(point_cloud_file, trajectory_file=None):
    """使用Open3D可视化点云和轨迹"""
    print(f'🎨 使用Open3D可视化')

    # 加载点云（包含强度信息）
    print(f'📖 加载点云文件: {point_cloud_file}')
    pcd, intensity_values = load_pcd_with_intensity(point_cloud_file)

    if len(pcd.points) == 0:
        print(f'❌ 点云文件为空或格式不支持: {point_cloud_file}')
        return False

    print(f'✅ 成功加载 {len(pcd.points)} 个点')

    # 设置点云颜色 - 优先使用强度信息
    points = np.asarray(pcd.points)
    colored_successfully = False

    # 首先尝试基于强度着色
    if intensity_values is not None:
        print("🎨 尝试基于强度信息着色...")
        colored_successfully = apply_intensity_coloring(pcd, intensity_values)

    # 如果强度着色失败，检查原有颜色
    if not colored_successfully:
        need_coloring = True

        if pcd.has_colors():
            colors_array = np.asarray(pcd.colors)
            print(f"📊 检测到原始颜色数据，范围: {colors_array.min():.3f} ~ {colors_array.max():.3f}")

            # 如果颜色都接近0（黑色），则需要重新着色
            if colors_array.max() < 0.1:
                print("⚠️  颜色数据过暗，将重新着色")
                need_coloring = True
            else:
                print("✅ 使用原有颜色数据")
                need_coloring = False
                colored_successfully = True
        else:
            print("📊 未检测到颜色信息，将生成颜色")

        # 如果仍需要着色，使用高度映射
        if need_coloring:
            z_min, z_max = points[:, 2].min(), points[:, 2].max()
            z_range = z_max - z_min

            print(f"🎨 生成高度颜色映射 (高度范围: {z_min:.2f}m ~ {z_max:.2f}m)")

            if z_range > 0.01:
                # 创建热力图着色 (蓝->绿->黄->红)
                normalized_z = (points[:, 2] - z_min) / z_range
                colors = np.zeros((len(points), 3))

                # 使用热力图配色方案
                print("🎨 应用高度热力图配色...")
                for i, nz in enumerate(normalized_z):
                    if nz < 0.25:
                        # 蓝色到青色
                        colors[i] = [0, 4*nz, 1]
                    elif nz < 0.5:
                        # 青色到绿色
                        colors[i] = [0, 1, 1-4*(nz-0.25)]
                    elif nz < 0.75:
                        # 绿色到黄色
                        colors[i] = [4*(nz-0.5), 1, 0]
                    else:
                        # 黄色到红色
                        colors[i] = [1, 1-4*(nz-0.75), 0]

                # 应用颜色到点云
                pcd.colors = o3d.utility.Vector3dVector(colors)
                print(f"✅ 高度热力图着色完成，颜色范围: {colors.min():.3f} ~ {colors.max():.3f}")
                colored_successfully = True

                # 显示样本颜色
                sample_indices = [0, len(colors)//4, len(colors)//2, 3*len(colors)//4, -1]
                print("🎨 高度着色样本:")
                for idx in sample_indices:
                    h = points[idx, 2]
                    c = colors[idx]
                    print(f"   点{idx}: 高度={h:.2f}m, RGB=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})")
            else:
                # 如果高度差太小，使用统一的亮灰色
                colors = np.full((len(points), 3), [0.8, 0.8, 0.8])
                pcd.colors = o3d.utility.Vector3dVector(colors)
                print("✅ 应用统一亮灰色着色")
                colored_successfully = True

    # 最终验证 - 确保点云有颜色
    if not colored_successfully:
        print("⚠️  所有着色方法都失败，使用备用随机着色方案")
        backup_colors = np.random.rand(len(points), 3) * 0.6 + 0.2  # 随机但适中的颜色
        pcd.colors = o3d.utility.Vector3dVector(backup_colors)
        colored_successfully = True

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
    print(f'\n📊 点云统计信息:')
    print(f'   点数量: {len(points):,}')
    print(f'   X范围: {points[:, 0].min():.2f} ~ {points[:, 0].max():.2f} ({points[:, 0].max() - points[:, 0].min():.2f}m)')
    print(f'   Y范围: {points[:, 1].min():.2f} ~ {points[:, 1].max():.2f} ({points[:, 1].max() - points[:, 1].min():.2f}m)')
    print(f'   Z范围: {points[:, 2].min():.2f} ~ {points[:, 2].max():.2f} ({points[:, 2].max() - points[:, 2].min():.2f}m)')

    # 添加颜色信息
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        print(f'   颜色范围: R({colors[:,0].min():.2f}~{colors[:,0].max():.2f}) G({colors[:,1].min():.2f}~{colors[:,1].max():.2f}) B({colors[:,2].min():.2f}~{colors[:,2].max():.2f})')

        # 添加强度信息
        if intensity_values is not None:
            print(f'   强度信息: {intensity_values.min():.1f} ~ {intensity_values.max():.1f} (基于强度着色)')
        else:
            print(f'   着色方案: 基于高度或默认方案')

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
    print(f'\n🚀 启动可视化 (几何对象: {len(geometries)}, 着色: {"✅" if colored_successfully else "❌"})')

    o3d.visualization.draw_geometries(
        geometries,
        window_name="MID360 SLAM 地图查看器 - 强度着色版",
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