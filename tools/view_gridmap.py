#!/usr/bin/env python3
"""
栅格地图查看工具
用于查看和分析生成的栅格地图
"""

import numpy as np
import cv2
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path
import argparse

def load_gridmap(yaml_file):
    """加载栅格地图"""
    yaml_file = Path(yaml_file)

    if not yaml_file.exists():
        print(f"错误: YAML文件不存在: {yaml_file}")
        return None, None

    # 加载YAML元数据
    with open(yaml_file, 'r', encoding='utf-8') as f:
        metadata = yaml.safe_load(f)

    # 加载PGM图像
    pgm_file = yaml_file.parent / metadata['image']
    if not pgm_file.exists():
        print(f"错误: PGM文件不存在: {pgm_file}")
        return None, None

    image = cv2.imread(str(pgm_file), cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"错误: 无法加载PGM文件: {pgm_file}")
        return None, None

    return image, metadata

def analyze_gridmap(image, metadata):
    """分析栅格地图统计信息"""
    total_pixels = image.size

    # 统计不同类型的像素
    obstacles = np.sum(image == 0)      # 黑色 = 障碍物
    free_space = np.sum(image == 255)   # 白色 = 自由空间
    unknown = np.sum((image != 0) & (image != 255))  # 灰色 = 未知

    print(f"\n栅格地图分析:")
    print(f"  图像尺寸: {image.shape[1]} x {image.shape[0]} 像素")
    print(f"  分辨率: {metadata['resolution']} 米/像素")
    print(f"  地图原点: ({metadata['origin'][0]:.2f}, {metadata['origin'][1]:.2f})")
    print(f"  实际地图尺寸: {image.shape[1] * metadata['resolution']:.1f} x {image.shape[0] * metadata['resolution']:.1f} 米")
    print(f"\n像素统计:")
    print(f"  总像素数: {total_pixels}")
    print(f"  障碍物像素: {obstacles} ({obstacles/total_pixels*100:.1f}%)")
    print(f"  自由空间像素: {free_space} ({free_space/total_pixels*100:.1f}%)")
    print(f"  未知区域像素: {unknown} ({unknown/total_pixels*100:.1f}%)")

def visualize_gridmap(image, metadata, save_png=False, output_file=None):
    """可视化栅格地图"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # 原始地图显示
    ax1.imshow(image, cmap='gray', origin='lower')
    ax1.set_title('原始栅格地图\n(黑色=障碍物, 白色=自由空间, 灰色=未知)', fontsize=12)
    ax1.set_xlabel(f'X 像素 (分辨率: {metadata["resolution"]:.3f} m/pixel)')
    ax1.set_ylabel(f'Y 像素')
    ax1.grid(True, alpha=0.3)

    # 彩色增强显示
    colored_map = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

    # 白色 = 自由空间
    free_mask = image == 255
    colored_map[free_mask] = [255, 255, 255]

    # 黑色/深蓝 = 障碍物
    obstacle_mask = image == 0
    colored_map[obstacle_mask] = [0, 0, 139]  # 深蓝色

    # 灰色 = 未知区域
    unknown_mask = (image != 0) & (image != 255)
    colored_map[unknown_mask] = [128, 128, 128]

    ax2.imshow(colored_map, origin='lower')
    ax2.set_title('增强彩色地图\n(深蓝=障碍物, 白色=自由空间, 灰色=未知)', fontsize=12)
    ax2.set_xlabel(f'X 像素')
    ax2.set_ylabel(f'Y 像素')
    ax2.grid(True, alpha=0.3)

    # 添加比例尺
    scale_length_pixels = int(10.0 / metadata['resolution'])  # 10米的比例尺
    if scale_length_pixels > 0:
        scale_y = image.shape[0] - 50
        scale_x_start = 50
        scale_x_end = scale_x_start + scale_length_pixels

        # 在两个子图上都添加比例尺
        for ax in [ax1, ax2]:
            ax.plot([scale_x_start, scale_x_end], [scale_y, scale_y], 'r-', linewidth=3)
            ax.text(scale_x_start + scale_length_pixels/2, scale_y - 20, '10m',
                   ha='center', va='top', color='red', fontweight='bold', fontsize=10)

    # 添加原点标记
    origin_x = -metadata['origin'][0] / metadata['resolution']
    origin_y = -metadata['origin'][1] / metadata['resolution']

    for ax in [ax1, ax2]:
        if 0 <= origin_x < image.shape[1] and 0 <= origin_y < image.shape[0]:
            ax.plot(origin_x, origin_y, 'r+', markersize=15, markeredgewidth=3)
            ax.text(origin_x + 20, origin_y + 20, '原点', color='red', fontweight='bold')

    plt.tight_layout()

    if save_png and output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"可视化图像已保存为: {output_file}")

    plt.show()

def main():
    parser = argparse.ArgumentParser(description='查看和分析栅格地图')
    parser.add_argument('yaml_file', help='栅格地图YAML文件路径')
    parser.add_argument('--save-png', action='store_true', help='保存可视化为PNG图像')
    parser.add_argument('-o', '--output', help='输出PNG文件路径')

    args = parser.parse_args()

    # 加载栅格地图
    image, metadata = load_gridmap(args.yaml_file)
    if image is None:
        return 1

    print(f"成功加载栅格地图: {args.yaml_file}")

    # 分析地图
    analyze_gridmap(image, metadata)

    # 设置输出文件
    output_file = None
    if args.save_png:
        if args.output:
            output_file = Path(args.output)
        else:
            yaml_path = Path(args.yaml_file)
            output_file = yaml_path.with_name(yaml_path.stem + '_visualization.png')

    # 可视化地图
    try:
        visualize_gridmap(image, metadata, args.save_png, output_file)
    except Exception as e:
        print(f"可视化时出错: {e}")
        print("注意: 如果在无图形界面环境中运行，请使用 --save-png 参数")
        return 1

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main())