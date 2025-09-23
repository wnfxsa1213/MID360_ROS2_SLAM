#!/usr/bin/env python3
"""
测试优化后的地图精细化配置

用于验证新配置是否能解决无建筑物区域噪点问题
"""

import os
import sys
import argparse
import time
from pathlib import Path

# 直接运行主模块
import subprocess
import numpy as np
import open3d as o3d

def analyze_point_density_distribution(pcd_path: str, name: str = ""):
    """分析点云密度分布"""
    print(f"\n=== {name} 密度分析 ===")

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    if len(points) == 0:
        print("点云为空")
        return

    print(f"总点数: {len(points):,}")

    # 计算边界框
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    dimensions = max_bound - min_bound
    volume = np.prod(dimensions)

    print(f"边界框: {dimensions}")
    print(f"体积: {volume:.2f} 立方米")
    print(f"整体密度: {len(points)/volume:.2f} 点/立方米")

    # 分析空间分布
    # 将空间分成网格，分析每个网格的密度
    grid_size = 1.0  # 1米网格

    x_bins = int(np.ceil(dimensions[0] / grid_size))
    y_bins = int(np.ceil(dimensions[1] / grid_size))
    z_bins = int(np.ceil(dimensions[2] / grid_size))

    # 计算每个点所在的网格索引
    grid_indices = np.floor((points - min_bound) / grid_size).astype(int)

    # 统计每个网格的点数
    grid_counts = {}
    for i, (x, y, z) in enumerate(grid_indices):
        # 确保索引在范围内
        x = min(x, x_bins - 1)
        y = min(y, y_bins - 1)
        z = min(z, z_bins - 1)

        grid_key = (x, y, z)
        grid_counts[grid_key] = grid_counts.get(grid_key, 0) + 1

    # 计算网格密度统计
    densities = list(grid_counts.values())
    total_grids = x_bins * y_bins * z_bins
    empty_grids = total_grids - len(grid_counts)

    print(f"\n网格分析 (1m³网格):")
    print(f"总网格数: {total_grids:,}")
    print(f"非空网格数: {len(grid_counts):,}")
    print(f"空网格数: {empty_grids:,}")
    print(f"空网格比例: {empty_grids/total_grids*100:.1f}%")

    if densities:
        print(f"非空网格密度统计:")
        print(f"  平均密度: {np.mean(densities):.2f} 点/网格")
        print(f"  密度标准差: {np.std(densities):.2f}")
        print(f"  最小密度: {np.min(densities)} 点/网格")
        print(f"  最大密度: {np.max(densities)} 点/网格")
        print(f"  中位数密度: {np.median(densities):.2f} 点/网格")

        # 找出低密度网格（可能的噪声区域）
        low_density_threshold = np.percentile(densities, 25)  # 25%分位数
        low_density_grids = sum(1 for d in densities if d <= low_density_threshold)
        print(f"  低密度网格数量 (<= {low_density_threshold:.0f} 点): {low_density_grids}")

def test_optimized_refinement():
    """测试优化后的精细化配置"""

    # 查找测试用的地图文件
    saved_maps_dir = Path("/home/tianyu/codes/Mid360map/saved_maps")

    # 查找原始PCD文件
    original_files = list(saved_maps_dir.glob("*[!_refined].pcd"))

    if not original_files:
        print("❌ 未找到原始PCD文件进行测试")
        return False

    # 使用最新的文件进行测试
    original_file = max(original_files, key=os.path.getmtime)
    print(f"✅ 使用测试文件: {original_file}")

    # 分析原始文件密度分布
    analyze_point_density_distribution(str(original_file), "原始地图")

    # 设置输出文件路径
    output_file = saved_maps_dir / f"{original_file.stem}_optimized_refined.pcd"

    # 使用优化配置进行处理
    config_path = Path(__file__).parent / "map_refinement/config/refinement_config_optimized.yaml"

    print(f"\n🔧 开始使用优化配置进行精细化处理...")
    print(f"配置文件: {config_path}")
    print(f"输出文件: {output_file}")

    try:
        # 初始化处理管道
        pipeline = MapRefinementPipeline(str(config_path))

        # 执行处理
        start_time = time.time()
        result = pipeline.process_single_cloud(
            str(original_file),
            str(output_file),
            pipeline="custom",  # 使用自定义管道
            save_intermediate=True
        )
        processing_time = time.time() - start_time

        if result['success']:
            print(f"✅ 处理成功完成!")
            print(f"处理时间: {processing_time:.1f}s")
            print(f"点数变化: {result['original_points']:,} -> {result['final_points']:,}")
            print(f"点数变化率: {result['point_change_ratio']*100:.2f}%")

            # 分析处理后的密度分布
            analyze_point_density_distribution(str(output_file), "优化精细化后")

            # 生成质量报告
            report_path = pipeline.generate_quality_report(
                str(original_file),
                str(output_file),
                result
            )

            if report_path:
                print(f"📊 质量报告已生成: {report_path}")

            return True

        else:
            print(f"❌ 处理失败: {result.get('error', 'unknown error')}")
            return False

    except Exception as e:
        print(f"❌ 处理过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def compare_results():
    """比较原始配置和优化配置的结果"""
    saved_maps_dir = Path("/home/tianyu/codes/Mid360map/saved_maps")

    # 查找文件
    original_files = list(saved_maps_dir.glob("*[!_refined].pcd"))
    if not original_files:
        print("❌ 未找到原始文件")
        return

    original_file = max(original_files, key=os.path.getmtime)
    old_refined_file = saved_maps_dir / f"{original_file.stem}_refined.pcd"
    new_refined_file = saved_maps_dir / f"{original_file.stem}_optimized_refined.pcd"

    print("🔍 对比分析结果:")
    print("=" * 50)

    # 检查文件是否存在
    files_to_analyze = [
        (original_file, "原始地图"),
    ]

    if old_refined_file.exists():
        files_to_analyze.append((old_refined_file, "原配置精细化"))

    if new_refined_file.exists():
        files_to_analyze.append((new_refined_file, "优化配置精细化"))

    # 分析每个文件
    for file_path, name in files_to_analyze:
        analyze_point_density_distribution(str(file_path), name)

    # 对比总结
    print("\n📊 对比总结:")
    print("=" * 50)
    if old_refined_file.exists() and new_refined_file.exists():
        old_pcd = o3d.io.read_point_cloud(str(old_refined_file))
        new_pcd = o3d.io.read_point_cloud(str(new_refined_file))
        original_pcd = o3d.io.read_point_cloud(str(original_file))

        print(f"原始点数: {len(original_pcd.points):,}")
        print(f"原配置结果: {len(old_pcd.points):,} 点")
        print(f"优化配置结果: {len(new_pcd.points):,} 点")

        old_change = len(old_pcd.points) - len(original_pcd.points)
        new_change = len(new_pcd.points) - len(original_pcd.points)

        print(f"原配置点数变化: {old_change:+,} ({old_change/len(original_pcd.points)*100:+.2f}%)")
        print(f"优化配置点数变化: {new_change:+,} ({new_change/len(original_pcd.points)*100:+.2f}%)")

        if abs(new_change) < abs(old_change):
            print("✅ 优化配置减少了不必要的点数变化")
        else:
            print("⚠️  需要进一步调整配置参数")

def main():
    parser = argparse.ArgumentParser(description="测试优化后的地图精细化配置")
    parser.add_argument('--test', action='store_true', help='运行精细化测试')
    parser.add_argument('--compare', action='store_true', help='对比分析结果')
    parser.add_argument('--all', action='store_true', help='运行完整测试和对比')

    args = parser.parse_args()

    if args.all:
        print("🚀 开始完整测试流程...")
        success = test_optimized_refinement()
        if success:
            compare_results()
    elif args.test:
        test_optimized_refinement()
    elif args.compare:
        compare_results()
    else:
        print("请指定测试模式: --test, --compare, 或 --all")

if __name__ == "__main__":
    main()