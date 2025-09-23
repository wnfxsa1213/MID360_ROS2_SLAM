#!/usr/bin/env python3
"""
地图精细化工具性能基准测试

测试多线程优化效果，对比单线程和多线程性能。
"""

import os
import sys
import time
import json
import argparse
import multiprocessing
from pathlib import Path
from typing import Dict, List, Any

# 添加模块路径
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import open3d as o3d
from utils.io_utils import PointCloudIO
from refinement.noise_reducer import NoiseReducer
import logging

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class PerformanceBenchmark:
    """性能基准测试类"""

    def __init__(self, config_path: str = None):
        """初始化性能测试"""
        # 加载配置
        if config_path and os.path.exists(config_path):
            self.config = PointCloudIO.load_config(config_path)
        else:
            # 默认配置
            self.config = {
                'noise_reduction': {
                    'statistical_outlier': {'mean_k': 20, 'std_dev_mul_thresh': 2.0},
                    'radius_outlier': {'radius': 0.3, 'min_neighbors': 3},
                    'density_filter': {'min_density': 10},
                    'adaptive_denoising': {
                        'mode': 'balanced',
                        'threshold_method': 'hybrid',
                        'noise_percentile': 85,
                        'feature_weights': {
                            'curvature': 0.2,
                            'normal_variation': 0.3,
                            'density': 0.2
                        }
                    }
                },
                'performance': {
                    'num_threads': 8,
                    'memory_limit_gb': 16,
                    'progress_display': True
                }
            }

    def create_test_pointcloud(self, num_points: int = 50000) -> o3d.geometry.PointCloud:
        """创建测试点云"""
        logger.info(f"创建测试点云 - 点数: {num_points}")

        # 创建一个包含噪声的室内场景点云
        # 1. 地面平面
        ground_points = np.random.uniform(-5, 5, (num_points // 4, 3))
        ground_points[:, 2] = np.random.normal(0, 0.02, num_points // 4)  # 地面高度

        # 2. 墙面
        wall_points_1 = np.random.uniform(-5, 5, (num_points // 4, 3))
        wall_points_1[:, 0] = 5 + np.random.normal(0, 0.02, num_points // 4)  # 右墙

        wall_points_2 = np.random.uniform(-5, 5, (num_points // 4, 3))
        wall_points_2[:, 1] = 5 + np.random.normal(0, 0.02, num_points // 4)  # 前墙

        # 3. 添加噪声点
        noise_points = np.random.uniform(-6, 6, (num_points // 4, 3))

        # 合并所有点
        all_points = np.vstack([ground_points, wall_points_1, wall_points_2, noise_points])

        # 创建点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)

        # 计算法向量
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamRadius(0.1)
        )

        return pcd

    def benchmark_single_vs_multi_thread(self, test_sizes: List[int]) -> Dict[str, Any]:
        """对比单线程和多线程性能"""
        results = {
            'cpu_count': multiprocessing.cpu_count(),
            'test_sizes': test_sizes,
            'results': []
        }

        for size in test_sizes:
            logger.info(f"==== 测试点云大小: {size} ====")

            # 创建测试点云
            test_pcd = self.create_test_pointcloud(size)

            size_result = {
                'point_count': size,
                'single_thread': {},
                'multi_thread': {}
            }

            # 单线程测试
            logger.info("单线程测试...")
            single_config = self.config.copy()
            single_config['performance']['num_threads'] = 1

            start_time = time.time()
            single_reducer = NoiseReducer(single_config['noise_reduction'])
            single_result = single_reducer.adaptive_noise_removal(test_pcd)
            single_time = time.time() - start_time

            size_result['single_thread'] = {
                'time': single_time,
                'original_points': len(test_pcd.points),
                'remaining_points': len(single_result.cleaned_pcd.points),
                'removed_points': len(single_result.noise_indices),
                'removal_ratio': single_result.noise_ratio
            }

            # 多线程测试
            logger.info(f"多线程测试 (线程数: {self.config['performance']['num_threads']})...")
            multi_config = self.config.copy()

            start_time = time.time()
            multi_reducer = NoiseReducer(multi_config['noise_reduction'])
            multi_result = multi_reducer.adaptive_noise_removal(test_pcd)
            multi_time = time.time() - start_time

            size_result['multi_thread'] = {
                'time': multi_time,
                'original_points': len(test_pcd.points),
                'remaining_points': len(multi_result.cleaned_pcd.points),
                'removed_points': len(multi_result.noise_indices),
                'removal_ratio': multi_result.noise_ratio,
                'thread_count': self.config['performance']['num_threads']
            }

            # 计算性能提升
            speedup = single_time / multi_time if multi_time > 0 else 0
            size_result['speedup'] = speedup
            size_result['efficiency'] = speedup / self.config['performance']['num_threads']

            logger.info(f"性能对比:")
            logger.info(f"  单线程耗时: {single_time:.2f}秒")
            logger.info(f"  多线程耗时: {multi_time:.2f}秒")
            logger.info(f"  加速比: {speedup:.2f}x")
            logger.info(f"  效率: {size_result['efficiency']:.2%}")

            results['results'].append(size_result)

        return results

    def benchmark_thread_scaling(self, point_count: int = 50000) -> Dict[str, Any]:
        """测试线程数量对性能的影响"""
        max_threads = multiprocessing.cpu_count()
        thread_counts = [1, 2, 4] + ([8] if max_threads >= 8 else []) + ([max_threads] if max_threads > 8 else [])

        logger.info(f"==== 线程扩展性测试 (点数: {point_count}) ====")

        # 创建测试点云
        test_pcd = self.create_test_pointcloud(point_count)

        results = {
            'point_count': point_count,
            'max_cpu_count': max_threads,
            'thread_results': []
        }

        baseline_time = None

        for thread_count in thread_counts:
            logger.info(f"测试线程数: {thread_count}")

            # 配置线程数
            test_config = self.config.copy()
            test_config['performance']['num_threads'] = thread_count

            # 执行测试
            start_time = time.time()
            reducer = NoiseReducer(test_config['noise_reduction'])
            result = reducer.adaptive_noise_removal(test_pcd)
            test_time = time.time() - start_time

            if baseline_time is None:
                baseline_time = test_time

            speedup = baseline_time / test_time if test_time > 0 else 0
            efficiency = speedup / thread_count if thread_count > 0 else 0

            thread_result = {
                'thread_count': thread_count,
                'time': test_time,
                'speedup': speedup,
                'efficiency': efficiency,
                'remaining_points': len(result.cleaned_pcd.points),
                'removal_ratio': result.noise_ratio
            }

            results['thread_results'].append(thread_result)

            logger.info(f"  耗时: {test_time:.2f}秒")
            logger.info(f"  加速比: {speedup:.2f}x")
            logger.info(f"  效率: {efficiency:.2%}")

        return results

    def save_results(self, results: Dict[str, Any], output_file: str):
        """保存测试结果"""
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

        logger.info(f"测试结果已保存到: {output_file}")

    def generate_report(self, results: Dict[str, Any]) -> str:
        """生成性能报告"""
        report = []
        report.append("# 地图精细化工具多线程性能测试报告")
        report.append("")
        report.append(f"## 系统信息")
        report.append(f"- CPU核心数: {results.get('cpu_count', 'N/A')}")
        report.append(f"- 测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("")

        if 'results' in results:
            # 单线程 vs 多线程对比
            report.append("## 单线程 vs 多线程性能对比")
            report.append("")
            report.append("| 点数 | 单线程(秒) | 多线程(秒) | 加速比 | 效率 |")
            report.append("|------|------------|------------|--------|------|")

            for result in results['results']:
                single_time = result['single_thread']['time']
                multi_time = result['multi_thread']['time']
                speedup = result['speedup']
                efficiency = result['efficiency']

                report.append(f"| {result['point_count']:,} | {single_time:.2f} | {multi_time:.2f} | {speedup:.2f}x | {efficiency:.1%} |")

        if 'thread_results' in results:
            # 线程扩展性测试
            report.append("")
            report.append("## 线程扩展性测试")
            report.append("")
            report.append("| 线程数 | 耗时(秒) | 加速比 | 效率 |")
            report.append("|--------|----------|--------|------|")

            for result in results['thread_results']:
                thread_count = result['thread_count']
                time_taken = result['time']
                speedup = result['speedup']
                efficiency = result['efficiency']

                report.append(f"| {thread_count} | {time_taken:.2f} | {speedup:.2f}x | {efficiency:.1%} |")

        return "\n".join(report)

def main():
    parser = argparse.ArgumentParser(description='地图精细化工具性能基准测试')
    parser.add_argument('--config', help='配置文件路径')
    parser.add_argument('--output-dir', default='benchmark_results', help='结果输出目录')
    parser.add_argument('--test-sizes', nargs='+', type=int,
                       default=[10000, 25000, 50000, 100000],
                       help='测试点云大小列表')
    parser.add_argument('--scaling-test-size', type=int, default=50000,
                       help='线程扩展性测试使用的点云大小')

    args = parser.parse_args()

    # 创建输出目录
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    # 初始化基准测试
    benchmark = PerformanceBenchmark(args.config)

    logger.info("开始性能基准测试...")

    # 执行单线程 vs 多线程对比测试
    logger.info("执行单线程 vs 多线程对比测试")
    comparison_results = benchmark.benchmark_single_vs_multi_thread(args.test_sizes)

    # 保存对比测试结果
    comparison_file = output_dir / 'single_vs_multi_thread.json'
    benchmark.save_results(comparison_results, str(comparison_file))

    # 执行线程扩展性测试
    logger.info("执行线程扩展性测试")
    scaling_results = benchmark.benchmark_thread_scaling(args.scaling_test_size)

    # 保存扩展性测试结果
    scaling_file = output_dir / 'thread_scaling.json'
    benchmark.save_results(scaling_results, str(scaling_file))

    # 生成综合报告
    logger.info("生成性能报告")

    # 合并结果用于报告生成
    combined_results = {
        **comparison_results,
        **scaling_results
    }

    report = benchmark.generate_report(combined_results)

    # 保存报告
    report_file = output_dir / 'performance_report.md'
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(report)

    logger.info(f"性能测试完成!")
    logger.info(f"结果已保存到: {output_dir}")
    logger.info(f"详细报告: {report_file}")

    # 显示简要结果
    print("\n" + "="*60)
    print("性能测试简要结果:")
    print("="*60)

    if comparison_results['results']:
        best_speedup = max(r['speedup'] for r in comparison_results['results'])
        avg_speedup = np.mean([r['speedup'] for r in comparison_results['results']])
        print(f"最佳加速比: {best_speedup:.2f}x")
        print(f"平均加速比: {avg_speedup:.2f}x")

    if scaling_results['thread_results']:
        max_threads_result = max(scaling_results['thread_results'], key=lambda x: x['thread_count'])
        print(f"最大线程数测试 ({max_threads_result['thread_count']}线程): {max_threads_result['speedup']:.2f}x 加速")
        print(f"线程效率: {max_threads_result['efficiency']:.1%}")

if __name__ == '__main__':
    main()