#!/usr/bin/env python3
"""
地图精细化工具性能基准测试 - 便捷接口

为slam_tools.sh提供简单的性能测试接口
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='地图精细化性能测试')
    parser.add_argument('--quick', action='store_true', help='快速测试 (较小数据集)')
    parser.add_argument('--full', action='store_true', help='完整测试 (大数据集)')
    parser.add_argument('--config', help='配置文件路径')

    args = parser.parse_args()

    # 脚本路径
    script_dir = Path(__file__).parent
    benchmark_script = script_dir / 'map_refinement' / 'performance_benchmark.py'

    if not benchmark_script.exists():
        print(f"❌ 基准测试脚本不存在: {benchmark_script}")
        return 1

    # 构建命令
    cmd = ['python3', str(benchmark_script)]

    if args.config:
        cmd.extend(['--config', args.config])

    if args.quick:
        # 快速测试：较小的数据集
        cmd.extend(['--test-sizes', '5000', '15000', '30000'])
        cmd.extend(['--scaling-test-size', '25000'])
        print("🚀 执行快速性能测试...")
    elif args.full:
        # 完整测试：大数据集
        cmd.extend(['--test-sizes', '10000', '50000', '100000', '200000'])
        cmd.extend(['--scaling-test-size', '100000'])
        print("🔬 执行完整性能测试...")
    else:
        # 默认测试
        print("⚡ 执行标准性能测试...")

    # 设置输出目录
    output_dir = script_dir.parent / 'benchmark_results'
    cmd.extend(['--output-dir', str(output_dir)])

    print(f"结果将保存到: {output_dir}")
    print("开始测试...")

    # 执行测试
    try:
        result = subprocess.run(cmd, check=True)
        print("✅ 性能测试完成!")

        # 显示结果文件
        if output_dir.exists():
            print(f"\n📊 测试结果:")
            for file in output_dir.glob('*'):
                print(f"  • {file.name}")

        return 0
    except subprocess.CalledProcessError as e:
        print(f"❌ 性能测试失败: {e}")
        return 1
    except Exception as e:
        print(f"❌ 执行错误: {e}")
        return 1

if __name__ == '__main__':
    exit(main())