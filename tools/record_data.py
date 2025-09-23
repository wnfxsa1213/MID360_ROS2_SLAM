#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLAM数据录制工具 - 使用rosbag2录制传感器数据
用法: python3 record_data.py [选项]
"""

import os
import sys
import yaml
import argparse
import subprocess
from datetime import datetime

def load_config():
    """加载配置文件"""
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'launch_config.yaml'
    )

    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    else:
        return {}

def main():
    parser = argparse.ArgumentParser(description='SLAM数据录制工具')
    parser.add_argument('-o', '--output', help='输出目录 (默认: data/rosbags)')
    parser.add_argument('-n', '--name', help='录制文件名前缀 (默认: slam_data)')
    parser.add_argument('-t', '--topics', nargs='+', help='要录制的话题列表')
    parser.add_argument('-c', '--compression', choices=['none', 'zstd'],
                       default='zstd', help='压缩格式 (默认: zstd)')
    parser.add_argument('-d', '--duration', type=int, help='录制时长(秒)')
    parser.add_argument('--all-topics', action='store_true', help='录制所有话题')

    args = parser.parse_args()

    # 加载配置
    config = load_config()
    rosbag_config = config.get('rosbag', {})

    # 设置输出目录
    output_dir = args.output or rosbag_config.get('output_dir', 'data/rosbags')
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # 生成文件名
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    name_prefix = args.name or 'slam_data'
    bag_name = f'{name_prefix}_{timestamp}'
    bag_path = os.path.join(output_dir, bag_name)

    # 映射压缩格式
    compression_format = 'fake_comp' if args.compression == 'none' else args.compression

    # 准备录制命令
    cmd = [
        'ros2', 'bag', 'record',
        '-o', bag_path,
        '--compression-mode', 'file',
        '--compression-format', compression_format
    ]

    # 添加录制时长
    if args.duration:
        cmd.extend(['-d', str(args.duration)])

    # 设置要录制的话题
    if args.all_topics:
        cmd.append('-a')  # 录制所有话题
        topics_str = "所有话题"
    else:
        # 使用指定话题或配置文件中的话题
        topics = args.topics or rosbag_config.get('topics', [
            '/livox/lidar',
            '/livox/imu',
            '/fastlio2/lio_odom',
            '/fastlio2/path'
        ])
        cmd.extend(topics)
        topics_str = ', '.join(topics)

    print(f'🎥 开始录制SLAM数据')
    print(f'📁 输出路径: {bag_path}')
    print(f'📋 录制话题: {topics_str}')
    print(f'🗜️  压缩格式: {args.compression}')
    if args.duration:
        print(f'⏱️  录制时长: {args.duration}秒')
    else:
        print(f'⏱️  录制时长: 手动停止 (Ctrl+C)')
    print(f'🚀 启动录制...')
    print('-' * 50)

    try:
        # 执行录制
        subprocess.run(cmd, check=True)
        print(f'✅ 录制完成: {bag_path}')

        # 显示文件信息
        if os.path.exists(bag_path):
            result = subprocess.run(['ros2', 'bag', 'info', bag_path],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print('\n📊 录制文件信息:')
                print(result.stdout)

    except KeyboardInterrupt:
        print(f'\n⚠️  用户停止录制')
        print(f'✅ 录制文件已保存: {bag_path}')
    except subprocess.CalledProcessError as e:
        print(f'❌ 录制失败: {e}')
        sys.exit(1)
    except Exception as e:
        print(f'❌ 发生错误: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()