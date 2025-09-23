#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import subprocess
import sys
from pathlib import Path
import signal
import time
from datetime import datetime

def signal_handler(sig, frame):
    """优雅退出信号处理"""
    print("\n📦 录制中断，正在保存数据...")
    sys.exit(0)

def record_bag(topics, output_dir, bag_name=None, duration=None, compress=False):
    """录制ROS2 bag数据"""

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 创建输出目录
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # 生成bag文件名
    if not bag_name:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"recorded_data_{timestamp}"

    bag_path = output_path / bag_name

    # 构建录制命令
    cmd = [
        'ros2', 'bag', 'record',
        '-o', str(bag_path)
    ]

    # 添加话题
    if topics:
        cmd.extend(topics)
    else:
        cmd.append('-a')  # 录制所有话题

    # 添加录制时长
    if duration:
        cmd.extend(['--max-duration', str(duration)])

    # 添加压缩选项
    if compress:
        cmd.extend(['--compression-mode', 'file'])
        cmd.extend(['--compression-format', 'zstd'])

    print(f"📦 开始录制bag数据...")
    print(f"   输出路径: {bag_path}")
    print(f"   话题列表: {topics if topics else '所有话题'}")
    if duration:
        print(f"   录制时长: {duration}秒")
    print(f"   压缩模式: {'启用' if compress else '禁用'}")
    print(f"\n🎯 执行命令: {' '.join(cmd)}")
    print("   按 Ctrl+C 停止录制\n")

    try:
        # 启动录制进程
        process = subprocess.Popen(cmd)

        # 等待进程完成或用户中断
        if duration:
            # 有时长限制，等待指定时间
            for i in range(duration):
                time.sleep(1)
                if process.poll() is not None:
                    break
                print(f"\r⏱️  录制中... {i+1}/{duration}s", end="", flush=True)
            print()
        else:
            # 无时长限制，等待用户中断
            process.wait()

        print(f"\n✅ 录制完成!")
        print(f"   数据保存到: {bag_path}")

        # 显示录制信息
        info_cmd = ['ros2', 'bag', 'info', str(bag_path)]
        print(f"\n📊 录制信息:")
        subprocess.run(info_cmd)

        return bag_path

    except KeyboardInterrupt:
        print(f"\n📦 用户中断录制")
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        return bag_path
    except Exception as e:
        print(f"❌ 录制失败: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(
        description='ROS2 bag数据录制工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 录制所有话题
  python3 record_bag.py -o data/bags

  # 录制指定话题
  python3 record_bag.py -t /livox/imu /livox/lidar -o data/bags

  # 录制30秒并压缩
  python3 record_bag.py -t /livox/imu /livox/lidar -o data/bags -d 30 -c

  # 自定义bag名称
  python3 record_bag.py -t /livox/imu /livox/lidar -o data/bags -n my_test_data
        """
    )

    parser.add_argument('-t', '--topics', nargs='+',
                       help='要录制的话题列表 (默认录制所有话题)')
    parser.add_argument('-o', '--output-dir', required=True,
                       help='输出目录路径')
    parser.add_argument('-n', '--name',
                       help='bag文件名 (默认使用时间戳)')
    parser.add_argument('-d', '--duration', type=int,
                       help='录制时长(秒) (默认无限制)')
    parser.add_argument('-c', '--compress', action='store_true',
                       help='启用压缩 (zstd格式)')
    parser.add_argument('--list-topics', action='store_true',
                       help='列出当前可用话题')

    args = parser.parse_args()

    # 列出话题选项
    if args.list_topics:
        print("📋 当前可用话题:")
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for topic in result.stdout.strip().split('\n'):
                    print(f"   {topic}")
            else:
                print("❌ 无法获取话题列表")
        except Exception as e:
            print(f"❌ 获取话题列表失败: {e}")
        return

    # 开始录制
    bag_path = record_bag(
        topics=args.topics,
        output_dir=args.output_dir,
        bag_name=args.name,
        duration=args.duration,
        compress=args.compress
    )

    if bag_path and bag_path.exists():
        print(f"\n🎉 录制成功完成!")

        # 提供后续建议
        print(f"\n💡 后续操作建议:")
        print(f"   分析数据质量: python3 tools/bag_data_analyzer.py {bag_path}")
        print(f"   播放数据: ros2 bag play {bag_path}")
        if args.topics and '/livox/imu' in args.topics:
            print(f"   监控数据: python3 tools/imu_odom_monitor.py")
    else:
        print(f"\n❌ 录制失败或被中断")
        sys.exit(1)

if __name__ == '__main__':
    main()