#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import sys
from pathlib import Path
import time

def extract_raw_sensors(input_bag, output_dir):
    """提取原始传感器数据"""

    print(f"🔄 提取原始传感器数据...")
    print(f"   输入: {input_bag}")
    print(f"   输出: {output_dir}")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 使用ros2 bag filter命令（如果可用）
    filter_cmd = [
        'ros2', 'bag', 'filter',
        str(input_bag),
        str(output_dir / 'raw_sensors'),
        "topic == '/livox/imu' or topic == '/livox/lidar' or topic == '/tf_static'"
    ]

    try:
        print("🚀 尝试使用ros2 bag filter...")
        result = subprocess.run(filter_cmd, capture_output=True, text=True, timeout=300)

        if result.returncode == 0:
            print("✅ 原始数据提取成功!")
            return output_dir / 'raw_sensors'
        else:
            print(f"❌ filter命令失败: {result.stderr}")
            return None

    except subprocess.TimeoutExpired:
        print("⏰ 提取超时")
        return None
    except FileNotFoundError:
        print("❌ ros2 bag filter不可用")
        return None
    except Exception as e:
        print(f"❌ 提取失败: {e}")
        return None

def play_with_fastlio2(bag_path):
    """使用优化参数运行FastLIO2处理原始数据"""

    print(f"\n🚀 使用FastLIO2重新处理数据...")
    print(f"   数据包: {bag_path}")

    # 启动FastLIO2
    fastlio_cmd = [
        'ros2', 'launch', 'fastlio2', 'cooperative_slam_system.launch.py'
    ]

    print("1. 启动FastLIO2...")
    fastlio_process = subprocess.Popen(fastlio_cmd)

    # 等待FastLIO2初始化
    time.sleep(5)

    try:
        # 播放原始数据
        play_cmd = [
            'ros2', 'bag', 'play', str(bag_path),
            '--rate', '0.8',  # 慢速播放确保稳定
            '--clock'
        ]

        print("2. 播放原始数据...")
        play_result = subprocess.run(play_cmd, timeout=300)

        if play_result.returncode == 0:
            print("✅ 数据播放完成!")
        else:
            print("❌ 播放过程出现问题")

    except subprocess.TimeoutExpired:
        print("⏰ 播放超时")
    except KeyboardInterrupt:
        print("⏹️ 用户中断")
    finally:
        # 停止FastLIO2
        print("3. 停止FastLIO2...")
        fastlio_process.terminate()
        fastlio_process.wait(timeout=10)

def main():
    if len(sys.argv) != 3:
        print("用法: python3 extract_raw_sensors.py <input_bag> <output_dir>")
        print("例如: python3 extract_raw_sensors.py data/rosbags/slam_data.db3 data/clean_bags")
        sys.exit(1)

    input_bag = Path(sys.argv[1])
    output_dir = Path(sys.argv[2])

    if not input_bag.exists():
        print(f"❌ 输入文件不存在: {input_bag}")
        sys.exit(1)

    # Step 1: 提取原始传感器数据
    raw_bag = extract_raw_sensors(input_bag, output_dir)

    if raw_bag and raw_bag.exists():
        print(f"\n📊 检查提取的数据...")
        info_cmd = ['ros2', 'bag', 'info', str(raw_bag)]
        subprocess.run(info_cmd)

        # Step 2: 询问是否重新运行FastLIO2
        choice = input("\n🤖 是否使用FastLIO2重新处理这些数据? (y/n): ")
        if choice.lower() == 'y':
            play_with_fastlio2(raw_bag)
    else:
        print("❌ 数据提取失败")

if __name__ == '__main__':
    main()