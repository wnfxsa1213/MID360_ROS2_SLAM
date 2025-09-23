#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import argparse
import sys
import json
from pathlib import Path

def analyze_bag_data(bag_path):
    """分析bag文件数据质量"""

    print(f"🔍 分析bag文件: {bag_path}")

    # 连接sqlite数据库
    conn = sqlite3.connect(bag_path)
    cursor = conn.cursor()

    # 获取所有话题和消息
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
    tables = cursor.fetchall()

    # 获取topics表
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}

    analysis_results = {}

    # 分析每个话题
    for topic_id, topic_info in topics.items():
        topic_name = topic_info['name']
        print(f"\n📊 分析话题: {topic_name}")

        # 获取消息时间戳
        cursor.execute("SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
        timestamps = [row[0] for row in cursor.fetchall()]

        if not timestamps:
            continue

        # 转换为秒
        timestamps_sec = np.array(timestamps) / 1e9

        # 计算时间间隔
        if len(timestamps_sec) > 1:
            intervals = np.diff(timestamps_sec)

            analysis_results[topic_name] = {
                'message_count': int(len(timestamps)),
                'duration': float(timestamps_sec[-1] - timestamps_sec[0]),
                'avg_frequency': float(len(timestamps) / (timestamps_sec[-1] - timestamps_sec[0]) if len(timestamps) > 1 else 0),
                'avg_interval': float(np.mean(intervals)),
                'std_interval': float(np.std(intervals)),
                'min_interval': float(np.min(intervals)),
                'max_interval': float(np.max(intervals)),
                'interval_outliers': int(np.sum(intervals > (np.mean(intervals) + 3 * np.std(intervals))))
            }

            # 检查异常间隔
            outlier_threshold = np.mean(intervals) + 3 * np.std(intervals)
            outliers = intervals > outlier_threshold

            if np.any(outliers):
                print(f"  ⚠️  发现 {np.sum(outliers)} 个异常时间间隔")
                print(f"      最大间隔: {np.max(intervals):.3f}s")
                print(f"      异常阈值: {outlier_threshold:.3f}s")

            print(f"  📈 频率: {analysis_results[topic_name]['avg_frequency']:.1f} Hz")
            print(f"  ⏱️  间隔: {analysis_results[topic_name]['avg_interval']*1000:.1f}ms (±{analysis_results[topic_name]['std_interval']*1000:.1f}ms)")

    conn.close()

    # 生成报告
    generate_report(analysis_results, bag_path)

    return analysis_results

def generate_report(results, bag_path):
    """生成数据质量报告"""

    report = {
        'bag_file': str(bag_path),
        'analysis_timestamp': str(np.datetime64('now')),
        'topics': results
    }

    # 检查问题
    issues = []

    for topic_name, data in results.items():
        if '/imu' in topic_name:
            # IMU应该是200Hz
            if data['avg_frequency'] < 180:
                issues.append(f"IMU频率过低: {data['avg_frequency']:.1f}Hz < 180Hz")
            if data['std_interval'] > 0.01:  # 10ms
                issues.append(f"IMU时间抖动过大: {data['std_interval']*1000:.1f}ms")

        elif '/lidar' in topic_name:
            # LiDAR应该是10Hz
            if data['avg_frequency'] < 8:
                issues.append(f"LiDAR频率过低: {data['avg_frequency']:.1f}Hz < 8Hz")
            if data['std_interval'] > 0.05:  # 50ms
                issues.append(f"LiDAR时间抖动过大: {data['std_interval']*1000:.1f}ms")

        elif '/lio_odom' in topic_name:
            # 里程计应该是10Hz
            if data['avg_frequency'] < 8:
                issues.append(f"里程计频率过低: {data['avg_frequency']:.1f}Hz < 8Hz")

        if data['interval_outliers'] > 0:
            issues.append(f"{topic_name}: {data['interval_outliers']} 个异常时间间隔")

    report['issues'] = issues

    # 保存报告
    report_path = Path('bag_analysis_report.json')
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n📋 数据质量报告:")
    print(f"   📄 已保存到: {report_path}")

    if issues:
        print(f"\n🚨 发现 {len(issues)} 个问题:")
        for i, issue in enumerate(issues, 1):
            print(f"   {i}. {issue}")
    else:
        print("\n✅ 数据质量良好，未发现明显问题")

def analyze_imu_data(bag_path):
    """深度分析IMU数据"""

    print(f"\n🔬 深度分析IMU数据...")

    # 这里需要rosbag2_py库来解析消息内容
    # 由于环境限制，我们先返回基本分析
    print("   📊 IMU数据详细分析需要rosbag2_py库支持")
    print("   💡 建议运行: pip install rosbag2_py")

def main():
    parser = argparse.ArgumentParser(description='分析ROS2 bag文件数据质量')
    parser.add_argument('bag_path', help='bag文件路径(.db3)')
    parser.add_argument('--detailed-imu', action='store_true', help='详细分析IMU数据')

    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"❌ 文件不存在: {bag_path}")
        sys.exit(1)

    # 分析数据质量
    results = analyze_bag_data(bag_path)

    if args.detailed_imu:
        analyze_imu_data(bag_path)

if __name__ == '__main__':
    main()