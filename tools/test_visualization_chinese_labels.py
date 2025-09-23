#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化工具中文标签显示测试

该脚本模拟动态对象过滤器可视化工具的实际使用场景，
测试所有中文标签在matplotlib图表中的显示效果。

作者: SLAM系统
日期: 2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.animation import FuncAnimation
from datetime import datetime, timedelta
import sys
import os

# 导入中文字体配置
try:
    from chinese_font_config import configure_chinese_font, get_current_font, print_font_status
    print("正在配置中文字体...")
    font_configured = configure_chinese_font()
    if font_configured:
        print(f"✓ 中文字体配置成功: {get_current_font()}")
    else:
        print("✗ 中文字体配置失败")
        sys.exit(1)
except ImportError as e:
    print(f"✗ 无法导入中文字体配置模块: {e}")
    sys.exit(1)

# 与原始可视化工具相同的标签定义
LABELS = {
    'filter_ratio': '过滤比例',
    'processing_time': '处理时间(ms)',
    'memory_usage': '内存使用(MB)',
    'point_count': '点云数量',
    'time': '时间',
    'ratio': '比例',
    'ms': 'ms',
    'mb': 'MB'
}

# 扩展的中文标签用于完整测试
EXTENDED_LABELS = {
    # 基础标签
    'filter_ratio': '过滤比例',
    'processing_time': '处理时间(ms)',
    'memory_usage': '内存使用(MB)',
    'point_count': '点云数量',
    'time': '时间',
    'ratio': '比例',

    # 统计相关
    'total_points': '总点数',
    'dynamic_points': '动态点数',
    'static_points': '静态点数',
    'filtered_points': '已过滤点数',

    # 性能相关
    'avg_processing_time': '平均处理时间',
    'max_memory_usage': '最大内存使用',
    'cpu_usage': 'CPU使用率',
    'fps': '帧率',

    # SLAM相关
    'localization_accuracy': '定位精度',
    'mapping_quality': '建图质量',
    'trajectory_length': '轨迹长度',
    'loop_closure': '回环检测',

    # 传感器相关
    'lidar_frequency': '激光雷达频率',
    'imu_frequency': 'IMU频率',
    'sensor_fusion': '传感器融合',

    # 环境相关
    'dynamic_objects': '动态对象',
    'static_environment': '静态环境',
    'obstacle_detection': '障碍物检测',
    'free_space': '自由空间'
}

def generate_sample_data(num_points=50):
    """生成示例数据"""
    # 时间序列
    start_time = datetime.now() - timedelta(minutes=num_points)
    timestamps = [start_time + timedelta(minutes=i) for i in range(num_points)]

    # 模拟数据
    data = {
        'timestamps': timestamps,
        'point_counts': np.random.randint(8000, 12000, num_points),
        'filter_ratios': np.random.uniform(0.1, 0.9, num_points),
        'processing_times': np.random.uniform(8, 25, num_points),
        'memory_usage': np.random.uniform(150, 300, num_points),
        'dynamic_points': np.random.randint(500, 2000, num_points),
        'static_points': np.random.randint(6000, 10000, num_points)
    }

    return data

def test_realtime_charts():
    """测试实时统计图表（模拟原始可视化工具的图表样式）"""
    print("测试实时统计图表...")

    data = generate_sample_data()

    # 创建2x2子图布局（与原始工具相同）
    fig, ((ax_points, ax_ratio), (ax_time, ax_memory)) = plt.subplots(2, 2, figsize=(12, 8))

    fig.suptitle("动态过滤器实时统计", fontsize=14, fontweight='bold')

    # 点云统计图
    ax_points.plot(data['timestamps'], data['point_counts'], 'b-', linewidth=2, label='总点数')
    ax_points.plot(data['timestamps'], data['static_points'], 'g-', linewidth=2, label='静态点数')
    ax_points.plot(data['timestamps'], data['dynamic_points'], 'r-', linewidth=2, label='动态点数')
    ax_points.set_title("点云统计")
    ax_points.set_ylabel("点数")
    ax_points.legend()
    ax_points.grid(True, alpha=0.3)

    # 过滤比例图
    ax_ratio.plot(data['timestamps'], data['filter_ratios'], 'r-', linewidth=2)
    ax_ratio.set_title("过滤比例")
    ax_ratio.set_ylabel("比例")
    ax_ratio.set_ylim(0, 1)
    ax_ratio.grid(True, alpha=0.3)

    # 处理时间图
    ax_time.plot(data['timestamps'], data['processing_times'], 'g-', linewidth=2)
    ax_time.set_title("处理时间")
    ax_time.set_ylabel("时间 (ms)")
    ax_time.grid(True, alpha=0.3)

    # 内存使用图
    ax_memory.plot(data['timestamps'], data['memory_usage'], 'm-', linewidth=2)
    ax_memory.set_title("内存使用")
    ax_memory.set_ylabel("内存 (MB)")
    ax_memory.grid(True, alpha=0.3)

    # 格式化时间轴
    for ax in [ax_points, ax_ratio, ax_time, ax_memory]:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=10))
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()

    # 保存图表
    output_path = "visualization_chinese_test_realtime.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ 实时统计图表已保存: {output_path}")

    plt.show()

def test_performance_charts():
    """测试性能监控图表"""
    print("测试性能监控图表...")

    data = generate_sample_data()

    fig, (ax_time, ax_memory) = plt.subplots(2, 1, figsize=(12, 8))

    fig.suptitle("性能监控", fontsize=14, fontweight='bold')

    # 处理时间趋势
    ax_time.plot(data['timestamps'], data['processing_times'], 'b-', linewidth=2)
    avg_time = np.mean(data['processing_times'])
    ax_time.axhline(y=avg_time, color='r', linestyle='--', linewidth=2,
                   label=f'平均: {avg_time:.2f}ms')
    ax_time.set_title("处理时间趋势")
    ax_time.set_ylabel("时间 (ms)")
    ax_time.legend()
    ax_time.grid(True, alpha=0.3)

    # 内存使用趋势
    ax_memory.plot(data['timestamps'], data['memory_usage'], 'g-', linewidth=2)
    max_memory = np.max(data['memory_usage'])
    ax_memory.axhline(y=max_memory, color='r', linestyle='--', linewidth=2,
                     label=f'最大: {max_memory:.2f}MB')
    ax_memory.set_title("内存使用趋势")
    ax_memory.set_ylabel("内存 (MB)")
    ax_memory.legend()
    ax_memory.grid(True, alpha=0.3)

    # 格式化时间轴
    for ax in [ax_time, ax_memory]:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=10))
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()

    # 保存图表
    output_path = "visualization_chinese_test_performance.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ 性能监控图表已保存: {output_path}")

    plt.show()

def test_pointcloud_comparison():
    """测试点云对比图表"""
    print("测试点云对比图表...")

    # 生成模拟点云数据
    n_points_raw = 5000
    n_points_filtered = 3000

    # 原始点云（模拟）
    raw_points = np.random.randn(n_points_raw, 2) * 5

    # 过滤后点云（去除一些点）
    indices = np.random.choice(n_points_raw, n_points_filtered, replace=False)
    filtered_points = raw_points[indices]

    fig, (ax_raw, ax_filtered) = plt.subplots(1, 2, figsize=(12, 6))

    fig.suptitle("点云对比 (X-Y平面投影)", fontsize=14, fontweight='bold')

    # 原始点云
    ax_raw.scatter(raw_points[:, 0], raw_points[:, 1], c='red', s=1, alpha=0.6)
    ax_raw.set_title(f"原始点云 ({n_points_raw} 点)")
    ax_raw.set_xlabel("X (m)")
    ax_raw.set_ylabel("Y (m)")
    ax_raw.set_aspect('equal')
    ax_raw.grid(True, alpha=0.3)

    # 过滤后点云
    ax_filtered.scatter(filtered_points[:, 0], filtered_points[:, 1], c='green', s=1, alpha=0.6)
    ax_filtered.set_title(f"过滤后点云 ({n_points_filtered} 点)")
    ax_filtered.set_xlabel("X (m)")
    ax_filtered.set_ylabel("Y (m)")
    ax_filtered.set_aspect('equal')
    ax_filtered.grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存图表
    output_path = "visualization_chinese_test_pointcloud.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ 点云对比图表已保存: {output_path}")

    plt.show()

def test_all_chinese_labels():
    """测试所有中文标签"""
    print("测试扩展中文标签...")

    # 创建一个综合的标签测试图表
    fig, ax = plt.subplots(figsize=(14, 10))

    # 创建测试数据
    labels = list(EXTENDED_LABELS.values())
    values = np.random.uniform(20, 100, len(labels))

    # 创建水平条形图
    y_pos = np.arange(len(labels))
    bars = ax.barh(y_pos, values, color=plt.cm.Set3(np.linspace(0, 1, len(labels))))

    # 设置标签
    ax.set_yticks(y_pos)
    ax.set_yticklabels(labels, fontsize=10)
    ax.set_xlabel('性能指标 (%)', fontsize=12)
    ax.set_title('SLAM系统完整中文标签显示测试', fontsize=16, fontweight='bold')

    # 添加数值标签
    for bar, value, label in zip(bars, values, labels):
        ax.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2,
               f'{value:.1f}%', ha='left', va='center', fontsize=9)

    # 添加网格
    ax.grid(axis='x', alpha=0.3)

    # 设置布局
    plt.tight_layout()

    # 保存图表
    output_path = "visualization_chinese_test_all_labels.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ 完整标签测试图表已保存: {output_path}")

    plt.show()

def test_mixed_chart_types():
    """测试混合图表类型"""
    print("测试混合图表类型...")

    fig = plt.figure(figsize=(15, 10))

    # 创建网格布局
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    # 1. 线图
    ax1 = fig.add_subplot(gs[0, 0])
    x = np.linspace(0, 10, 50)
    ax1.plot(x, np.sin(x), 'b-', label='正弦波')
    ax1.set_title('线图测试')
    ax1.set_xlabel('时间')
    ax1.set_ylabel('幅值')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. 柱状图
    ax2 = fig.add_subplot(gs[0, 1])
    categories = ['静态点', '动态点', '噪声点']
    values = [65, 25, 10]
    ax2.bar(categories, values, color=['green', 'red', 'orange'])
    ax2.set_title('柱状图测试')
    ax2.set_ylabel('百分比 (%)')

    # 3. 饼图
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.pie(values, labels=categories, autopct='%1.1f%%', startangle=90)
    ax3.set_title('饼图测试')

    # 4. 散点图
    ax4 = fig.add_subplot(gs[1, 0])
    x_scatter = np.random.randn(100)
    y_scatter = np.random.randn(100)
    ax4.scatter(x_scatter, y_scatter, alpha=0.6)
    ax4.set_title('散点图测试')
    ax4.set_xlabel('X坐标')
    ax4.set_ylabel('Y坐标')

    # 5. 直方图
    ax5 = fig.add_subplot(gs[1, 1])
    data_hist = np.random.normal(100, 15, 1000)
    ax5.hist(data_hist, bins=30, alpha=0.7, color='skyblue')
    ax5.set_title('直方图测试')
    ax5.set_xlabel('处理时间 (ms)')
    ax5.set_ylabel('频次')

    # 6. 箱型图
    ax6 = fig.add_subplot(gs[1, 2])
    data_box = [np.random.normal(0, std, 100) for std in range(1, 4)]
    ax6.boxplot(data_box, labels=['低噪声', '中噪声', '高噪声'])
    ax6.set_title('箱型图测试')
    ax6.set_ylabel('噪声水平')

    # 7. 热力图
    ax7 = fig.add_subplot(gs[2, :])
    data_heatmap = np.random.rand(5, 10)
    im = ax7.imshow(data_heatmap, cmap='viridis', aspect='auto')
    ax7.set_title('性能监控热力图')
    ax7.set_xlabel('时间步长')
    ax7.set_ylabel('性能指标')

    # 设置热力图的刻度标签
    y_labels = ['CPU使用率', '内存使用率', '网络带宽', '磁盘I/O', '处理延迟']
    ax7.set_yticks(range(len(y_labels)))
    ax7.set_yticklabels(y_labels)

    # 添加颜色条
    plt.colorbar(im, ax=ax7, label='性能值')

    # 设置总标题
    fig.suptitle('matplotlib中文字体混合图表类型测试', fontsize=16, fontweight='bold')

    # 保存图表
    output_path = "visualization_chinese_test_mixed_charts.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ 混合图表测试已保存: {output_path}")

    plt.show()

def run_all_tests():
    """运行所有测试"""
    print("=" * 60)
    print("可视化工具中文标签显示测试")
    print("=" * 60)

    # 显示字体状态
    print("\n字体配置状态:")
    print_font_status()

    print("\n开始测试...")

    try:
        # 1. 测试实时统计图表
        test_realtime_charts()

        # 2. 测试性能监控图表
        test_performance_charts()

        # 3. 测试点云对比图表
        test_pointcloud_comparison()

        # 4. 测试所有中文标签
        test_all_chinese_labels()

        # 5. 测试混合图表类型
        test_mixed_chart_types()

        print("\n" + "=" * 60)
        print("✅ 所有测试完成！")
        print("生成的测试图片:")
        test_files = [
            "visualization_chinese_test_realtime.png",
            "visualization_chinese_test_performance.png",
            "visualization_chinese_test_pointcloud.png",
            "visualization_chinese_test_all_labels.png",
            "visualization_chinese_test_mixed_charts.png"
        ]

        for i, filename in enumerate(test_files, 1):
            if os.path.exists(filename):
                print(f"  {i}. {filename}")

        print("\n请检查生成的图片文件，验证中文标签是否正确显示。")

    except Exception as e:
        print(f"\n❌ 测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    run_all_tests()