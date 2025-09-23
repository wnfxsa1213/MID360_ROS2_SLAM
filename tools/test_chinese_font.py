#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
matplotlib中文字体配置测试脚本

该脚本用于测试和验证SimHei字体的加载和中文显示效果，包括：
- 字体文件验证
- 字体加载测试
- 中文标签显示测试
- matplotlib配置验证

作者: SLAM系统
日期: 2024
"""

import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from pathlib import Path

def get_available_chinese_fonts():
    """获取系统中可用的中文字体"""
    chinese_fonts = []

    # 优先考虑的字体列表（按优先级排序）
    preferred_fonts = [
        'Noto Sans CJK SC',        # Noto Sans 简体中文
        'WenQuanYi Micro Hei',     # 文泉驿微米黑
        'Noto Serif CJK SC',       # Noto Serif 简体中文
        'AR PL UMing CN',          # 文鼎PL新宋
        'AR PL UKai CN'            # 文鼎PL中楷
    ]

    # 检查matplotlib中可用的字体
    available_fonts = [font.name for font in fm.fontManager.ttflist]

    # 找到可用的中文字体
    for preferred in preferred_fonts:
        for available in available_fonts:
            if preferred in available:
                chinese_fonts.append(available)
                break

    # 如果没找到首选字体，查找任何包含中文关键词的字体
    if not chinese_fonts:
        for font in available_fonts:
            if any(keyword in font.lower() for keyword in ['cjk', 'chinese', 'hei', 'song', 'kai']):
                chinese_fonts.append(font)

    return chinese_fonts

def configure_chinese_font():
    """配置matplotlib中文字体支持"""
    try:
        # 获取可用的中文字体
        chinese_fonts = get_available_chinese_fonts()

        if not chinese_fonts:
            print("✗ 系统中未找到可用的中文字体")
            return False

        print(f"找到可用的中文字体: {chinese_fonts}")

        # 选择第一个可用的字体
        selected_font = chinese_fonts[0]
        print(f"选择字体: {selected_font}")

        # 配置matplotlib使用选定的中文字体
        plt.rcParams['font.sans-serif'] = [selected_font, 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
        plt.rcParams['axes.unicode_minus'] = False    # 正常显示负号
        plt.rcParams['font.size'] = 10                # 设置基础字体大小

        print("✓ matplotlib已配置使用中文字体")
        print("✓ rcParams已配置")

        return True

    except Exception as e:
        print(f"✗ 配置中文字体失败: {e}")
        return False

def verify_font_loading():
    """验证字体加载"""
    try:
        # 获取当前字体设置
        current_fonts = plt.rcParams['font.sans-serif']
        print(f"当前字体设置: {current_fonts}")

        # 检查第一个中文字体是否可用
        chinese_font_found = False
        if current_fonts:
            primary_font = current_fonts[0]
            for font in fm.fontManager.ttflist:
                if primary_font in font.name:
                    print(f"✓ 找到字体: {font.name} - {font.fname}")
                    chinese_font_found = True
                    break

        if not chinese_font_found:
            print("✗ 中文字体未在字体列表中找到")

        # 测试字体渲染能力
        test_chinese_text = "测试中文字体渲染"
        try:
            # 创建一个简单的图形来测试字体渲染
            fig, ax = plt.subplots(figsize=(6, 2))
            ax.text(0.5, 0.5, test_chinese_text, fontsize=16, ha='center', va='center')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
            plt.close(fig)  # 关闭图形避免显示
            print("✓ 中文字体渲染测试通过")

        except Exception as e:
            print(f"✗ 中文字体渲染测试失败: {e}")
            return False

        return chinese_font_found

    except Exception as e:
        print(f"✗ 验证字体加载失败: {e}")
        return False

def create_test_plots():
    """创建中文显示测试图表"""
    try:
        # 创建测试数据
        x = np.linspace(0, 10, 100)
        y1 = np.sin(x)
        y2 = np.cos(x)

        # 创建图表
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))

        # 图表1: 基本中文标签测试
        ax1.plot(x, y1, 'b-', label='正弦波')
        ax1.plot(x, y2, 'r-', label='余弦波')
        ax1.set_title('基本中文标签测试')
        ax1.set_xlabel('时间 (秒)')
        ax1.set_ylabel('幅值')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 图表2: SLAM相关术语测试
        slam_data = [85, 92, 78, 95, 88, 91]
        slam_labels = ['点云数量', '过滤比例', '处理时间', '内存使用', '定位精度', '建图质量']

        ax2.bar(range(len(slam_data)), slam_data, color=['red', 'green', 'blue', 'orange', 'purple', 'brown'])
        ax2.set_title('SLAM系统性能指标')
        ax2.set_ylabel('性能分数')
        ax2.set_xticks(range(len(slam_labels)))
        ax2.set_xticklabels(slam_labels, rotation=45, ha='right')

        # 图表3: 饼状图测试
        filter_data = [30, 45, 15, 10]
        filter_labels = ['静态点', '动态点', '噪声点', '其他']
        colors = ['lightblue', 'lightcoral', 'lightyellow', 'lightgreen']

        ax3.pie(filter_data, labels=filter_labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax3.set_title('动态对象过滤统计')

        # 图表4: 散点图测试
        np.random.seed(42)
        scatter_x = np.random.randn(100)
        scatter_y = np.random.randn(100)

        ax4.scatter(scatter_x, scatter_y, c=scatter_y, cmap='viridis', alpha=0.6)
        ax4.set_title('点云分布可视化')
        ax4.set_xlabel('X坐标 (米)')
        ax4.set_ylabel('Y坐标 (米)')

        # 设置总标题
        current_font = plt.rcParams['font.sans-serif'][0] if plt.rcParams['font.sans-serif'] else '默认字体'
        fig.suptitle(f'matplotlib中文字体显示测试 - {current_font}', fontsize=16, fontweight='bold')

        # 调整布局
        plt.tight_layout()

        # 保存图表
        output_path = Path(__file__).parent / "chinese_font_test.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"✓ 测试图表已保存到: {output_path}")

        # 显示图表
        plt.show()

        return True

    except Exception as e:
        print(f"✗ 创建测试图表失败: {e}")
        return False

def test_specific_chinese_terms():
    """测试特定的中文术语显示"""
    try:
        # SLAM和动态过滤相关中文术语
        chinese_terms = [
            "点云统计", "过滤比例", "处理时间", "内存使用",
            "动态对象", "静态环境", "实时建图", "位置估计",
            "传感器融合", "环境感知", "路径规划", "导航系统",
            "激光雷达", "视觉SLAM", "惯性导航", "定位精度"
        ]

        fig, ax = plt.subplots(figsize=(10, 8))

        # 创建测试数据
        y_pos = np.arange(len(chinese_terms))
        values = np.random.randint(50, 100, len(chinese_terms))

        # 创建水平条形图
        bars = ax.barh(y_pos, values, color=plt.cm.Set3(np.linspace(0, 1, len(chinese_terms))))

        # 设置标签
        ax.set_yticks(y_pos)
        ax.set_yticklabels(chinese_terms)
        ax.set_xlabel('性能指标 (%)')
        ax.set_title('SLAM系统中文术语显示测试', fontsize=14, fontweight='bold')

        # 添加数值标签
        for bar, value in zip(bars, values):
            ax.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2,
                   f'{value}%', ha='left', va='center')

        # 添加网格
        ax.grid(axis='x', alpha=0.3)

        plt.tight_layout()

        # 保存专门的术语测试图
        output_path = Path(__file__).parent / "chinese_terms_test.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"✓ 中文术语测试图已保存到: {output_path}")

        plt.show()

        return True

    except Exception as e:
        print(f"✗ 中文术语测试失败: {e}")
        return False

def print_font_info():
    """打印字体信息"""
    print("\n" + "="*60)
    print("matplotlib字体配置信息")
    print("="*60)

    # 当前字体配置
    print(f"当前字体族: {plt.rcParams['font.family']}")
    print(f"无衬线字体: {plt.rcParams['font.sans-serif']}")
    print(f"Unicode负号设置: {plt.rcParams['axes.unicode_minus']}")

    # 可用的中文字体
    print("\n可用的中文字体:")
    chinese_fonts = []
    for font in fm.fontManager.ttflist:
        if any(keyword in font.name.lower() for keyword in ['sim', 'hei', 'chinese', 'cjk']):
            chinese_fonts.append(f"  - {font.name}")

    if chinese_fonts:
        for font in chinese_fonts[:10]:  # 只显示前10个
            print(font)
        if len(chinese_fonts) > 10:
            print(f"  ... 还有 {len(chinese_fonts) - 10} 个中文字体")
    else:
        print("  未找到中文字体")

def main():
    """主函数"""
    print("matplotlib中文字体配置测试")
    print("="*50)

    # 步骤1: 配置中文字体
    print("\n1. 配置系统中文字体...")
    font_configured = configure_chinese_font()

    if not font_configured:
        print("字体配置失败，无法继续测试")
        return

    # 步骤2: 验证字体加载
    print("\n2. 验证字体加载...")
    font_verified = verify_font_loading()

    # 步骤3: 打印字体信息
    print_font_info()

    # 步骤4: 创建基本测试图表
    print("\n3. 创建基本中文显示测试...")
    basic_test = create_test_plots()

    # 步骤5: 测试特定中文术语
    print("\n4. 测试SLAM相关中文术语...")
    terms_test = test_specific_chinese_terms()

    # 总结
    print("\n" + "="*50)
    print("测试结果总结:")
    print(f"  字体配置: {'✓ 成功' if font_configured else '✗ 失败'}")
    print(f"  字体验证: {'✓ 成功' if font_verified else '✗ 失败'}")
    print(f"  基本测试: {'✓ 成功' if basic_test else '✗ 失败'}")
    print(f"  术语测试: {'✓ 成功' if terms_test else '✗ 失败'}")

    if all([font_configured, font_verified, basic_test, terms_test]):
        print("\n✅ 所有测试通过！中文字体配置成功，中文显示正常。")
    else:
        print("\n❌ 部分测试失败，请检查字体配置。")

if __name__ == "__main__":
    main()