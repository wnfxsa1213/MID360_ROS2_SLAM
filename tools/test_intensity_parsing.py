#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试PCD强度解析功能的脚本
"""

import os
import sys
import numpy as np

def test_load_pcd_with_intensity(file_path):
    """
    测试PCD文件强度解析功能
    """
    print(f'🧪 测试PCD强度解析: {os.path.basename(file_path)}')
    print('=' * 60)

    intensity_values = None

    try:
        # 读取PCD文件头部
        print(f'📖 分析PCD文件结构...')

        with open(file_path, 'r') as f:
            lines = f.readlines()

        # 解析PCD头部
        data_start = 0
        has_intensity = False
        intensity_index = -1
        field_count = 0
        point_count = 0

        for i, line in enumerate(lines):
            if line.startswith('DATA'):
                data_start = i + 1
                break

            # 检查字段定义
            if line.startswith('FIELDS'):
                fields = line.split()[1:]  # 跳过 'FIELDS' 关键字
                field_count = len(fields)
                print(f"📊 检测到字段 ({field_count}个): {fields}")

                if 'intensity' in fields:
                    has_intensity = True
                    intensity_index = fields.index('intensity')
                    print(f"✅ 发现强度字段 'intensity'，位于索引 {intensity_index}")
                elif 'i' in fields:
                    has_intensity = True
                    intensity_index = fields.index('i')
                    print(f"✅ 发现强度字段 'i'，位于索引 {intensity_index}")

            # 获取点数量
            elif line.startswith('POINTS'):
                point_count = int(line.split()[1])
                print(f"📊 点云总数: {point_count:,}")

        print(f"📊 数据起始行: {data_start}")
        print(f"📊 是否包含强度: {'是' if has_intensity else '否'}")

        # 如果有强度信息，解析前几行数据作为示例
        if has_intensity and data_start < len(lines):
            print(f"\n🔍 解析强度数据样本...")
            intensity_values = []
            sample_count = min(10, len(lines) - data_start)  # 只解析前10行作为示例

            print(f"解析前 {sample_count} 行数据作为示例:")

            for i, line in enumerate(lines[data_start:data_start + sample_count]):
                line = line.strip()
                if line:
                    parts = line.split()
                    if len(parts) > intensity_index:
                        try:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            intensity = float(parts[intensity_index])
                            intensity_values.append(intensity)
                            print(f"  点{i+1}: 位置=({x:.2f}, {y:.2f}, {z:.2f}), 强度={intensity:.1f}")
                        except (ValueError, IndexError) as e:
                            print(f"  点{i+1}: 解析错误 - {e}")
                            intensity_values.append(0.0)

            if intensity_values:
                intensity_values = np.array(intensity_values)
                print(f"\n✅ 成功提取 {len(intensity_values)} 个强度值（样本）")
                print(f"📊 强度范围: {intensity_values.min():.1f} ~ {intensity_values.max():.1f}")
                print(f"📊 强度均值: {intensity_values.mean():.1f}")

                # 显示强度分布
                print(f"\n🎨 强度分布预览:")
                print(f"   低强度 (<20): {np.sum(intensity_values < 20)} 个")
                print(f"   中等强度 (20-50): {np.sum((intensity_values >= 20) & (intensity_values < 50))} 个")
                print(f"   高强度 (>=50): {np.sum(intensity_values >= 50)} 个")

            else:
                print("⚠️  强度数据解析失败")
                intensity_values = None
        else:
            print("⚠️  未发现强度字段或数据为空")

    except Exception as e:
        print(f"❌ 强度解析出错: {e}")
        intensity_values = None

    return intensity_values is not None

def main():
    if len(sys.argv) != 2:
        print("用法: python3 test_intensity_parsing.py <pcd_file>")
        sys.exit(1)

    pcd_file = sys.argv[1]

    if not os.path.exists(pcd_file):
        print(f"❌ 文件不存在: {pcd_file}")
        sys.exit(1)

    # 测试强度解析
    success = test_load_pcd_with_intensity(pcd_file)

    print(f"\n{'='*60}")
    if success:
        print("🎉 强度解析测试成功！")
        print("💡 修复后的view_saved_map.py现在应该能够正确显示基于强度的颜色映射")
    else:
        print("❌ 强度解析测试失败")
        print("💡 将回退到基于高度的颜色映射")

if __name__ == '__main__':
    main()