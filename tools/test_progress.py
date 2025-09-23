#!/usr/bin/env python3
"""
测试进度条功能的简单脚本
"""

import os
import sys
import numpy as np
import open3d as o3d

# 添加map_refinement模块到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'map_refinement'))

from utils.progress_manager import initialize_progress_manager, progress_bar
from refinement.noise_reducer import NoiseReducer


def create_test_point_cloud(n_points=10000):
    """创建测试点云"""
    # 创建一个简单的立方体点云
    points = np.random.rand(n_points, 3) * 10
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def test_progress_bars():
    """测试进度条功能"""
    print("🧪 开始测试进度条功能...")

    # 初始化进度管理器
    progress_manager = initialize_progress_manager(True, True)
    print("✅ 进度管理器初始化成功")

    # 创建测试点云
    test_pcd = create_test_point_cloud(10000)
    print(f"✅ 创建测试点云: {len(test_pcd.points)} 个点")

    # 测试去噪器
    config = {
        'statistical_outlier': {'mean_k': 20, 'std_dev_mul_thresh': 2.0},
        'radius_outlier': {'radius': 0.3, 'min_neighbors': 3},
        'density_filter': {'min_density': 10},
        'adaptive_denoising': {
            'mode': 'conservative',
            'feature_weights': {'curvature': 0.2, 'normal_variation': 0.3, 'density': 0.2},
            'threshold_method': 'hybrid',
            'noise_percentile': 85
        },
        'performance': {'num_threads': 4, 'progress_display': True}
    }

    print("\n🔧 初始化去噪器...")
    noise_reducer = NoiseReducer(config)

    print("\n📊 测试多阶段去噪进度条...")
    try:
        cleaned_pcd, results = noise_reducer.multi_stage_denoising(test_pcd, ['statistical', 'density'])
        print(f"✅ 去噪完成: {len(test_pcd.points)} -> {len(cleaned_pcd.points)} 点")
    except Exception as e:
        print(f"❌ 去噪测试失败: {e}")
        return False

    print("\n🎉 所有进度条测试通过！")
    return True


if __name__ == "__main__":
    success = test_progress_bars()
    if success:
        print("\n💡 提示：现在运行地图精细化时应该能看到进度条了！")
        print("   使用命令: python3 map_refinement.py your_file.pcd")
    else:
        print("\n❌ 测试失败，请检查配置")

    sys.exit(0 if success else 1)