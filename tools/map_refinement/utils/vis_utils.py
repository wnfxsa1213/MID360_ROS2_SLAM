import copy
import numpy as np
import open3d as o3d
from typing import List, Tuple, Dict, Optional, Union
import logging
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import json
from .metrics import QualityMetrics

"""
可视化工具

提供点云处理结果的可视化功能：
- 点云对比显示
- 质量指标可视化
- 处理过程可视化
- 报告图表生成
"""


logger = logging.getLogger(__name__)

class VisualizationUtils:
    """可视化工具类"""
    
    def __init__(self):
        """初始化可视化工具"""
        # 设置绘图样式
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
        # 颜色配置
        self.colors = {
            'original': [0.7, 0.7, 0.7],      # 灰色
            'refined': [0.2, 0.8, 0.2],       # 绿色
            'noise': [1.0, 0.2, 0.2],         # 红色
            'features': [0.2, 0.2, 1.0],      # 蓝色
            'edges': [1.0, 0.6, 0.0]          # 橙色
        }
        
        logger.info("可视化工具初始化完成")
        
    def visualize_point_cloud_comparison(self, original_pcd: o3d.geometry.PointCloud,
                                       refined_pcd: o3d.geometry.PointCloud,
                                       title: str = "点云对比") -> None:
        """
        可视化点云对比
        
        Args:
            original_pcd: 原始点云
            refined_pcd: 精细化后点云
            title: 窗口标题
        """
        logger.info("显示点云对比可视化")
        
        # 复制点云以避免修改原始数据
        original_copy = o3d.geometry.PointCloud(original_pcd)
        refined_copy = o3d.geometry.PointCloud(refined_pcd)
        
        # 设置颜色
        original_copy.paint_uniform_color(self.colors['original'])
        refined_copy.paint_uniform_color(self.colors['refined'])
        
        # 分别移动点云以便对比
        original_bbox = original_copy.get_axis_aligned_bounding_box()
        offset = original_bbox.get_extent()[0] * 1.2
        
        # 平移精细化点云
        refined_copy.translate([offset, 0, 0])
        
        # 创建坐标系
        coord_frame_original = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0]
        )
        coord_frame_refined = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[offset, 0, 0]
        )
        
        # 创建文本标签（简化版本）
        logger.info("原始点云（左）vs 精细化点云（右）")
        
        # 显示
        o3d.visualization.draw_geometries(
            [original_copy, refined_copy, coord_frame_original, coord_frame_refined],
            window_name=title,
            width=1200,
            height=800
        )
        
    def visualize_processing_stages(self, stage_results: List[Tuple[str, o3d.geometry.PointCloud]]) -> None:
        """
        可视化处理阶段
        
        Args:
            stage_results: 阶段结果列表 [(阶段名, 点云), ...]
        """
        logger.info("显示处理阶段可视化")
        
        if not stage_results:
            logger.warning("无处理阶段数据")
            return
            
        # 准备显示的几何体
        geometries = []
        
        # 计算布局
        cols = min(3, len(stage_results))  # 最多3列
        rows = (len(stage_results) + cols - 1) // cols
        
        for i, (stage_name, pcd) in enumerate(stage_results):
            if len(pcd.points) == 0:
                continue
                
            # 复制点云
            stage_pcd = o3d.geometry.PointCloud(pcd)
            
            # 计算位置偏移
            row = i // cols
            col = i % cols
            
            bbox = stage_pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            
            offset_x = col * extent[0] * 1.5
            offset_y = row * extent[1] * 1.5
            
            stage_pcd.translate([offset_x, offset_y, 0])
            
            # 设置颜色
            color_intensity = 0.3 + 0.7 * i / len(stage_results)
            stage_pcd.paint_uniform_color([color_intensity, 0.8, 1.0 - color_intensity])
            
            geometries.append(stage_pcd)
            
            # 添加坐标系
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.2, origin=[offset_x, offset_y, 0]
            )
            geometries.append(coord_frame)
            
            logger.info(f"阶段 {i+1}: {stage_name} - {len(pcd.points)} 点")
            
        # 显示
        o3d.visualization.draw_geometries(
            geometries,
            window_name="处理阶段对比",
            width=1400,
            height=900
        )
        
    def visualize_feature_enhancement(self, original_pcd: o3d.geometry.PointCloud,
                                    edge_indices: np.ndarray,
                                    feature_indices: np.ndarray) -> None:
        """
        可视化特征增强结果
        
        Args:
            original_pcd: 原始点云
            edge_indices: 边缘点索引
            feature_indices: 特征点索引
        """
        logger.info("显示特征增强可视化")
        
        # 复制点云
        display_pcd = o3d.geometry.PointCloud(original_pcd)
        
        # 设置基础颜色（灰色）
        colors = np.array([[0.7, 0.7, 0.7]] * len(display_pcd.points))
        
        # 标记边缘点（橙色）
        if len(edge_indices) > 0:
            colors[edge_indices] = self.colors['edges']
            
        # 标记特征点（蓝色）
        if len(feature_indices) > 0:
            colors[feature_indices] = self.colors['features']
            
        display_pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 创建图例信息
        logger.info(f"特征可视化: 普通点（灰色）, 边缘点（橙色, {len(edge_indices)}个）, 特征点（蓝色, {len(feature_indices)}个）")
        
        # 显示
        o3d.visualization.draw_geometries(
            [display_pcd],
            window_name="特征增强结果",
            width=1000,
            height=800
        )
        
    def plot_quality_metrics(self, metrics: Dict, output_path: str = None) -> str:
        """
        绘制质量指标图表
        
        Args:
            metrics: 质量指标字典
            output_path: 输出图片路径
            
        Returns:
            生成的图片路径
        """
        logger.info("生成质量指标图表")
        
        # 创建子图
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('地图质量评估报告', fontsize=16, fontweight='bold')
        
        # 1. 密度分布图
        if 'density' in metrics:
            self._plot_density_distribution(axes[0, 0], metrics['density'])
            
        # 2. 几何质量雷达图
        if 'geometric' in metrics:
            self._plot_geometric_radar(axes[0, 1], metrics['geometric'])
            
        # 3. 特征保持度条形图
        if 'feature_preservation' in metrics:
            self._plot_preservation_bars(axes[1, 0], metrics['feature_preservation'])
            
        # 4. 综合评分
        if 'comprehensive_score' in metrics:
            self._plot_comprehensive_score(axes[1, 1], metrics['comprehensive_score'])
            
        plt.tight_layout()
        
        # 保存图片
        if output_path is None:
            output_path = "quality_metrics_report.png"
            
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"质量指标图表保存到: {output_path}")
        
        return output_path
        
    def plot_comparison_report(self, original_metrics: Dict, refined_metrics: Dict,
                             output_path: str = None) -> str:
        """
        绘制对比分析报告
        
        Args:
            original_metrics: 原始指标
            refined_metrics: 精细化后指标
            output_path: 输出路径
            
        Returns:
            生成的图片路径
        """
        logger.info("生成对比分析报告")
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('地图精细化前后对比分析', fontsize=16, fontweight='bold')
        
        # 1. 密度对比
        self._plot_density_comparison(axes[0, 0], original_metrics.get('density', {}), 
                                    refined_metrics.get('density', {}))
        
        # 2. 几何质量对比
        self._plot_geometric_comparison(axes[0, 1], original_metrics.get('geometric', {}),
                                      refined_metrics.get('geometric', {}))
        
        # 3. 综合评分对比
        self._plot_score_comparison(axes[0, 2], original_metrics, refined_metrics)
        
        # 4. 改进指标
        self._plot_improvement_metrics(axes[1, 0], original_metrics, refined_metrics)
        
        # 5. 特征保持度（如果有）
        if 'feature_preservation' in refined_metrics:
            self._plot_preservation_details(axes[1, 1], refined_metrics['feature_preservation'])
        else:
            axes[1, 1].text(0.5, 0.5, '无特征保持度数据', ha='center', va='center')
            axes[1, 1].set_title('特征保持度')
            
        # 6. 处理效果总结
        self._plot_processing_summary(axes[1, 2], original_metrics, refined_metrics)
        
        plt.tight_layout()
        
        # 保存图片
        if output_path is None:
            output_path = "comparison_report.png"
            
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"对比分析报告保存到: {output_path}")
        
        return output_path
        
    def create_interactive_visualization(self, pcd: o3d.geometry.PointCloud,
                                       feature_data: Dict = None) -> None:
        """
        创建交互式可视化
        
        Args:
            pcd: 点云数据
            feature_data: 特征数据字典
        """
        logger.info("启动交互式可视化")
        
        # 准备几何体
        geometries = [pcd]
        
        # 添加特征可视化
        if feature_data:
            if 'edges' in feature_data:
                edge_spheres = self._create_feature_spheres(
                    pcd, feature_data['edges'], self.colors['edges'], 0.01
                )
                geometries.extend(edge_spheres)
                
            if 'features' in feature_data:
                feature_spheres = self._create_feature_spheres(
                    pcd, feature_data['features'], self.colors['features'], 0.015
                )
                geometries.extend(feature_spheres)
                
        # 添加坐标系
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        geometries.append(coord_frame)
        
        # 启动可视化
        o3d.visualization.draw_geometries(
            geometries,
            window_name="交互式点云可视化",
            width=1200,
            height=900
        )
        
    # ==================== 私有方法 ====================
    
    def _plot_density_distribution(self, ax, density_metrics: Dict):
        """绘制密度分布图"""
        if 'density_distribution' in density_metrics:
            densities = density_metrics['density_distribution']
            ax.hist(densities, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
            ax.set_title('点云密度分布')
            ax.set_xlabel('密度 (点/立方米)')
            ax.set_ylabel('频次')
            ax.grid(True, alpha=0.3)
            
            # 添加统计信息
            stats = density_metrics.get('statistics', {})
            mean_density = stats.get('mean_density', 0)
            ax.axvline(mean_density, color='red', linestyle='--', 
                      label=f'均值: {mean_density:.1f}')
            ax.legend()
        else:
            ax.text(0.5, 0.5, '无密度数据', ha='center', va='center')
            ax.set_title('密度分布')
            
    def _plot_geometric_radar(self, ax, geometric_metrics: Dict):
        """绘制几何质量雷达图"""
        # 提取指标
        metrics_names = ['法向量一致性', '表面平滑度', '边缘清晰度', '几何复杂度']
        
        values = [
            geometric_metrics.get('normal_consistency', {}).get('consistency_score', 0),
            geometric_metrics.get('surface_smoothness', {}).get('smoothness_score', 0),
            geometric_metrics.get('edge_metrics', {}).get('edge_sharpness', 0),
            geometric_metrics.get('geometric_complexity', {}).get('complexity_score', 0)
        ]
        
        # 雷达图
        angles = np.linspace(0, 2 * np.pi, len(metrics_names), endpoint=False)
        values = np.concatenate((values, [values[0]]))  # 闭合
        angles = np.concatenate((angles, [angles[0]]))
        
        ax.plot(angles, values, 'o-', linewidth=2, label='几何质量')
        ax.fill(angles, values, alpha=0.25)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metrics_names)
        ax.set_ylim(0, 1)
        ax.set_title('几何质量雷达图')
        ax.grid(True)
        
    def _plot_preservation_bars(self, ax, preservation_metrics: Dict):
        """绘制特征保持度条形图"""
        categories = ['特征点', '结构', '细节', '空间分布']
        
        values = [
            preservation_metrics.get('feature_preservation', {}).get('preservation_ratio', 0),
            preservation_metrics.get('structure_preservation', {}).get('structure_score', 0),
            preservation_metrics.get('detail_preservation', {}).get('detail_preservation', 0),
            preservation_metrics.get('spatial_preservation', {}).get('spatial_preservation', 0)
        ]
        
        bars = ax.bar(categories, values, color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])
        ax.set_title('特征保持度')
        ax.set_ylabel('保持度评分')
        ax.set_ylim(0, 1)
        
        # 添加数值标签
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                   f'{value:.3f}', ha='center', va='bottom')
                   
    def _plot_comprehensive_score(self, ax, score: float):
        """绘制综合评分"""
        # 创建圆形进度条
        theta = np.linspace(0, 2*np.pi, 100)
        radius = 1
        
        # 背景圆
        ax.plot(radius * np.cos(theta), radius * np.sin(theta), 'lightgray', linewidth=8)
        
        # 评分弧线
        score_angle = 2 * np.pi * (score / 100)
        score_theta = np.linspace(0, score_angle, int(score))
        ax.plot(radius * np.cos(score_theta), radius * np.sin(score_theta), 
               'green', linewidth=8)
        
        # 评分文本
        ax.text(0, 0, f'{score:.1f}', ha='center', va='center', 
               fontsize=20, fontweight='bold')
        ax.text(0, -0.3, '综合评分', ha='center', va='center', fontsize=12)
        
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title('综合质量评分')
        
    def _plot_density_comparison(self, ax, original_density: Dict, refined_density: Dict):
        """绘制密度对比"""
        categories = ['均值', '标准差', '均匀性']
        
        original_values = [
            original_density.get('statistics', {}).get('mean_density', 0),
            original_density.get('statistics', {}).get('std_density', 0),
            original_density.get('uniformity_score', 0)
        ]
        
        refined_values = [
            refined_density.get('statistics', {}).get('mean_density', 0),
            refined_density.get('statistics', {}).get('std_density', 0),
            refined_density.get('uniformity_score', 0)
        ]
        
        x = np.arange(len(categories))
        width = 0.35
        
        ax.bar(x - width/2, original_values, width, label='原始', alpha=0.8)
        ax.bar(x + width/2, refined_values, width, label='精细化', alpha=0.8)
        
        ax.set_title('密度指标对比')
        ax.set_xticks(x)
        ax.set_xticklabels(categories)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_geometric_comparison(self, ax, original_geometric: Dict, refined_geometric: Dict):
        """绘制几何质量对比"""
        metrics = ['normal_consistency', 'surface_smoothness', 'edge_sharpness']
        labels = ['法向量一致性', '表面平滑度', '边缘清晰度']
        
        original_values = []
        refined_values = []
        
        for metric in metrics:
            if metric == 'normal_consistency':
                orig_val = original_geometric.get('normal_consistency', {}).get('consistency_score', 0)
                ref_val = refined_geometric.get('normal_consistency', {}).get('consistency_score', 0)
            elif metric == 'surface_smoothness':
                orig_val = original_geometric.get('surface_smoothness', {}).get('smoothness_score', 0)
                ref_val = refined_geometric.get('surface_smoothness', {}).get('smoothness_score', 0)
            else:  # edge_sharpness
                orig_val = original_geometric.get('edge_metrics', {}).get('edge_sharpness', 0)
                ref_val = refined_geometric.get('edge_metrics', {}).get('edge_sharpness', 0)
                
            original_values.append(orig_val)
            refined_values.append(ref_val)
        
        x = np.arange(len(labels))
        width = 0.35
        
        ax.bar(x - width/2, original_values, width, label='原始', alpha=0.8)
        ax.bar(x + width/2, refined_values, width, label='精细化', alpha=0.8)
        
        ax.set_title('几何质量对比')
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_score_comparison(self, ax, original_metrics: Dict, refined_metrics: Dict):
        """绘制评分对比"""
        
        metrics_calculator = QualityMetrics()
        original_score = metrics_calculator.compute_comprehensive_score(original_metrics)
        refined_score = metrics_calculator.compute_comprehensive_score(refined_metrics)
        
        categories = ['原始', '精细化']
        scores = [original_score, refined_score]
        colors = ['lightcoral', 'lightgreen']
        
        bars = ax.bar(categories, scores, color=colors, alpha=0.8)
        ax.set_title('综合评分对比')
        ax.set_ylabel('评分')
        ax.set_ylim(0, 100)
        
        # 添加数值标签
        for bar, score in zip(bars, scores):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                   f'{score:.1f}', ha='center', va='bottom', fontweight='bold')
                   
        # 添加改进指示
        improvement = refined_score - original_score
        if improvement > 0:
            ax.text(0.5, max(scores) * 0.8, f'改进: +{improvement:.1f}', 
                   ha='center', va='center', fontsize=12, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
                   
    def _plot_improvement_metrics(self, ax, original_metrics: Dict, refined_metrics: Dict):
        """绘制改进指标"""
        improvements = {}
        
        # 计算各项改进
        if 'density' in original_metrics and 'density' in refined_metrics:
            orig_uniformity = original_metrics['density'].get('uniformity_score', 0)
            ref_uniformity = refined_metrics['density'].get('uniformity_score', 0)
            improvements['密度均匀性'] = ref_uniformity - orig_uniformity
            
        # 可以添加更多改进指标...
        
        if improvements:
            categories = list(improvements.keys())
            values = list(improvements.values())
            colors = ['green' if v > 0 else 'red' for v in values]
            
            bars = ax.bar(categories, values, color=colors, alpha=0.7)
            ax.set_title('改进指标')
            ax.set_ylabel('改进幅度')
            ax.axhline(y=0, color='black', linestyle='-', alpha=0.3)
            
            # 添加数值标签
            for bar, value in zip(bars, values):
                height = bar.get_height()
                y_pos = height + 0.01 if height > 0 else height - 0.01
                va = 'bottom' if height > 0 else 'top'
                ax.text(bar.get_x() + bar.get_width()/2., y_pos,
                       f'{value:.3f}', ha='center', va=va)
        else:
            ax.text(0.5, 0.5, '无改进数据', ha='center', va='center')
            ax.set_title('改进指标')
            
    def _plot_preservation_details(self, ax, preservation_metrics: Dict):
        """绘制特征保持度详情"""
        # 提取详细数据
        feature_data = preservation_metrics.get('feature_preservation', {})
        
        if feature_data:
            labels = ['保持率', '损失率']
            sizes = [
                feature_data.get('preservation_ratio', 0),
                feature_data.get('feature_loss', 0)
            ]
            colors = ['lightgreen', 'lightcoral']
            
            wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors, 
                                            autopct='%1.1f%%', startangle=90)
            ax.set_title('特征保持度分析')
        else:
            ax.text(0.5, 0.5, '无特征保持度数据', ha='center', va='center')
            ax.set_title('特征保持度分析')
            
    def _plot_processing_summary(self, ax, original_metrics: Dict, refined_metrics: Dict):
        """绘制处理效果总结"""
        # 创建总结文本
        summary_text = "处理效果总结:\n\n"
        
        # 点数变化
        orig_points = original_metrics.get('point_count', 0)
        ref_points = refined_metrics.get('point_count', 0)
        if orig_points > 0:
            point_change = (ref_points - orig_points) / orig_points * 100
            summary_text += f"点数变化: {point_change:+.1f}%\n"
            
        # 质量评分变化
        metrics_calculator = QualityMetrics()
        orig_score = metrics_calculator.compute_comprehensive_score(original_metrics)
        ref_score = metrics_calculator.compute_comprehensive_score(refined_metrics)
        score_change = ref_score - orig_score
        summary_text += f"质量评分: {orig_score:.1f} → {ref_score:.1f}\n"
        summary_text += f"评分变化: {score_change:+.1f}\n\n"
        
        # 处理建议
        if score_change > 5:
            summary_text += "✅ 处理效果良好"
        elif score_change > 0:
            summary_text += "⚠️ 略有改善"
        else:
            summary_text += "❌ 需要调整参数"
            
        ax.text(0.1, 0.9, summary_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue"))
        ax.axis('off')
        ax.set_title('处理效果总结')
        
    def _create_feature_spheres(self, pcd: o3d.geometry.PointCloud, 
                              indices: np.ndarray, color: List[float], 
                              radius: float) -> List[o3d.geometry.TriangleMesh]:
        """创建特征点球体"""
        spheres = []
        points = np.asarray(pcd.points)
        
        for idx in indices[:100]:  # 限制数量以提高性能
            if idx < len(points):
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                sphere.translate(points[idx])
                sphere.paint_uniform_color(color)
                spheres.append(sphere)
                
        return spheres