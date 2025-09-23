#!/usr/bin/env python3
"""
地图精细化主工具

集成多种算法提升室内建图精度：
- 法向量ICP增强配准算法
- 平面结构保护
- 边缘特征强化  
- 几何约束优化
- 点云去噪
- 表面重建
- 细节保护过滤
- 密度均匀化

使用方法:
    python3 map_refinement.py input.pcd --output refined.pcd
    python3 map_refinement.py input.pcd --pipeline full --quality-report
    python3 map_refinement.py batch_refine *.pcd --output-dir refined/
"""

import os
import sys
import argparse
import logging
import time
from pathlib import Path
from typing import List, Dict, Optional
import json
import copy

# 添加map_refinement模块到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'map_refinement'))

import numpy as np
import open3d as o3d

# 导入所有算法模块
from utils.io_utils import PointCloudIO
from utils.progress_manager import initialize_progress_manager, get_progress_manager
from algorithms.normal_icp import NormalICP
from algorithms.plane_preserver import PlanePreserver
from algorithms.edge_enhancer import EdgeEnhancer
from algorithms.geometry_optimizer import GeometryOptimizer
from refinement.noise_reducer import NoiseReducer
from refinement.surface_reconstructor import SurfaceReconstructor
from refinement.detail_preserver import DetailPreserver
from refinement.density_uniformizer import DensityUniformizer

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class MapRefinementPipeline:
    """地图精细化处理管道"""
    
    def __init__(self, config_path: str = None):
        """
        初始化处理管道
        
        Args:
            config_path: 配置文件路径
        """
        # 加载配置
        if config_path and os.path.exists(config_path):
            self.config = PointCloudIO.load_config(config_path)
        else:
            # 使用默认配置
            default_config_path = os.path.join(
                os.path.dirname(__file__), 
                'map_refinement/config/refinement_config.yaml'
            )
            if os.path.exists(default_config_path):
                self.config = PointCloudIO.load_config(default_config_path)
            else:
                self.config = self._get_default_config()
        
        # 初始化进度管理器
        performance_config = self.config.get('performance', {})
        progress_display = performance_config.get('progress_display', True)
        progress_bar_config = performance_config.get('progress_bar', {})
        use_tqdm = progress_bar_config.get('use_tqdm', True)

        self.progress_manager = initialize_progress_manager(progress_display, use_tqdm)
        logger.info(f"进度管理器初始化完成 - 显示: {progress_display}, tqdm: {use_tqdm}")

        # 初始化算法模块
        self._initialize_modules()

        logger.info("地图精细化管道初始化完成")

    def _get_pipeline_steps(self, pipeline: str) -> List[str]:
        """获取管道处理步骤"""
        if pipeline == "full":
            return ['denoise', 'structure', 'edges', 'geometry', 'surface', 'details', 'density']
        elif pipeline == "basic":
            return ['denoise', 'structure']
        elif pipeline == "denoise":
            return ['denoise']
        elif pipeline == "structure":
            return ['structure']
        elif pipeline == "edges":
            return ['edges']
        elif pipeline == "geometry":
            return ['geometry']
        elif pipeline == "surface":
            return ['surface']
        elif pipeline == "details":
            return ['details']
        elif pipeline == "density":
            return ['density']
        else:
            return ['denoise', 'structure']  # 默认基础流程

    def _get_default_config(self) -> Dict:
        """获取默认配置"""
        return {
            'normal_icp': {
                'max_iterations': 50,
                'convergence_threshold': 1e-6,
                'normal_search_radius': 0.1,
                'normal_weight': 0.7,
                'distance_weight': 0.3,
                'angle_threshold': 30.0
            },
            'plane_preserver': {
                'ransac_distance_threshold': 0.02,
                'ransac_max_iterations': 1000,
                'min_plane_points': 100,
                'parallel_threshold': 5.0,
                'perpendicular_threshold': 85.0
            },
            'edge_enhancer': {
                'curvature_threshold': 0.1,
                'edge_search_radius': 0.05,
                'edge_weight_multiplier': 2.0,
                'line_feature_min_points': 20
            },
            'geometry_optimizer': {
                'enable_parallel_constraint': True,
                'enable_perpendicular_constraint': True,
                'enable_distance_constraint': True,
                'enable_angle_constraint': True,
                'constraint_weight': 0.5
            },
            'noise_reduction': {
                'statistical_outlier': {
                    'mean_k': 50,
                    'std_dev_mul_thresh': 1.0
                },
                'radius_outlier': {
                    'radius': 0.05,
                    'min_neighbors': 10
                },
                'density_filter': {
                    'min_density': 50
                }
            },
            'surface_reconstruction': {
                'method': 'adaptive',
                'poisson_depth': 9,
                'poisson_width': 0,
                'poisson_scale': 1.1
            },
            'detail_preservation': {
                'multi_scale_levels': 3,
                'edge_sensitivity': 0.8,
                'texture_enhancement': True
            },
            'density_uniformization': {
                'target_density': 100,
                'adaptive_sampling': True,
                'hole_filling': True,
                'interpolation_method': 'linear'
            },
            'performance': {
                'num_threads': 4,
                'memory_limit_gb': 8,
                'progress_display': True,
                'progress_bar': {
                    'use_tqdm': True,
                    'refresh_rate': 0.1,
                    'show_nested': True
                },
                'optimization_strategy': 'auto'
            }
        }
        
    def _initialize_modules(self):
        """初始化所有算法模块"""
        try:
            self.normal_icp = NormalICP(self.config['normal_icp'])
            self.plane_preserver = PlanePreserver(self.config['plane_preserver'])
            self.edge_enhancer = EdgeEnhancer(self.config['edge_enhancer'])
            self.geometry_optimizer = GeometryOptimizer(self.config['geometry_optimizer'])
            self.noise_reducer = NoiseReducer(self.config['noise_reduction'])
            self.surface_reconstructor = SurfaceReconstructor(self.config['surface_reconstruction'])
            self.detail_preserver = DetailPreserver(self.config['detail_preservation'])
            self.density_uniformizer = DensityUniformizer(self.config['density_uniformization'])
            
            logger.info("所有算法模块初始化成功")
        except Exception as e:
            logger.error(f"算法模块初始化失败: {str(e)}")
            raise
            
    def process_single_cloud(self, input_path: str, output_path: str,
                           pipeline: str = "full", save_intermediate: bool = False) -> Dict:
        """
        处理单个点云文件
        
        Args:
            input_path: 输入点云路径
            output_path: 输出点云路径
            pipeline: 处理管道类型
            save_intermediate: 是否保存中间结果
            
        Returns:
            处理结果信息
        """
        logger.info(f"开始处理点云: {input_path}")
        start_time = time.time()
        
        # 加载点云
        try:
            pcd = PointCloudIO.load_point_cloud(input_path)
            original_info = PointCloudIO.get_point_cloud_info(pcd)
            logger.info(f"原始点云信息: {original_info['num_points']} 点")
        except Exception as e:
            logger.error(f"加载点云失败: {str(e)}")
            return {'success': False, 'error': str(e)}
        
        # 创建输出目录
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # 根据管道类型选择处理步骤
        processing_results = {}
        current_pcd = copy.deepcopy(pcd)

        # 获取自定义管道配置
        pipeline_config = self.config.get('processing_pipeline', {})
        skip_density = pipeline_config.get('skip_density_uniformization', False)
        skip_surface = pipeline_config.get('skip_surface_reconstruction', False)
        custom_steps = pipeline_config.get('custom_pipeline', [])

        # 确定处理步骤
        if pipeline == "custom" and custom_steps:
            processing_steps = custom_steps
        else:
            processing_steps = self._get_pipeline_steps(pipeline)

        # 应用跳过配置
        if skip_density and 'density' in processing_steps:
            processing_steps.remove('density')
        if skip_surface and 'surface' in processing_steps:
            processing_steps.remove('surface')

        total_steps = len(processing_steps)
        logger.info(f"处理管道: {pipeline}, 步骤: {processing_steps}")

        try:
            with self.progress_manager.progress_bar(total_steps, f"地图精细化-{pipeline}模式", unit="步骤") as main_pbar:
                step_counter = 0
                plane_info = {}  # 用于存储平面信息

                for step in processing_steps:
                    step_counter += 1

                    if step == 'denoise':
                        # 点云去噪
                        logger.info(f"步骤 {step_counter}/{total_steps}: 点云去噪")
                        denoised_pcd, denoise_results = self.noise_reducer.multi_stage_denoising(current_pcd)
                        current_pcd = denoised_pcd
                        processing_results['denoising'] = {
                            'stages': len(denoise_results),
                            'points_removed': original_info['num_points'] - len(current_pcd.points)
                        }

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_denoised")
                        main_pbar.update(1)

                    elif step == 'structure':
                        # 平面结构保护
                        logger.info(f"步骤 {step_counter}/{total_steps}: 平面结构保护")
                        preserved_pcd, plane_info = self.plane_preserver.preserve_planes(current_pcd)
                        current_pcd = preserved_pcd
                        processing_results['plane_preservation'] = {
                            'planes_detected': len(plane_info.get('planes', [])),
                            'constraints_applied': len(plane_info.get('constraints', []))
                        }

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_plane_preserved")
                        main_pbar.update(1)

                    elif step == 'edges':
                        # 边缘特征强化
                        logger.info(f"步骤 {step_counter}/{total_steps}: 边缘特征强化")
                        enhanced_pcd, edge_info = self.edge_enhancer.enhance_edges(current_pcd)
                        current_pcd = enhanced_pcd
                        processing_results['edge_enhancement'] = {
                            'edge_points': edge_info.get('total_edge_points', 0),
                            'line_features': edge_info.get('line_features', 0)
                        }

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_edge_enhanced")
                        main_pbar.update(1)

                    elif step == 'geometry':
                        # 几何约束优化
                        logger.info(f"步骤 {step_counter}/{total_steps}: 几何约束优化")
                        if plane_info.get('planes'):
                            planes = plane_info.get('planes', [])
                            optimized_pcd, opt_info = self.geometry_optimizer.optimize_geometry(current_pcd, planes)
                            current_pcd = optimized_pcd
                            processing_results['geometry_optimization'] = {
                                'constraints': opt_info.get('constraints', 0),
                                'error_reduction': opt_info.get('error_reduction', 0)
                            }
                        else:
                            processing_results['geometry_optimization'] = {'skipped': 'no_planes'}

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_geometry_optimized")
                        main_pbar.update(1)

                    elif step == 'surface':
                        # 表面重建
                        surface_config = self.config.get('surface_reconstruction', {})
                        surface_method = surface_config.get('method', 'poisson')

                        if surface_method == 'disabled':
                            logger.info(f"步骤 {step_counter}/{total_steps}: 表面重建 (已禁用)")
                            processing_results['surface_reconstruction'] = {'skipped': 'disabled_in_config'}
                        else:
                            logger.info(f"步骤 {step_counter}/{total_steps}: 表面重建")
                            surface_result = self.surface_reconstructor.reconstruct_surface(current_pcd)
                            if hasattr(surface_result, 'mesh') and len(surface_result.mesh.triangles) > 0:
                                # 从网格重新采样点云
                                resampled_pcd = surface_result.mesh.sample_points_uniformly(
                                    number_of_points=len(current_pcd.points)
                                )
                                current_pcd = resampled_pcd
                                processing_results['surface_reconstruction'] = {
                                    'method': surface_result.method,
                                    'vertices': len(surface_result.mesh.vertices),
                                    'triangles': len(surface_result.mesh.triangles)
                                }
                            else:
                                processing_results['surface_reconstruction'] = {'skipped': 'reconstruction_failed'}

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_surface_reconstructed")
                        main_pbar.update(1)

                    elif step == 'details':
                        # 细节保护
                        logger.info(f"步骤 {step_counter}/{total_steps}: 细节保护")
                        detail_result = self.detail_preserver.preserve_details(current_pcd)
                        current_pcd = detail_result.processed_pcd
                        processing_results['detail_preservation'] = {
                            'method': detail_result.method,
                            'feature_points': len(detail_result.feature_map)
                        }

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_detail_preserved")
                        main_pbar.update(1)

                    elif step == 'density':
                        # 密度均匀化
                        logger.info(f"步骤 {step_counter}/{total_steps}: 密度均匀化")
                        density_result = self.density_uniformizer.uniformize_density(current_pcd)
                        current_pcd = density_result.uniformized_pcd
                        processing_results['density_uniformization'] = {
                            'method': density_result.method,
                            'variance_reduction': density_result.density_variance_reduction
                        }

                        if save_intermediate:
                            self._save_intermediate(current_pcd, output_path, f"{step_counter:02d}_density_uniformized")
                        main_pbar.update(1)

                    else:
                        logger.warning(f"未知的处理步骤: {step}")
                        main_pbar.update(1)

                # 最终处理和保存
                logger.info("最终处理和保存")
            
            # 确保法向量一致性
            if current_pcd.has_normals():
                current_pcd.orient_normals_consistent_tangent_plane(100)
            else:
                current_pcd.estimate_normals()
                current_pcd.orient_normals_consistent_tangent_plane(100)
                
            # 保存最终结果
            success = PointCloudIO.save_point_cloud(current_pcd, output_path)
            
            if not success:
                raise Exception(f"保存点云失败: {output_path}")
                
            # 计算处理时间和质量指标
            processing_time = time.time() - start_time
            final_info = PointCloudIO.get_point_cloud_info(current_pcd)
            
            result = {
                'success': True,
                'processing_time': processing_time,
                'original_points': original_info['num_points'],
                'final_points': final_info['num_points'],
                'point_change_ratio': (final_info['num_points'] - original_info['num_points']) / original_info['num_points'],
                'pipeline': pipeline,
                'processing_results': processing_results
            }
            
            logger.info(f"处理完成: {original_info['num_points']} -> {final_info['num_points']} 点, "
                       f"耗时 {processing_time:.1f}s")
            
            return result
            
        except Exception as e:
            logger.error(f"处理过程中发生错误: {str(e)}")
            return {'success': False, 'error': str(e)}
            
    def _save_intermediate(self, pcd: o3d.geometry.PointCloud, base_output_path: str, suffix: str):
        """保存中间结果"""
        base_name = os.path.splitext(base_output_path)[0]
        intermediate_path = f"{base_name}_{suffix}.pcd"
        PointCloudIO.save_point_cloud(pcd, intermediate_path)
        logger.debug(f"保存中间结果: {intermediate_path}")
        
    def batch_process(self, input_pattern: str, output_dir: str, 
                     pipeline: str = "full") -> Dict:
        """
        批量处理点云文件
        
        Args:
            input_pattern: 输入文件模式（支持通配符）
            output_dir: 输出目录
            pipeline: 处理管道类型
            
        Returns:
            批量处理结果
        """
        import glob
        
        # 找到匹配的文件
        input_files = glob.glob(input_pattern)
        
        if not input_files:
            logger.warning(f"未找到匹配的文件: {input_pattern}")
            return {'success': False, 'error': 'no_files_found'}
            
        logger.info(f"找到 {len(input_files)} 个文件进行批量处理")
        
        # 创建输出目录
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # 批量处理
        results = []
        start_time = time.time()
        
        for i, input_file in enumerate(input_files):
            logger.info(f"处理文件 {i+1}/{len(input_files)}: {input_file}")
            
            # 生成输出文件路径
            base_name = os.path.basename(input_file)
            name_without_ext = os.path.splitext(base_name)[0]
            output_file = os.path.join(output_dir, f"{name_without_ext}_refined.pcd")
            
            # 处理单个文件
            result = self.process_single_cloud(input_file, output_file, pipeline)
            result['input_file'] = input_file
            result['output_file'] = output_file
            results.append(result)
            
        total_time = time.time() - start_time
        
        # 计算统计信息
        successful_count = sum(1 for r in results if r.get('success', False))
        total_original_points = sum(r.get('original_points', 0) for r in results if r.get('success', False))
        total_final_points = sum(r.get('final_points', 0) for r in results if r.get('success', False))
        
        batch_result = {
            'success': True,
            'total_files': len(input_files),
            'successful_files': successful_count,
            'failed_files': len(input_files) - successful_count,
            'total_time': total_time,
            'total_original_points': total_original_points,
            'total_final_points': total_final_points,
            'results': results
        }
        
        logger.info(f"批量处理完成: {successful_count}/{len(input_files)} 文件成功, "
                   f"总耗时 {total_time:.1f}s")
        
        return batch_result
        
    def generate_quality_report(self, input_path: str, output_path: str, result: Dict) -> str:
        """
        生成质量评估报告
        
        Args:
            input_path: 原始文件路径
            output_path: 处理后文件路径
            result: 处理结果
            
        Returns:
            报告文件路径
        """
        try:
            # 加载原始和处理后的点云
            original_pcd = PointCloudIO.load_point_cloud(input_path)
            refined_pcd = PointCloudIO.load_point_cloud(output_path)
            
            # 计算质量指标
            quality_metrics = self._compute_quality_metrics(original_pcd, refined_pcd)
            
            # 生成报告
            report = {
                'input_file': input_path,
                'output_file': output_path,
                'processing_time': result.get('processing_time', 0),
                'pipeline': result.get('pipeline', 'unknown'),
                'point_statistics': {
                    'original_points': len(original_pcd.points),
                    'refined_points': len(refined_pcd.points),
                    'point_change': len(refined_pcd.points) - len(original_pcd.points),
                    'point_change_ratio': (len(refined_pcd.points) - len(original_pcd.points)) / len(original_pcd.points)
                },
                'quality_metrics': quality_metrics,
                'processing_details': result.get('processing_results', {})
            }
            
            # 保存报告
            report_path = os.path.splitext(output_path)[0] + "_quality_report.json"
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
                
            logger.info(f"质量评估报告保存到: {report_path}")
            
            return report_path
            
        except Exception as e:
            logger.error(f"生成质量报告失败: {str(e)}")
            return ""
            
    def _compute_quality_metrics(self, original_pcd: o3d.geometry.PointCloud,
                               refined_pcd: o3d.geometry.PointCloud) -> Dict:
        """计算质量指标"""
        metrics = {}
        
        try:
            # 密度分析
            original_density = self.density_uniformizer.compute_density_map(original_pcd)
            refined_density = self.density_uniformizer.compute_density_map(refined_pcd)
            
            metrics['density'] = {
                'original_mean': float(np.mean(original_density)),
                'original_std': float(np.std(original_density)),
                'refined_mean': float(np.mean(refined_density)),
                'refined_std': float(np.std(refined_density)),
                'uniformity_improvement': float(np.std(original_density) - np.std(refined_density))
            }
            
            # 几何质量
            if original_pcd.has_normals() and refined_pcd.has_normals():
                original_normals = np.asarray(original_pcd.normals)
                refined_normals = np.asarray(refined_pcd.normals)
                
                metrics['surface_quality'] = {
                    'normal_consistency_original': float(np.mean(np.abs(np.sum(original_normals * original_normals, axis=1)))),
                    'normal_consistency_refined': float(np.mean(np.abs(np.sum(refined_normals * refined_normals, axis=1))))
                }
                
            # 特征保持度
            feature_importance_original = self.detail_preserver.compute_feature_importance(original_pcd)
            feature_importance_refined = self.detail_preserver.compute_feature_importance(refined_pcd)
            
            if len(feature_importance_original) > 0 and len(feature_importance_refined) > 0:
                metrics['feature_preservation'] = {
                    'original_feature_strength': float(np.mean(feature_importance_original)),
                    'refined_feature_strength': float(np.mean(feature_importance_refined)),
                    'feature_preservation_ratio': float(np.mean(feature_importance_refined) / np.mean(feature_importance_original))
                }
                
        except Exception as e:
            logger.warning(f"计算质量指标时出错: {str(e)}")
            metrics['error'] = str(e)
            
        return metrics

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='地图精细化工具')
    parser.add_argument('input', help='输入点云文件或批处理模式(batch_refine)')
    parser.add_argument('files', nargs='*', help='批处理模式下的输入文件')
    parser.add_argument('--output', '-o', help='输出文件路径')
    parser.add_argument('--output-dir', help='批处理模式下的输出目录')
    parser.add_argument('--config', help='配置文件路径')
    parser.add_argument('--pipeline', default='full',
                       choices=['full', 'basic', 'denoise', 'structure', 'edges', 'geometry', 'surface', 'details', 'density', 'custom'],
                       help='处理管道类型')
    parser.add_argument('--denoising-mode', default=None,
                       choices=['conservative', 'balanced', 'aggressive'],
                       help='去噪模式: conservative(保守,保留更多细节), balanced(平衡,推荐), aggressive(激进,严格去噪)')
    parser.add_argument('--save-intermediate', action='store_true', help='保存中间结果')
    parser.add_argument('--quality-report', action='store_true', help='生成质量评估报告')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        
    try:
        # 初始化处理管道
        pipeline = MapRefinementPipeline(args.config)

        # 应用命令行去噪模式覆盖
        if args.denoising_mode:
            if 'noise_reduction' not in pipeline.config:
                pipeline.config['noise_reduction'] = {}
            if 'adaptive_denoising' not in pipeline.config['noise_reduction']:
                pipeline.config['noise_reduction']['adaptive_denoising'] = {}

            pipeline.config['noise_reduction']['adaptive_denoising']['mode'] = args.denoising_mode
            logger.info(f"使用命令行指定的去噪模式: {args.denoising_mode}")

            # 重新初始化去噪器以应用新配置
            pipeline.noise_reducer = NoiseReducer(pipeline.config.get('noise_reduction', {}))

        if args.input == 'batch_refine':
            # 批处理模式
            if not args.files:
                logger.error("批处理模式需要指定输入文件")
                return 1
                
            if not args.output_dir:
                logger.error("批处理模式需要指定输出目录")
                return 1
                
            # 处理多个文件模式
            input_pattern = ' '.join(args.files)
            result = pipeline.batch_process(input_pattern, args.output_dir, args.pipeline)
            
            if result['success']:
                print(f"批处理完成: {result['successful_files']}/{result['total_files']} 文件成功")
                return 0
            else:
                print(f"批处理失败: {result.get('error', 'unknown error')}")
                return 1
                
        else:
            # 单文件处理模式
            if not args.output:
                # 自动生成输出文件名
                input_path = Path(args.input)
                args.output = str(input_path.parent / f"{input_path.stem}_refined{input_path.suffix}")
                
            result = pipeline.process_single_cloud(
                args.input, args.output, args.pipeline, args.save_intermediate
            )
            
            if result['success']:
                print(f"处理完成: {args.input} -> {args.output}")
                print(f"点数变化: {result['original_points']} -> {result['final_points']}")
                print(f"处理时间: {result['processing_time']:.1f}s")
                
                # 生成质量报告
                if args.quality_report:
                    report_path = pipeline.generate_quality_report(args.input, args.output, result)
                    if report_path:
                        print(f"质量报告: {report_path}")
                        
                return 0
            else:
                print(f"处理失败: {result.get('error', 'unknown error')}")
                return 1
                
    except Exception as e:
        logger.error(f"程序执行失败: {str(e)}")
        return 1

if __name__ == '__main__':
    sys.exit(main())