#!/usr/bin/env python3
"""
地图精细化主工具（重构版）

集成多种算法提升室内建图精度：
- 法向量ICP增强配准算法
- 平面结构保护
- 边缘特征强化
- 几何约束优化
- 点云去噪
- 表面重建
- 细节保护过滤
- 密度均匀化

新特性：
- 明确的子命令：refine（单文件）、batch（批处理）、report（质量报告）
- 兼容旧调用："batch_refine" 仍可用（等价于子命令 batch）
- 支持 --dry-run、--overwrite、--threads、--save-config

使用示例：
  python3 map_refinement.py refine input.pcd -o refined.pcd --pipeline full --quality-report
  python3 map_refinement.py batch "saved_maps/*.pcd" --output-dir refined/ --pipeline basic
  python3 map_refinement.py report refined.pcd --input input.pcd
"""

import os
import sys
import argparse
import logging
import time
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import json
import copy
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed

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

_CONFIG_CACHE: Dict[Tuple[str, float], Dict] = {}


def _load_config_cached(path: str) -> Dict:
    abs_path = os.path.abspath(path)
    mtime = os.path.getmtime(abs_path)
    cache_key = (abs_path, mtime)
    if cache_key not in _CONFIG_CACHE:
        config = PointCloudIO.load_config(abs_path)
        _CONFIG_CACHE[cache_key] = config
    return copy.deepcopy(_CONFIG_CACHE[cache_key])


def _batch_process_worker(args):
    (
        config_dict,
        input_file,
        output_file,
        pipeline_name,
        overwrite,
        dry_run,
        save_intermediate,
    ) = args
    worker = MapRefinementPipeline(config_dict=config_dict)
    return worker.process_single_cloud(
        input_file,
        output_file,
        pipeline=pipeline_name,
        save_intermediate=save_intermediate,
        overwrite=overwrite,
        dry_run=dry_run,
    )

class MapRefinementPipeline:
    """地图精细化处理管道"""
    
    def __init__(self, config_path: str = None, config_dict: Optional[Dict] = None):
        """
        初始化处理管道
        
        Args:
            config_path: 配置文件路径
        """
        self._config_source_path = None

        if config_dict is not None:
            self.config = copy.deepcopy(config_dict)
        else:
            if config_path and os.path.exists(config_path):
                self.config = _load_config_cached(config_path)
                self._config_source_path = os.path.abspath(config_path)
            else:
                default_config_path = os.path.join(
                    os.path.dirname(__file__),
                    'map_refinement/config/refinement_config.yaml'
                )
                if os.path.exists(default_config_path):
                    self.config = _load_config_cached(default_config_path)
                    self._config_source_path = os.path.abspath(default_config_path)
                else:
                    self.config = self._get_default_config()
        
        # 初始化进度管理器
        performance_config = self.config.get('performance', {})
        progress_display = performance_config.get('progress_display', True)
        progress_bar_config = performance_config.get('progress_bar', {})
        use_tqdm = progress_bar_config.get('use_tqdm', True)

        self.progress_manager = initialize_progress_manager(progress_display, use_tqdm)
        logger.info(f"进度管理器初始化完成 - 显示: {progress_display}, tqdm: {use_tqdm}")

        intermediate_workers = max(0, performance_config.get('intermediate_io_workers', 1))
        self._intermediate_executor = None
        self._pending_io = []
        if intermediate_workers > 0:
            self._intermediate_executor = ThreadPoolExecutor(max_workers=intermediate_workers)
            logger.debug(f"中间结果写入采用异步线程池，线程数: {intermediate_workers}")

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
                'interpolation_method': 'linear',
                'mode': 'kd_tree',
                'voxel_density_downsample': 2.0,
                'voxel_neighbor_levels': 0,
                'hybrid_knn': 64,
                'hybrid_radius_scale': 1.0,
                'use_gpu': False
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
    
    def _wait_for_intermediate_tasks(self):
        if not self._pending_io:
            return
        for future in self._pending_io:
            try:
                future.result()
            except Exception as exc:
                logger.warning(f"异步保存中间结果失败: {exc}")
        self._pending_io.clear()

    def _shutdown_intermediate_executor(self):
        self._wait_for_intermediate_tasks()
        if self._intermediate_executor:
            self._intermediate_executor.shutdown(wait=True)
            self._intermediate_executor = None

    def __del__(self):
        try:
            self._shutdown_intermediate_executor()
        except Exception:
            pass
            
    def process_single_cloud(self, input_path: str, output_path: str,
                           pipeline: str = "full", save_intermediate: bool = False,
                           overwrite: bool = False, dry_run: bool = False) -> Dict:
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

        original_points = original_info.get('num_points', 0)
        
        # 创建输出目录
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        if original_points == 0:
            # 空点云没有处理意义，直接跳过
            logger.warning(f"输入点云为空，跳过处理: {input_path}")
            if dry_run:
                return {
                    'success': True,
                    'dry_run': True,
                    'skipped': True,
                    'reason': 'empty_point_cloud',
                    'output_file': output_path,
                    'processing_time': 0.0,
                    'original_points': 0,
                    'final_points': 0,
                    'pipeline': pipeline,
                    'processing_results': {}
                }

            if overwrite or not os.path.exists(output_path):
                PointCloudIO.save_point_cloud(pcd, output_path)

            self._wait_for_intermediate_tasks()
            return {
                'success': True,
                'skipped': True,
                'reason': 'empty_point_cloud',
                'output_file': output_path,
                'processing_time': 0.0,
                'original_points': 0,
                'final_points': 0,
                'pipeline': pipeline,
                'processing_results': {}
            }

        # 已存在输出且未要求覆盖
        if os.path.exists(output_path) and not overwrite:
            logger.info(f"输出已存在且未指定 --overwrite，跳过: {output_path}")
            self._wait_for_intermediate_tasks()
            return {'success': True, 'skipped': True, 'output_file': output_path,
                    'processing_time': 0.0, 'original_points': original_info['num_points'],
                    'final_points': None, 'pipeline': pipeline, 'processing_results': {}}

        if dry_run:
            logger.info("dry-run 模式：仅检查输入输出与配置，不执行计算")
            self._wait_for_intermediate_tasks()
            return {'success': True, 'dry_run': True, 'output_file': output_path,
                    'processing_time': 0.0, 'original_points': original_info['num_points'],
                    'final_points': None, 'pipeline': pipeline, 'processing_results': {}}
            
        # 根据管道类型选择处理步骤
        processing_results = {}
        if hasattr(pcd, "clone"):
            current_pcd = pcd.clone()
        else:
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
                            'points_removed': max(original_points - len(current_pcd.points), 0)
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
            
            final_points = final_info.get('num_points', 0)
            point_change_ratio = None
            if original_points > 0:
                point_change_ratio = (final_points - original_points) / original_points

            result = {
                'success': True,
                'processing_time': processing_time,
                'original_points': original_points,
                'final_points': final_points,
                'point_change_ratio': point_change_ratio,
                'pipeline': pipeline,
                'processing_results': processing_results
            }
            
            logger.info(f"处理完成: {original_points} -> {final_points} 点, "
                       f"耗时 {processing_time:.1f}s")
            
            self._wait_for_intermediate_tasks()
            return result
            
        except Exception as e:
            logger.error(f"处理过程中发生错误: {str(e)}")
            self._wait_for_intermediate_tasks()
            return {'success': False, 'error': str(e)}
            
    def _save_intermediate(self, pcd: o3d.geometry.PointCloud, base_output_path: str, suffix: str):
        """保存中间结果"""
        base_name = os.path.splitext(base_output_path)[0]
        intermediate_path = f"{base_name}_{suffix}.pcd"
        if self._intermediate_executor:
            if hasattr(pcd, "clone"):
                snapshot = pcd.clone()
            else:
                snapshot = copy.deepcopy(pcd)
            future = self._intermediate_executor.submit(PointCloudIO.save_point_cloud, snapshot, intermediate_path)
            self._pending_io.append(future)
        else:
            PointCloudIO.save_point_cloud(pcd, intermediate_path)
        logger.debug(f"保存中间结果: {intermediate_path}")
        
    def batch_process(self, input_pattern: str, output_dir: str,
                     pipeline: str = "full", overwrite: bool = False,
                     dry_run: bool = False) -> Dict:
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
            os.makedirs(output_dir, exist_ok=True)
            
        # 批量处理
        results = []
        start_time = time.time()

        batch_workers = max(1, int(self.config.get('performance', {}).get('batch_workers', 1)))
        batch_workers = min(batch_workers, len(input_files))
        logger.info(f"批处理线程/进程数: {batch_workers}")

        if batch_workers > 1 and not dry_run:
            config_snapshot = copy.deepcopy(self.config)
            tasks = []
            for input_file in input_files:
                base_name = os.path.basename(input_file)
                name_without_ext = os.path.splitext(base_name)[0]
                output_file = os.path.join(output_dir, f"{name_without_ext}_refined.pcd")
                tasks.append(
                    (
                        config_snapshot,
                        input_file,
                        output_file,
                        pipeline,
                        overwrite,
                        dry_run,
                        False,
                    )
                )
            with ProcessPoolExecutor(max_workers=batch_workers) as executor:
                future_to_io = {executor.submit(_batch_process_worker, task): task for task in tasks}
                for future in as_completed(future_to_io):
                    task = future_to_io[future]
                    input_file = task[1]
                    output_file = task[2]
                    try:
                        result = future.result()
                    except Exception as exc:
                        logger.error(f"文件 {input_file} 处理失败: {exc}")
                        result = {'success': False, 'error': str(exc)}
                    result['input_file'] = input_file
                    result['output_file'] = output_file
                    results.append(result)
        else:
            for i, input_file in enumerate(input_files):
                logger.info(f"处理文件 {i+1}/{len(input_files)}: {input_file}")
                
                # 生成输出文件路径
                base_name = os.path.basename(input_file)
                name_without_ext = os.path.splitext(base_name)[0]
                output_file = os.path.join(output_dir, f"{name_without_ext}_refined.pcd")
                
                # 处理单个文件
                result = self.process_single_cloud(input_file, output_file, pipeline,
                                                   overwrite=overwrite, dry_run=dry_run)
                result['input_file'] = input_file
                result['output_file'] = output_file
                results.append(result)
        
        total_time = time.time() - start_time
        
        # 计算统计信息
        results.sort(key=lambda r: r.get('input_file', ''))
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
                    'point_change_ratio': ((len(refined_pcd.points) - len(original_pcd.points)) / len(original_pcd.points)
                                           if len(original_pcd.points) > 0 else None)
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

            if len(original_density) == 0 or len(refined_density) == 0:
                metrics['density'] = {'skipped': 'insufficient_points'}
            else:
                original_std = float(np.std(original_density))
                refined_std = float(np.std(refined_density))
                metrics['density'] = {
                    'original_mean': float(np.mean(original_density)),
                    'original_std': original_std,
                    'refined_mean': float(np.mean(refined_density)),
                    'refined_std': refined_std,
                    'uniformity_improvement': float(original_std - refined_std)
                }
            
            # 几何质量
            if original_pcd.has_normals() and refined_pcd.has_normals():
                original_normals = np.asarray(original_pcd.normals)
                refined_normals = np.asarray(refined_pcd.normals)
                n = min(len(original_normals), len(refined_normals))

                if n > 0:
                    original_normals = original_normals[:n]
                    refined_normals = refined_normals[:n]

                    def _safe_normalize(normals: np.ndarray) -> np.ndarray:
                        norms = np.linalg.norm(normals, axis=1, keepdims=True)
                        norms[norms == 0] = 1.0
                        return normals / norms

                    original_normals = _safe_normalize(original_normals)
                    refined_normals = _safe_normalize(refined_normals)
                    alignment = np.clip(np.sum(original_normals * refined_normals, axis=1), -1.0, 1.0)

                    metrics['surface_quality'] = {
                        'mean_alignment': float(np.mean(np.abs(alignment))),
                        'alignment_std': float(np.std(alignment))
                    }
            
            # 特征保持度
            feature_importance_original = self.detail_preserver.compute_feature_importance(original_pcd)
            feature_importance_refined = self.detail_preserver.compute_feature_importance(refined_pcd)
            
            if len(feature_importance_original) > 0 and len(feature_importance_refined) > 0:
                original_strength = float(np.mean(feature_importance_original))
                refined_strength = float(np.mean(feature_importance_refined))

                preservation_ratio = None
                if abs(original_strength) > 1e-9:
                    preservation_ratio = refined_strength / original_strength

                metrics['feature_preservation'] = {
                    'original_feature_strength': original_strength,
                    'refined_feature_strength': refined_strength,
                    'feature_preservation_ratio': preservation_ratio
                }
                
        except Exception as e:
            logger.warning(f"计算质量指标时出错: {str(e)}")
            metrics['error'] = str(e)
            
        return metrics

def _apply_cli_overrides(pipeline: MapRefinementPipeline, denoising_mode: Optional[str], threads: Optional[int], save_config_path: Optional[str]):
    if denoising_mode:
        if 'noise_reduction' not in pipeline.config:
            pipeline.config['noise_reduction'] = {}
        if 'adaptive_denoising' not in pipeline.config['noise_reduction']:
            pipeline.config['noise_reduction']['adaptive_denoising'] = {}
        pipeline.config['noise_reduction']['adaptive_denoising']['mode'] = denoising_mode
        logger.info(f"使用命令行指定的去噪模式: {denoising_mode}")
        pipeline.noise_reducer = NoiseReducer(pipeline.config.get('noise_reduction', {}))

    if threads and threads > 0:
        perf = pipeline.config.setdefault('performance', {})
        perf['num_threads'] = threads
        os.environ["OMP_NUM_THREADS"] = str(threads)
        os.environ["OPEN3D_CPU_THREADS"] = str(threads)
        logger.info(f"设置线程数: {threads}")

    if save_config_path:
        try:
            Path(save_config_path).parent.mkdir(parents=True, exist_ok=True)
            with open(save_config_path, 'w', encoding='utf-8') as f:
                json.dump(pipeline.config, f, indent=2, ensure_ascii=False)
            logger.info(f"已保存有效配置到: {save_config_path}")
        except Exception as e:
            logger.warning(f"保存配置失败: {e}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='地图精细化工具')
    subparsers = parser.add_subparsers(dest='command')

    # 共用选项
    parser.add_argument('--config', help='配置文件路径')
    parser.add_argument('--pipeline', default='full',
                       choices=['full', 'basic', 'denoise', 'structure', 'edges', 'geometry', 'surface', 'details', 'density', 'custom'],
                       help='处理管道类型')
    parser.add_argument('--denoising-mode', default=None,
                       choices=['conservative', 'balanced', 'aggressive'],
                       help='去噪模式覆盖')
    parser.add_argument('--threads', type=int, default=None, help='覆盖线程数')
    parser.add_argument('--save-config', help='将最终生效的配置保存到该路径(JSON)')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')

    # refine 子命令
    p_refine = subparsers.add_parser('refine', help='处理单个点云')
    p_refine.add_argument('input', help='输入点云文件')
    p_refine.add_argument('--output', '-o', help='输出文件路径')
    p_refine.add_argument('--save-intermediate', action='store_true', help='保存中间结果')
    p_refine.add_argument('--quality-report', action='store_true', help='生成质量评估报告')
    p_refine.add_argument('--overwrite', action='store_true', help='覆盖已存在的输出')
    p_refine.add_argument('--dry-run', action='store_true', help='不执行，仅检查')

    # batch 子命令
    p_batch = subparsers.add_parser('batch', help='批量处理点云（通配符）')
    p_batch.add_argument('pattern', help='输入文件通配符，如 "saved_maps/*.pcd"')
    p_batch.add_argument('--output-dir', required=True, help='输出目录')
    p_batch.add_argument('--overwrite', action='store_true', help='覆盖已存在的输出')
    p_batch.add_argument('--dry-run', action='store_true', help='不执行，仅检查')

    # report 子命令
    p_report = subparsers.add_parser('report', help='生成质量评估报告')
    p_report.add_argument('output', help='精细化后的点云文件')
    p_report.add_argument('--input', required=True, help='原始点云文件')

    # 兼容旧调用：batch_refine *.pcd --output-dir
    parser.add_argument('legacy', nargs='?', help=argparse.SUPPRESS)
    parser.add_argument('legacy_files', nargs='*', help=argparse.SUPPRESS)

    args = parser.parse_args()

    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        pipe = MapRefinementPipeline(getattr(args, 'config', None))
        _apply_cli_overrides(pipe, getattr(args, 'denoising_mode', None), getattr(args, 'threads', None), getattr(args, 'save_config', None))

        # 旧式调用兼容：batch_refine
        if getattr(args, 'legacy', None) == 'batch_refine':
            files = getattr(args, 'legacy_files', [])
            if not files:
                logger.error("批处理模式需要指定输入文件通配符")
                return 1
            pattern = ' '.join(files)
            out_dir = getattr(args, 'output_dir', None) or 'refined'
            res = pipe.batch_process(pattern, out_dir, getattr(args, 'pipeline', 'full'))
            print(f"批处理完成: {res.get('successful_files', 0)}/{res.get('total_files', 0)} 文件成功")
            return 0 if res.get('success', False) else 1

        if args.command == 'refine':
            out = args.output
            if not out:
                in_path = Path(args.input)
                out = str(in_path.parent / f"{in_path.stem}_refined{in_path.suffix}")
            result = pipe.process_single_cloud(args.input, out, getattr(args, 'pipeline', 'full'),
                                               getattr(args, 'save_intermediate', False),
                                               getattr(args, 'overwrite', False),
                                               getattr(args, 'dry_run', False))
            if result.get('success') and not result.get('skipped') and not result.get('dry_run'):
                print(f"处理完成: {args.input} -> {out}")
                print(f"点数变化: {result['original_points']} -> {result['final_points']}")
                print(f"处理时间: {result['processing_time']:.1f}s")
                if getattr(args, 'quality_report', False):
                    report_path = pipe.generate_quality_report(args.input, out, result)
                    if report_path:
                        print(f"质量报告: {report_path}")
            elif result.get('skipped'):
                print(f"跳过（已存在）: {out}")
            elif result.get('dry_run'):
                print(f"dry-run: 将输出到 {out}")
            return 0 if result.get('success') else 1

        elif args.command == 'batch':
            res = pipe.batch_process(args.pattern, args.output_dir, getattr(args, 'pipeline', 'full'),
                                     getattr(args, 'overwrite', False), getattr(args, 'dry_run', False))
            print(f"批处理完成: {res.get('successful_files', 0)}/{res.get('total_files', 0)} 文件成功")
            return 0 if res.get('success', False) else 1

        elif args.command == 'report':
            dummy = {'processing_time': 0, 'pipeline': 'unknown', 'processing_results': {}}
            rpt = pipe.generate_quality_report(args.input, args.output, dummy)
            print(f"质量报告: {rpt}")
            return 0 if rpt else 1

        else:
            parser.print_help()
            return 0

    except Exception as e:
        logger.error(f"程序执行失败: {str(e)}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
