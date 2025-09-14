#!/usr/bin/env python3
"""
高级点云处理工具 - 统一强度感知处理
支持完整PCD字段读取和多种增强策略
作者: Claude Code Assistant
"""

import numpy as np
import open3d as o3d
import struct
import argparse
import os
from pathlib import Path
import time
import matplotlib.pyplot as plt

class IntensityAwarePointCloudProcessor:
    def __init__(self, input_pcd_path):
        """初始化强度感知点云处理器"""
        self.input_path = input_pcd_path
        self.points = None
        self.intensity = None
        self.normals = None
        self.curvature = None
        self.field_info = {}
        
    def parse_pcd_header(self):
        """解析PCD文件头部信息"""
        print("解析PCD文件头部...")
        
        # 先以二进制模式找到头部结束位置
        data_offset = 0
        header_lines = []
        
        with open(self.input_path, 'rb') as f:
            while True:
                line_start = f.tell()
                line_bytes = f.readline()
                
                if not line_bytes:
                    break
                    
                try:
                    line = line_bytes.decode('utf-8').strip()
                    header_lines.append(line)
                    
                    if line.startswith('DATA'):
                        data_offset = f.tell()
                        break
                        
                except UnicodeDecodeError:
                    # 遇到二进制数据，回退到行开始
                    f.seek(line_start)
                    data_offset = line_start
                    break
        
        # 解析头部信息
        header = {'data_offset': data_offset}
        
        for line in header_lines:
            if line.startswith('FIELDS'):
                header['fields'] = line.split()[1:]
            elif line.startswith('SIZE'):
                header['sizes'] = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                header['types'] = line.split()[1:]
            elif line.startswith('COUNT'):
                header['counts'] = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'):
                header['points'] = int(line.split()[1])
            elif line.startswith('DATA'):
                header['data_type'] = line.split()[1]
        
        self.field_info = header
        
        print(f"   字段: {header.get('fields', [])}")
        print(f"   点数: {header.get('points', 0):,}")
        print(f"   数据类型: {header.get('data_type', 'unknown')}")
        print(f"   数据偏移: {data_offset}")
        
        return header
        
    def load_pcd_with_intensity(self):
        """加载PCD文件并保留所有字段信息"""
        print("加载PCD文件(保留强度信息)...")
        
        header = self.parse_pcd_header()
        
        if header.get('data_type') != 'binary':
            raise ValueError("错误: 目前仅支持二进制PCD文件")
        
        fields = header['fields']
        sizes = header['sizes']
        types = header['types']
        counts = header['counts']
        num_points = header['points']
        
        # 计算每个点的字节数
        point_size = sum(sizes)
        
        # 读取二进制数据
        with open(self.input_path, 'rb') as f:
            f.seek(header['data_offset'])
            data = f.read(point_size * num_points)
        
        # 解析数据
        parsed_data = {}
        offset = 0
        
        for i, field in enumerate(fields):
            field_type = types[i]
            field_size = sizes[i]
            field_count = counts[i]
            
            # 确定struct格式
            if field_type == 'F':  # float
                fmt = 'f'
            elif field_type == 'U':  # unsigned int
                fmt = 'I' if field_size == 4 else 'H' if field_size == 2 else 'B'
            elif field_type == 'I':  # signed int
                fmt = 'i' if field_size == 4 else 'h' if field_size == 2 else 'b'
            else:
                fmt = 'f'  # 默认
            
            # 提取字段数据
            field_data = []
            for point_idx in range(num_points):
                point_offset = point_idx * point_size + offset
                value = struct.unpack_from(fmt, data, point_offset)[0]
                field_data.append(value)
            
            parsed_data[field] = np.array(field_data)
            offset += field_size
        
        # 存储解析结果
        if 'x' in parsed_data and 'y' in parsed_data and 'z' in parsed_data:
            self.points = np.column_stack([parsed_data['x'], parsed_data['y'], parsed_data['z']])
        
        if 'intensity' in parsed_data:
            self.intensity = parsed_data['intensity']
            print(f"   成功读取强度数据: [{self.intensity.min():.1f}, {self.intensity.max():.1f}]")
        
        if 'normal_x' in parsed_data:
            self.normals = np.column_stack([
                parsed_data['normal_x'], parsed_data['normal_y'], parsed_data['normal_z']
            ])
            print(f"   成功读取法向量数据")
        
        if 'curvature' in parsed_data:
            self.curvature = parsed_data['curvature']
            print(f"   成功读取曲率数据: [{self.curvature.min():.4f}, {self.curvature.max():.4f}]")
        
        print(f"数据统计:")
        print(f"   总点数: {len(self.points):,}")
        print(f"   X范围: [{self.points[:, 0].min():.2f}, {self.points[:, 0].max():.2f}]")
        print(f"   Y范围: [{self.points[:, 1].min():.2f}, {self.points[:, 1].max():.2f}]")
        print(f"   Z范围: [{self.points[:, 2].min():.2f}, {self.points[:, 2].max():.2f}]")
        
        return parsed_data
        
    def create_intensity_colored_pointcloud(self):
        """基于强度创建彩色点云"""
        if self.intensity is None:
            print("无强度信息，创建标准点云")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.points)
            return pcd
        
        print("基于强度创建彩色点云...")
        
        # 标准化强度到[0,1]
        intensity_norm = (self.intensity - self.intensity.min()) / (self.intensity.max() - self.intensity.min())
        
        # 创建热图颜色映射
        colors = np.zeros((len(self.points), 3))
        
        for i, intensity_val in enumerate(intensity_norm):
            if intensity_val < 0.25:  # 低强度 - 深蓝到蓝
                colors[i] = [0, 0, 0.5 + intensity_val * 2]
            elif intensity_val < 0.5:  # 中低强度 - 蓝到青
                t = (intensity_val - 0.25) * 4
                colors[i] = [0, t, 1]
            elif intensity_val < 0.75:  # 中高强度 - 青到黄
                t = (intensity_val - 0.5) * 4
                colors[i] = [t, 1, 1-t]
            else:  # 高强度 - 黄到红
                t = (intensity_val - 0.75) * 4
                colors[i] = [1, 1-t, 0]
        
        # 创建Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if self.normals is not None:
            pcd.normals = o3d.utility.Vector3dVector(self.normals)
        
        print(f"   强度色彩映射完成")
        return pcd
        
    def intensity_based_filtering(self, intensity_range=None, method='preserve_high'):
        """基于强度的智能过滤"""
        if self.intensity is None:
            print("无强度信息，跳过强度过滤")
            return self.points, None
        
        print(f"基于强度的智能过滤 (方法: {method})...")
        
        if intensity_range is None:
            # 自适应强度范围
            q25, q75 = np.percentile(self.intensity, [25, 75])
            iqr = q75 - q25
            
            if method == 'preserve_high':
                # 保留高强度点（通常是反射率好的表面）
                threshold = q75 + 0.5 * iqr
                mask = self.intensity >= threshold
                print(f"   高强度阈值: {threshold:.1f}")
            elif method == 'remove_noise':
                # 移除异常低强度点（可能是噪声）
                threshold = q25 - 1.5 * iqr
                mask = self.intensity >= max(threshold, 0)
                print(f"   噪声过滤阈值: {max(threshold, 0):.1f}")
            elif method == 'balanced':
                # 平衡策略：移除极值
                low_thresh = q25 - 1.5 * iqr
                high_thresh = q75 + 1.5 * iqr
                mask = (self.intensity >= max(low_thresh, 0)) & (self.intensity <= high_thresh)
                print(f"   平衡过滤范围: [{max(low_thresh, 0):.1f}, {high_thresh:.1f}]")
            else:
                mask = np.ones(len(self.intensity), dtype=bool)
        else:
            # 用户指定范围
            mask = (self.intensity >= intensity_range[0]) & (self.intensity <= intensity_range[1])
            print(f"   自定义强度范围: [{intensity_range[0]:.1f}, {intensity_range[1]:.1f}]")
        
        # 应用过滤
        filtered_points = self.points[mask]
        filtered_intensity = self.intensity[mask]
        filtered_normals = self.normals[mask] if self.normals is not None else None
        filtered_curvature = self.curvature[mask] if self.curvature is not None else None
        
        original_count = len(self.points)
        filtered_count = len(filtered_points)
        
        print(f"   过滤结果: {original_count:,} -> {filtered_count:,} ({filtered_count/original_count*100:.1f}%)")
        
        return filtered_points, filtered_intensity, filtered_normals, filtered_curvature
        
    def curvature_based_enhancement(self, curvature_threshold=0.1):
        """基于曲率的细节增强"""
        if self.curvature is None:
            print("警告: 无曲率信息，跳过曲率增强")
            return self.points
        
        print(f"几何: 基于曲率的细节增强 (阈值: {curvature_threshold})...")
        
        # 高曲率区域通常对应边缘和细节
        high_curvature_mask = self.curvature > curvature_threshold
        high_curvature_count = np.sum(high_curvature_mask)
        
        print(f"   高曲率点数: {high_curvature_count:,} ({high_curvature_count/len(self.points)*100:.1f}%)")
        
        if high_curvature_count > 0:
            # 在高曲率点周围增加细节点
            high_curv_points = self.points[high_curvature_mask]
            enhanced_points = []
            
            # 在每个高曲率点周围生成精细采样点
            for point in high_curv_points[:min(len(high_curv_points), 10000)]:  # 限制处理数量
                # 生成2个邻近点
                for _ in range(2):
                    noise = np.random.normal(0, 0.005, 3)  # 5mm标准差
                    enhanced_points.append(point + noise)
            
            if enhanced_points:
                all_points = np.vstack([self.points, enhanced_points])
                print(f"   增加细节点: {len(enhanced_points):,} 个")
                return all_points
        
        return self.points
        
    def _post_process_mesh(self, mesh):
        """网格后处理：清理和平滑"""
        if mesh is None:
            return None
            
        print("   网格后处理...")
        
        # 移除重复顶点和三角面
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_non_manifold_edges()
        
        # 平滑处理
        mesh = mesh.filter_smooth_simple(number_of_iterations=1)
        
        return mesh
        
    def _apply_density_filtering(self, mesh, densities, density_threshold=0.1):
        """应用密度过滤到Poisson重建网格"""
        if density_threshold <= 0 or len(densities) == 0:
            return mesh
            
        print(f"   密度过滤 (阈值: {density_threshold})...")
        densities = np.asarray(densities)
        
        if len(densities) > 0 and densities.max() > densities.min():
            # 添加密度颜色映射
            density_colors = plt.cm.plasma((densities - densities.min()) / (densities.max() - densities.min()))
            mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors[:, :3])
            
            # 移除低密度顶点
            vertices_to_remove = densities < np.quantile(densities, density_threshold)
            mesh.remove_vertices_by_mask(vertices_to_remove)
        else:
            print("   警告: 密度信息不足，跳过密度过滤")
            
        return mesh
        
    def surface_reconstruction(self, method='poisson', depth=9, density_threshold=0.1, alpha=0.03):
        """表面重建功能"""
        if method == 'poisson':
            print(f"表面重建: 使用{method}方法 (深度: {depth})...")
        elif method == 'alpha_shape':
            print(f"表面重建: 使用{method}方法 (alpha: {alpha})...")
        else:
            print(f"表面重建: 使用{method}方法...")
        
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        
        # 估计法向量（如果没有）
        if self.normals is None:
            print("   估计法向量...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            # 调整法向量方向
            pcd.orient_normals_consistent_tangent_plane(k=15)
        else:
            pcd.normals = o3d.utility.Vector3dVector(self.normals)
        
        mesh = None
        
        if method == 'poisson':
            # Poisson表面重建 - 适用于封闭表面
            print("   执行Poisson重建...")
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd, depth=depth, width=0, scale=1.1, linear_fit=False
            )
            # 应用密度过滤
            mesh = self._apply_density_filtering(mesh, densities, density_threshold)
            
        elif method == 'alpha_shape':
            # Alpha Shape重建 - 适用于复杂几何
            print("   执行Alpha Shape重建...")
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
            
        elif method == 'ball_pivoting':
            # Ball Pivoting重建 - 适用于均匀采样
            print("   执行Ball Pivoting重建...")
            distances = pcd.compute_nearest_neighbor_distance()
            avg_dist = np.mean(distances)
            radius = avg_dist * 1.5
            
            radii = [radius, radius * 2, radius * 4]
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                pcd, o3d.utility.DoubleVector(radii)
            )
            
        elif method == 'delaunay_2d':
            # 2D Delaunay三角化（适用于相对平坦的表面）
            print("   执行2D Delaunay重建...")
            mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_delaunay_2d(pcd)
        else:
            print(f"   错误: 不支持的重建方法 '{method}'")
            return None
            
        if mesh is None:
            print("   警告: 表面重建失败")
            return None
            
        # 统一的网格后处理
        mesh = self._post_process_mesh(mesh)
        
        # 计算网格统计信息
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        
        print(f"   重建完成:")
        print(f"     顶点数: {len(vertices):,}")
        print(f"     三角面数: {len(triangles):,}")
        
        if len(vertices) > 0:
            print(f"     边界框: [{vertices.min(axis=0)} - {vertices.max(axis=0)}]")
        else:
            print("     警告: 生成的网格为空")
            return None
        
        return mesh
        
    def mesh_quality_analysis(self, mesh):
        """网格质量分析"""
        if mesh is None:
            return None
            
        print("网格质量分析...")
        
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        
        # 计算三角面积
        triangle_areas = []
        edge_lengths = []
        
        for tri in triangles:
            v0, v1, v2 = vertices[tri]
            
            # 计算边长
            e1_len = np.linalg.norm(v1 - v0)
            e2_len = np.linalg.norm(v2 - v1) 
            e3_len = np.linalg.norm(v0 - v2)
            edge_lengths.extend([e1_len, e2_len, e3_len])
            
            # 计算面积（海伦公式）
            s = (e1_len + e2_len + e3_len) / 2
            area = np.sqrt(max(0, s * (s - e1_len) * (s - e2_len) * (s - e3_len)))
            triangle_areas.append(area)
        
        triangle_areas = np.array(triangle_areas)
        edge_lengths = np.array(edge_lengths)
        
        # 计算网格质量指标
        total_area = np.sum(triangle_areas)
        avg_edge_length = np.mean(edge_lengths)
        edge_length_std = np.std(edge_lengths)
        
        # 检查退化三角形（面积过小）
        degenerate_count = np.sum(triangle_areas < avg_edge_length * avg_edge_length * 0.001)
        
        quality_stats = {
            'vertex_count': len(vertices),
            'triangle_count': len(triangles),
            'total_surface_area': total_area,
            'avg_edge_length': avg_edge_length,
            'edge_length_std': edge_length_std,
            'degenerate_triangles': degenerate_count,
            'mesh_volume': mesh.get_volume() if mesh.is_watertight() else 0,
            'is_watertight': mesh.is_watertight()
        }
        
        print(f"   质量统计:")
        print(f"     表面积: {total_area:.3f}")
        print(f"     平均边长: {avg_edge_length:.4f}")
        print(f"     边长标准差: {edge_length_std:.4f}")
        print(f"     退化三角形: {degenerate_count}")
        print(f"     是否封闭: {quality_stats['is_watertight']}")
        if quality_stats['is_watertight']:
            print(f"     网格体积: {quality_stats['mesh_volume']:.3f}")
        
        return quality_stats
        
    def adaptive_outlier_removal(self, method='statistical'):
        """自适应离群点移除"""
        print(f"自适应离群点移除 (方法: {method})...")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        
        original_count = len(self.points)
        
        if method == 'statistical':
            # 基于密度的统计方法
            # 根据点云密度自适应调整参数
            avg_density = len(self.points) / (np.prod(self.points.max(axis=0) - self.points.min(axis=0)))
            
            if avg_density > 1000:  # 高密度
                nb_neighbors, std_ratio = 30, 2.5
            elif avg_density > 100:  # 中密度
                nb_neighbors, std_ratio = 20, 2.0
            else:  # 低密度
                nb_neighbors, std_ratio = 15, 1.5
            
            pcd, inlier_indices = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            
        elif method == 'radius':
            # 基于半径的方法
            avg_distance = np.mean([np.linalg.norm(self.points[i] - self.points[i+1]) for i in range(0, len(self.points)-1, 100)])
            radius = avg_distance * 5
            min_points = 10
            
            pcd, inlier_indices = pcd.remove_radius_outlier(nb_points=min_points, radius=radius)
        
        filtered_count = len(pcd.points)
        removal_rate = (original_count - filtered_count) / original_count * 100
        
        print(f"   移除点数: {original_count - filtered_count:,} ({removal_rate:.2f}%)")
        
        # 同步更新所有数组
        self.points = np.asarray(pcd.points)
        
        if self.intensity is not None:
            self.intensity = self.intensity[inlier_indices]
        
        if self.normals is not None:
            self.normals = self.normals[inlier_indices]
            
        if self.curvature is not None:
            self.curvature = self.curvature[inlier_indices]
        
        return self.points
        
    def save_enhanced_pcd(self, output_path, enhanced_points=None, enhanced_intensity=None, 
                         enhanced_normals=None, enhanced_curvature=None):
        """保存增强后的PCD文件（保留所有字段）"""
        print(f"保存: 保存增强PCD文件: {output_path}")
        
        points_to_save = enhanced_points if enhanced_points is not None else self.points
        intensity_to_save = enhanced_intensity if enhanced_intensity is not None else self.intensity
        normals_to_save = enhanced_normals if enhanced_normals is not None else self.normals
        curvature_to_save = enhanced_curvature if enhanced_curvature is not None else self.curvature
        
        # 构建PCD文件头部
        fields = ['x', 'y', 'z']
        if intensity_to_save is not None:
            fields.append('intensity')
        if normals_to_save is not None:
            fields.extend(['normal_x', 'normal_y', 'normal_z'])
        if curvature_to_save is not None:
            fields.append('curvature')
        
        num_points = len(points_to_save)
        
        # 写入PCD文件
        with open(output_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write(f"FIELDS {' '.join(fields)}\n")
            f.write(f"SIZE {' '.join(['4'] * len(fields))}\n")
            f.write(f"TYPE {' '.join(['F'] * len(fields))}\n")
            f.write(f"COUNT {' '.join(['1'] * len(fields))}\n")
            f.write(f"WIDTH {num_points}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {num_points}\n")
            f.write("DATA ascii\n")
            
            # 写入数据
            for i in range(num_points):
                line_data = [f"{points_to_save[i, 0]:.6f}", f"{points_to_save[i, 1]:.6f}", f"{points_to_save[i, 2]:.6f}"]
                
                if intensity_to_save is not None:
                    line_data.append(f"{intensity_to_save[i]:.3f}")
                
                if normals_to_save is not None:
                    line_data.extend([f"{normals_to_save[i, 0]:.6f}", f"{normals_to_save[i, 1]:.6f}", f"{normals_to_save[i, 2]:.6f}"])
                
                if curvature_to_save is not None:
                    line_data.append(f"{curvature_to_save[i]:.6f}")
                
                f.write(" ".join(line_data) + "\n")
        
        print(f"   完成: 保存完成: {num_points:,} 点")
        return True
        
    def process_full_pipeline(self, output_path, processing_options=None):
        """完整处理流水线"""
        print("开始: 开始高级点云处理流水线")
        start_time = time.time()
        
        if processing_options is None:
            processing_options = {
                'outlier_removal': True,
                'intensity_filtering': 'balanced',
                'curvature_enhancement': True,
                'create_colored_cloud': True,
                'surface_reconstruction': False,
                'reconstruction_method': 'poisson'
            }
        
        # 1. 加载数据（保留所有字段）
        self.load_pcd_with_intensity()
        original_count = len(self.points)
        
        # 2. 自适应离群点移除
        if processing_options.get('outlier_removal', True):
            self.adaptive_outlier_removal(method='statistical')
        
        # 3. 基于强度的过滤
        intensity_method = processing_options.get('intensity_filtering', 'balanced')
        if intensity_method and self.intensity is not None:
            filtered_points, filtered_intensity, filtered_normals, filtered_curvature = \
                self.intensity_based_filtering(method=intensity_method)
            
            # 更新数据
            self.points = filtered_points
            self.intensity = filtered_intensity
            self.normals = filtered_normals
            self.curvature = filtered_curvature
        
        # 4. 基于曲率的细节增强
        if processing_options.get('curvature_enhancement', True) and self.curvature is not None:
            enhanced_points = self.curvature_based_enhancement()
            if len(enhanced_points) > len(self.points):
                # 扩展其他数组以匹配新点数
                additional_count = len(enhanced_points) - len(self.points)
                if self.intensity is not None:
                    # 为新点分配平均强度
                    avg_intensity = np.mean(self.intensity)
                    self.intensity = np.concatenate([self.intensity, np.full(additional_count, avg_intensity)])
                self.points = enhanced_points
        
        # 5. 表面重建（新增功能）
        mesh = None
        mesh_stats = None
        if processing_options.get('surface_reconstruction', False):
            # 统一的参数提取和传递
            mesh_params = {
                'method': processing_options.get('reconstruction_method', 'poisson'),
                'depth': processing_options.get('poisson_depth', 9),
                'alpha': processing_options.get('alpha_value', 0.03),
                'density_threshold': processing_options.get('density_threshold', 0.1)
            }
            
            mesh = self.surface_reconstruction(**mesh_params)
            
            if mesh is not None:
                # 保存网格
                mesh_output = str(output_path).replace('.pcd', f'_mesh_{mesh_params["method"]}.ply')
                o3d.io.write_triangle_mesh(mesh_output, mesh)
                print(f"   网格保存: {mesh_output}")
                
                # 网格质量分析
                mesh_stats = self.mesh_quality_analysis(mesh)
        
        # 6. 保存增强后的PCD（保留所有字段）
        self.save_enhanced_pcd(output_path, self.points, self.intensity, self.normals, self.curvature)
        
        # 7. 创建可视化用的彩色点云
        if processing_options.get('create_colored_cloud', True):
            colored_pcd = self.create_intensity_colored_pointcloud()
            colored_output = str(output_path).replace('.pcd', '_colored.pcd')
            o3d.io.write_point_cloud(colored_output, colored_pcd)
            print(f"   提示: 彩色可视化文件: {colored_output}")
        
        elapsed_time = time.time() - start_time
        final_count = len(self.points)
        
        print(f"\n完成: 高级点云处理完成!")
        print(f"   处理时间: {elapsed_time:.2f} 秒")
        print(f"   原始点数: {original_count:,}")
        print(f"   处理后点数: {final_count:,}")
        print(f"   变化比例: {(final_count/original_count-1)*100:.1f}%")
        print(f"   输出文件: {output_path}")
        
        # 解释点云数量变化
        if final_count < original_count:
            reduction_rate = (1 - final_count/original_count) * 100
            print(f"\n点云数量减少原因分析:")
            print(f"   减少比例: {reduction_rate:.1f}%")
            print(f"   主要原因: 离群点移除 + 强度过滤")
            if processing_options.get('outlier_removal'):
                print(f"   - 统计离群点移除: 去除噪声和异常点")
            if intensity_method:
                print(f"   - 强度过滤({intensity_method}): 保留质量更好的点")
        elif final_count > original_count:
            increase_rate = (final_count/original_count - 1) * 100
            print(f"   点云增强: +{increase_rate:.1f}% (曲率细节增强)")
        
        return True, mesh_stats

def main():
    parser = argparse.ArgumentParser(description="高级点云处理工具 - 强度感知与表面重建")
    parser.add_argument("input", help="输入PCD文件路径")
    parser.add_argument("-o", "--output", help="输出PCD文件路径")
    parser.add_argument("--method", choices=['conservative', 'balanced', 'aggressive'], 
                       default='balanced', help="处理强度")
    parser.add_argument("--no-intensity-filter", action="store_true", help="跳过强度过滤")
    parser.add_argument("--no-curvature-enhance", action="store_true", help="跳过曲率增强")
    parser.add_argument("--intensity-range", nargs=2, type=float, help="自定义强度范围 min max")
    
    # 表面重建参数
    parser.add_argument("--surface-reconstruction", action="store_true", help="启用表面重建")
    parser.add_argument("--reconstruction-method", choices=['poisson', 'alpha_shape', 'ball_pivoting', 'delaunay_2d'],
                       default='poisson', help="表面重建方法")
    parser.add_argument("--poisson-depth", type=int, default=9, help="Poisson重建深度 (6-12)")
    parser.add_argument("--alpha-value", type=float, default=0.03, help="Alpha Shape参数")
    parser.add_argument("--density-threshold", type=float, default=0.1, help="Poisson密度过滤阈值 (0-1)")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"错误: 输入文件不存在: {args.input}")
        return
    
    if args.output:
        output_path = args.output
    else:
        input_path = Path(args.input)
        output_path = input_path.parent / f"enhanced_{input_path.name}"
    
    # 配置处理选项
    processing_options = {
        'outlier_removal': True,
        'intensity_filtering': None if args.no_intensity_filter else args.method,
        'curvature_enhancement': not args.no_curvature_enhance,
        'create_colored_cloud': True,
        'surface_reconstruction': args.surface_reconstruction,
        'reconstruction_method': args.reconstruction_method,
        'poisson_depth': args.poisson_depth,
        'alpha_value': args.alpha_value,
        'density_threshold': args.density_threshold
    }
    
    # 创建处理器
    processor = IntensityAwarePointCloudProcessor(args.input)
    
    try:
        _, mesh_stats = processor.process_full_pipeline(str(output_path), processing_options)
        
        print(f"\n提示: 使用建议:")
        print(f"   原文件: {args.input}")  
        print(f"   增强文件: {output_path}")
        print(f"   彩色文件: {str(output_path).replace('.pcd', '_colored.pcd')}")
        
        if args.surface_reconstruction and mesh_stats:
            mesh_file = str(output_path).replace('.pcd', f'_mesh_{args.reconstruction_method}.ply')
            print(f"   网格文件: {mesh_file}")
            print(f"   网格统计: {mesh_stats['triangle_count']:,} 三角面, {mesh_stats['vertex_count']:,} 顶点")
        
        print(f"   RViz查看: python3 tools/rviz_map_viewer.py {output_path}")
        
    except Exception as e:
        print(f"错误: 处理失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()