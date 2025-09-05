#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MID360 SLAM 地图保存工具
用于保存FAST-LIO2生成的稠密点云地图和轨迹数据
"""

import os
import sys
import argparse
import datetime
import rclpy
from rclpy.node import Node
from interface.srv import SaveMaps, SavePoses

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.get_logger().info('地图保存节点已启动')
        
        # 创建服务客户端
        self.save_maps_client = self.create_client(SaveMaps, 'fastlio2/save_maps')
        self.save_poses_client = self.create_client(SavePoses, 'fastlio2/save_poses')
        
        # 等待服务可用
        self.get_logger().info('等待FAST-LIO2服务可用...')
        if not self.save_maps_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('地图保存服务不可用！请确保FAST-LIO2节点正在运行')
            raise Exception('地图保存服务不可用')
        
        if not self.save_poses_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('轨迹保存服务不可用！请确保FAST-LIO2节点正在运行')
            raise Exception('轨迹保存服务不可用')
        
        self.get_logger().info('所有服务已连接，准备就绪')

    def save_map(self, file_path, save_patches=False):
        """保存稠密点云地图"""
        request = SaveMaps.Request()
        request.file_path = file_path
        request.save_patches = save_patches
        
        self.get_logger().info(f'正在保存地图到: {file_path}')
        future = self.save_maps_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 地图保存成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'❌ 地图保存失败: {response.message}')
                return False
        else:
            self.get_logger().error('❌ 地图保存服务调用失败')
            return False

    def save_poses(self, file_path):
        """保存轨迹数据"""
        request = SavePoses.Request()
        request.file_path = file_path
        
        self.get_logger().info(f'正在保存轨迹到: {file_path}')
        future = self.save_poses_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 轨迹保存成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'❌ 轨迹保存失败: {response.message}')
                return False
        else:
            self.get_logger().error('❌ 轨迹保存服务调用失败')
            return False

def main():
    parser = argparse.ArgumentParser(description='MID360 SLAM 地图保存工具')
    parser.add_argument('--output-dir', '-o', type=str, default='./saved_maps',
                        help='保存目录路径 (默认: ./saved_maps)')
    parser.add_argument('--map-name', '-n', type=str, default=None,
                        help='地图文件名前缀 (默认: 基于时间戳)')
    parser.add_argument('--format', '-f', choices=['pcd', 'ply'], default='pcd',
                        help='点云保存格式 (默认: pcd)')
    parser.add_argument('--save-map', action='store_true', default=True,
                        help='保存点云地图')
    parser.add_argument('--save-poses', action='store_true', default=True,
                        help='保存轨迹数据')
    parser.add_argument('--no-map', action='store_true',
                        help='不保存点云地图')
    parser.add_argument('--no-poses', action='store_true',
                        help='不保存轨迹数据')
    
    args = parser.parse_args()
    
    # 处理参数逻辑
    save_map = args.save_map and not args.no_map
    save_poses = args.save_poses and not args.no_poses
    
    if not save_map and not save_poses:
        print("❌ 错误: 必须至少选择保存地图或轨迹数据中的一项")
        sys.exit(1)
    
    # 确保输出目录存在
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成文件名
    if args.map_name:
        base_name = args.map_name
    else:
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        base_name = f'mid360_slam_{timestamp}'
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        map_saver = MapSaver()
        
        success_count = 0
        total_count = 0
        
        # 保存地图
        if save_map:
            total_count += 1
            map_file = os.path.join(output_dir, f'{base_name}.{args.format}')
            if map_saver.save_map(map_file):
                success_count += 1
        
        # 保存轨迹
        if save_poses:
            total_count += 1
            pose_file = os.path.join(output_dir, f'{base_name}_trajectory.txt')
            if map_saver.save_poses(pose_file):
                success_count += 1
        
        # 报告结果
        print(f'\n📊 保存完成: {success_count}/{total_count} 项成功')
        if success_count == total_count:
            print(f'✅ 所有数据已成功保存到: {output_dir}')
            if save_map:
                print(f'   📍 点云地图: {base_name}.{args.format}')
            if save_poses:
                print(f'   📍 轨迹数据: {base_name}_trajectory.txt')
        else:
            print('⚠️  部分保存失败，请检查日志')
            sys.exit(1)
            
    except Exception as e:
        print(f'❌ 错误: {e}')
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()