#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化的地图保存工具 - 快速保存当前SLAM地图和轨迹
用法: python3 save_map_simple.py [输出目录]
"""

import os
import sys
import datetime
import rclpy
from rclpy.node import Node
from interface.srv import SaveMaps, SavePoses

def main():
    # 获取输出目录
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = './saved_maps'
    
    # 确保输出目录存在
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成时间戳文件名
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    map_file = os.path.join(output_dir, f'mid360_map_{timestamp}.pcd')
    trajectory_file = os.path.join(output_dir, f'mid360_trajectory_{timestamp}.txt')
    
    print(f'🗺️  MID360 SLAM 地图保存工具')
    print(f'📁 保存目录: {output_dir}')
    print(f'📄 地图文件: {os.path.basename(map_file)}')
    print(f'📄 轨迹文件: {os.path.basename(trajectory_file)}')
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        node = Node('quick_map_saver')
        
        # 创建服务客户端
        save_maps_client = node.create_client(SaveMaps, 'fastlio2/save_maps')
        save_poses_client = node.create_client(SavePoses, 'fastlio2/save_poses')
        
        print('🔍 等待FAST-LIO2服务...')
        
        # 等待服务
        if not save_maps_client.wait_for_service(timeout_sec=5.0):
            print('❌ 地图保存服务不可用！请确保FAST-LIO2节点正在运行')
            sys.exit(1)
        
        if not save_poses_client.wait_for_service(timeout_sec=5.0):
            print('❌ 轨迹保存服务不可用！请确保FAST-LIO2节点正在运行')
            sys.exit(1)
        
        success_count = 0
        
        # 保存地图
        print('💾 保存点云地图...')
        request = SaveMaps.Request()
        request.file_path = map_file
        request.save_patches = False
        
        future = save_maps_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() and future.result().success:
            print(f'✅ 地图保存成功: {future.result().message}')
            success_count += 1
        else:
            error_msg = future.result().message if future.result() else '服务调用失败'
            print(f'❌ 地图保存失败: {error_msg}')
        
        # 保存轨迹
        print('💾 保存轨迹数据...')
        request = SavePoses.Request()
        request.file_path = trajectory_file
        
        future = save_poses_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() and future.result().success:
            print(f'✅ 轨迹保存成功: {future.result().message}')
            success_count += 1
        else:
            error_msg = future.result().message if future.result() else '服务调用失败'
            print(f'❌ 轨迹保存失败: {error_msg}')
        
        # 结果报告
        print(f'\n📊 保存完成: {success_count}/2 项成功')
        if success_count == 2:
            print('🎉 所有数据已成功保存！')
            print(f'📁 文件位置: {output_dir}')
        else:
            print('⚠️  部分保存失败')
            sys.exit(1)
        
    except KeyboardInterrupt:
        print('\n⚠️  用户中断操作')
    except Exception as e:
        print(f'❌ 错误: {e}')
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()