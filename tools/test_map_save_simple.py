#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的地图保存功能测试工具
用于测试SLAM地图保存功能是否正常工作
"""

import os
import sys
import rclpy
from rclpy.node import Node
from interface.srv import SaveMaps, SavePoses

class MapSaveTest(Node):
    def __init__(self):
        super().__init__('map_save_test')
        
    def test_services(self):
        """测试服务是否可用"""
        print("🔍 测试SLAM地图保存服务...")
        
        # 创建服务客户端
        save_maps_client = self.create_client(SaveMaps, 'fastlio2/save_maps')
        save_poses_client = self.create_client(SavePoses, 'fastlio2/save_poses')
        
        # 测试地图保存服务
        print("检查地图保存服务...", end=" ")
        if save_maps_client.wait_for_service(timeout_sec=3.0):
            print("✅ 可用")
        else:
            print("❌ 不可用")
            return False
        
        # 测试轨迹保存服务  
        print("检查轨迹保存服务...", end=" ")
        if save_poses_client.wait_for_service(timeout_sec=3.0):
            print("✅ 可用")
        else:
            print("❌ 不可用")
            return False
        
        return True

def main():
    print("🧪 MID360 SLAM 地图保存功能测试")
    print("=" * 40)
    
    # 初始化ROS2
    try:
        rclpy.init()
        
        # 创建测试节点
        test_node = MapSaveTest()
        
        # 测试服务可用性
        if test_node.test_services():
            print("\n✅ 所有服务测试通过！")
            print("地图保存功能已准备就绪")
        else:
            print("\n❌ 服务测试失败")
            print("请确保FAST-LIO2节点正在运行")
            sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n⚠️  测试被中断")
    except Exception as e:
        print(f"\n❌ 测试错误: {e}")
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()