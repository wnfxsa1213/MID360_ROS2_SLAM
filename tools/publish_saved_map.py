#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PCD地图发布节点
将保存的PCD文件作为ROS2话题发布，供RViz显示
"""

import os
import sys
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

class PCDMapPublisher(Node):
    def __init__(self, pcd_file, topic_name="/saved_map", frame_id="map", publish_rate=1.0):
        super().__init__('pcd_map_publisher')

        self.pcd_file = pcd_file
        self.frame_id = frame_id

        # 创建发布者
        self.publisher = self.create_publisher(PointCloud2, topic_name, 10)

        # 创建定时器
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 加载点云数据
        self.load_point_cloud()

        self.get_logger().info(f'🗺️  PCD地图发布节点启动')
        self.get_logger().info(f'   文件: {pcd_file}')
        self.get_logger().info(f'   话题: {topic_name}')
        self.get_logger().info(f'   坐标系: {frame_id}')
        self.get_logger().info(f'   发布频率: {publish_rate} Hz')

    def load_point_cloud(self):
        """加载PCD文件"""
        if not os.path.exists(self.pcd_file):
            self.get_logger().error(f'❌ PCD文件不存在: {self.pcd_file}')
            sys.exit(1)

        if not HAS_OPEN3D:
            self.get_logger().error('❌ 需要安装Open3D: pip install open3d')
            sys.exit(1)

        try:
            # 使用Open3D加载点云
            pcd = o3d.io.read_point_cloud(self.pcd_file)

            if len(pcd.points) == 0:
                self.get_logger().error(f'❌ 点云文件为空: {self.pcd_file}')
                sys.exit(1)

            # 转换为numpy数组
            self.points = np.asarray(pcd.points, dtype=np.float32)

            # 检查是否有强度信息
            self.has_intensity = False
            self.intensities = None

            # 尝试从颜色中提取强度（如果有的话）
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
                # 使用灰度值作为强度
                self.intensities = np.mean(colors, axis=1).astype(np.float32) * 255.0
                self.has_intensity = True
                self.get_logger().info(f'✅ 从颜色提取强度信息')
            else:
                # 使用高度作为强度
                z_min, z_max = self.points[:, 2].min(), self.points[:, 2].max()
                if z_max > z_min:
                    self.intensities = ((self.points[:, 2] - z_min) / (z_max - z_min) * 255.0).astype(np.float32)
                else:
                    self.intensities = np.full(len(self.points), 128.0, dtype=np.float32)
                self.has_intensity = True
                self.get_logger().info(f'✅ 使用高度生成强度信息')

            self.get_logger().info(f'✅ 成功加载 {len(self.points):,} 个点')
            self.get_logger().info(f'   X范围: {self.points[:, 0].min():.2f} ~ {self.points[:, 0].max():.2f}m')
            self.get_logger().info(f'   Y范围: {self.points[:, 1].min():.2f} ~ {self.points[:, 1].max():.2f}m')
            self.get_logger().info(f'   Z范围: {self.points[:, 2].min():.2f} ~ {self.points[:, 2].max():.2f}m')

            if self.has_intensity:
                self.get_logger().info(f'   强度范围: {self.intensities.min():.1f} ~ {self.intensities.max():.1f}')

        except Exception as e:
            self.get_logger().error(f'❌ 加载PCD文件失败: {e}')
            sys.exit(1)

    def create_pointcloud2_msg(self):
        """创建PointCloud2消息"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # 定义点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 * 4 bytes for x, y, z

        if self.has_intensity:
            fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
            point_step = 16  # 4 * 4 bytes

        # 打包点云数据
        cloud_data = []
        for i in range(len(self.points)):
            x, y, z = self.points[i]
            if self.has_intensity:
                intensity = self.intensities[i]
                cloud_data.append(struct.pack('ffff', x, y, z, intensity))
            else:
                cloud_data.append(struct.pack('fff', x, y, z))

        # 创建PointCloud2消息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(self.points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = point_step * len(self.points)
        cloud_msg.data = b''.join(cloud_data)
        cloud_msg.is_dense = True

        return cloud_msg

    def timer_callback(self):
        """定时发布回调"""
        if hasattr(self, 'points'):
            msg = self.create_pointcloud2_msg()
            self.publisher.publish(msg)

            # 每10秒打印一次状态
            if not hasattr(self, 'last_log_time'):
                self.last_log_time = 0

            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_log_time > 10.0:
                self.get_logger().info(f'📡 持续发布地图数据 ({len(self.points):,} 点)')
                self.last_log_time = current_time

def main():
    parser = argparse.ArgumentParser(description='发布保存的PCD地图到ROS2话题')
    parser.add_argument('pcd_file', type=str, help='PCD文件路径')
    parser.add_argument('--topic', '-t', type=str, default='/saved_map',
                       help='发布话题名称 (默认: /saved_map)')
    parser.add_argument('--frame', '-f', type=str, default='map',
                       help='坐标系名称 (默认: map)')
    parser.add_argument('--rate', '-r', type=float, default=1.0,
                       help='发布频率 Hz (默认: 1.0)')

    args = parser.parse_args()

    print('🗺️  PCD地图发布器')
    print('=' * 40)

    # 检查文件
    if not os.path.exists(args.pcd_file):
        print(f'❌ PCD文件不存在: {args.pcd_file}')
        sys.exit(1)

    # 初始化ROS2
    rclpy.init()

    try:
        # 创建节点
        node = PCDMapPublisher(
            pcd_file=args.pcd_file,
            topic_name=args.topic,
            frame_id=args.frame,
            publish_rate=args.rate
        )

        print(f'\n📡 开始发布地图数据...')
        print(f'💡 在RViz中添加PointCloud2显示，话题设置为: {args.topic}')
        print(f'⚠️  按Ctrl+C停止发布')

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\n⚠️  用户中断操作')
    except Exception as e:
        print(f'❌ 发布失败: {e}')
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()