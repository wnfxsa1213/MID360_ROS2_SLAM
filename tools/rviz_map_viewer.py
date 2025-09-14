#!/usr/bin/env python3
"""
RViz2地图查看器节点
读取保存的地图和轨迹文件，发布ROS2话题供RViz2显示
"""

import os
import sys
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import struct

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("⚠️ Open3D未安装，将使用基础PCD读取")

class SavedMapPublisher(Node):
    def __init__(self, pcd_file, trajectory_file=None):
        super().__init__('saved_map_publisher')
        
        self.pcd_file = pcd_file
        self.trajectory_file = trajectory_file
        
        # 创建发布器
        self.cloud_pub = self.create_publisher(PointCloud2, '/saved_map/point_cloud', 10)
        self.path_pub = self.create_publisher(Path, '/saved_map/trajectory', 10)
        
        # 定时器，定期发布数据
        self.timer = self.create_timer(1.0, self.publish_data)
        
        # 加载数据
        self.point_cloud = None
        self.trajectory_path = None
        
        self.load_point_cloud()
        if trajectory_file:
            self.load_trajectory()
            
        self.get_logger().info(f'🗺️ 地图查看器节点启动')
        self.get_logger().info(f'📁 点云文件: {pcd_file}')
        if trajectory_file:
            self.get_logger().info(f'📍 轨迹文件: {trajectory_file}')
        
    def load_point_cloud(self):
        """加载PCD点云文件"""
        try:
            if HAS_OPEN3D:
                # 使用Open3D加载
                pcd = o3d.io.read_point_cloud(self.pcd_file)
                if len(pcd.points) == 0:
                    raise ValueError("点云为空")
                    
                points = np.asarray(pcd.points, dtype=np.float32)
                
                # 尝试获取强度信息
                intensities = None
                # 优先从PCD文件读取实际强度数据
                if hasattr(pcd, 'point_cloud_o3d') or self._try_read_intensity_from_file():
                    # 从文件直接读取强度数据
                    intensities = self._extract_intensity_from_pcd()
                else:
                    # 使用基于Z坐标的伪强度值，避免单一颜色
                    z_values = points[:, 2]
                    z_normalized = (z_values - z_values.min()) / (z_values.max() - z_values.min() + 1e-6)
                    intensities = (z_normalized * 255).astype(np.float32)
                    
                self.create_pointcloud2_msg(points, intensities)
                self.get_logger().info(f'✅ 使用Open3D加载点云: {len(points)} 个点')
                
            else:
                # 基础PCD解析
                self.parse_pcd_basic()
                
        except Exception as e:
            self.get_logger().error(f'❌ 加载点云失败: {e}')
            
    def _extract_intensity_from_pcd(self):
        """从PCD文件提取真实强度数据"""
        try:
            return self._parse_pcd_for_intensity()
        except:
            return None
    
    def _try_read_intensity_from_file(self):
        """检查PCD文件是否包含intensity字段"""
        try:
            with open(self.pcd_file, 'r') as f:
                header = f.read(1000)  # 读取前1000字符检查头部
                return 'intensity' in header.lower()
        except:
            return False
            
    def _parse_pcd_for_intensity(self):
        """解析PCD文件获取强度数据"""
        with open(self.pcd_file, 'r') as f:
            lines = f.readlines()
            
        # 查找字段信息
        fields = []
        for line in lines:
            if line.startswith('FIELDS'):
                fields = line.strip().split()[1:]
                break
                
        if 'intensity' not in fields:
            return None
            
        intensity_idx = fields.index('intensity')
        
        # 读取数据行
        data_start = 0
        for i, line in enumerate(lines):
            if line.startswith('DATA'):
                data_start = i + 1
                break
                
        intensities = []
        for line in lines[data_start:data_start + 1000]:  # 限制读取数量
            values = line.strip().split()
            if len(values) > intensity_idx:
                try:
                    intensities.append(float(values[intensity_idx]))
                except:
                    intensities.append(100.0)
                    
        return np.array(intensities, dtype=np.float32) if intensities else None

    def parse_pcd_basic(self):
        """基础PCD文件解析"""
        try:
            with open(self.pcd_file, 'r') as f:
                lines = f.readlines()
                
            # 解析头部
            data_start = 0
            point_count = 0
            fields = []
            
            for i, line in enumerate(lines):
                if line.startswith('FIELDS'):
                    fields = line.strip().split()[1:]
                elif line.startswith('POINTS'):
                    point_count = int(line.strip().split()[1])
                elif line.startswith('DATA'):
                    data_start = i + 1
                    break
                    
            if 'x' not in fields or 'y' not in fields or 'z' not in fields:
                raise ValueError("PCD文件缺少xyz坐标信息")
                
            # 读取数据
            points = []
            intensities = []
            has_intensity = 'intensity' in fields
            
            for line in lines[data_start:data_start + point_count]:
                values = line.strip().split()
                if len(values) >= 3:
                    x, y, z = float(values[0]), float(values[1]), float(values[2])
                    points.append([x, y, z])
                    
                    if has_intensity and len(values) > 3:
                        intensities.append(float(values[3]))
                    else:
                        intensities.append(100.0)
                        
            if points:
                points = np.array(points, dtype=np.float32)
                intensities = np.array(intensities, dtype=np.float32)
                self.create_pointcloud2_msg(points, intensities)
                self.get_logger().info(f'✅ 基础解析加载点云: {len(points)} 个点')
                
        except Exception as e:
            self.get_logger().error(f'❌ 基础PCD解析失败: {e}')
            
    def create_pointcloud2_msg(self, points, intensities):
        """创建PointCloud2消息"""
        # 定义点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 打包数据
        cloud_data = []
        for i in range(len(points)):
            x, y, z = points[i]
            intensity = intensities[i] if i < len(intensities) else 100.0
            # 打包为二进制
            cloud_data.extend(struct.pack('ffff', x, y, z, intensity))
            
        # 创建PointCloud2消息
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        
        self.point_cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=16,  # 4 fields * 4 bytes each
            row_step=16 * len(points),
            data=bytes(cloud_data),
            is_dense=False
        )
        
    def load_trajectory(self):
        """加载轨迹文件"""
        try:
            trajectory_data = []
            
            with open(self.trajectory_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('#') or not line:
                        continue
                    
                    values = line.split()
                    if len(values) >= 8:
                        # 格式: timestamp x y z qx qy qz qw
                        timestamp = float(values[0])
                        x, y, z = float(values[1]), float(values[2]), float(values[3])
                        qx, qy, qz, qw = float(values[4]), float(values[5]), float(values[6]), float(values[7])
                        
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.header.stamp.sec = int(timestamp)
                        pose.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                        
                        pose.pose.position.x = x
                        pose.pose.position.y = y  
                        pose.pose.position.z = z
                        pose.pose.orientation.x = qx
                        pose.pose.orientation.y = qy
                        pose.pose.orientation.z = qz
                        pose.pose.orientation.w = qw
                        
                        trajectory_data.append(pose)
                        
            if trajectory_data:
                self.trajectory_path = Path()
                self.trajectory_path.header.frame_id = 'map'
                self.trajectory_path.header.stamp = self.get_clock().now().to_msg()
                self.trajectory_path.poses = trajectory_data
                
                self.get_logger().info(f'✅ 加载轨迹: {len(trajectory_data)} 个位姿')
            else:
                self.get_logger().warn('⚠️ 轨迹文件为空或格式不正确')
                
        except Exception as e:
            self.get_logger().error(f'❌ 加载轨迹失败: {e}')
            
    def publish_data(self):
        """定期发布数据"""
        current_time = self.get_clock().now().to_msg()
        
        # 发布点云
        if self.point_cloud:
            self.point_cloud.header.stamp = current_time
            self.cloud_pub.publish(self.point_cloud)
            
        # 发布轨迹
        if self.trajectory_path:
            self.trajectory_path.header.stamp = current_time
            self.path_pub.publish(self.trajectory_path)

def main():
    parser = argparse.ArgumentParser(description='RViz2地图查看器节点')
    parser.add_argument('pcd_file', help='PCD点云文件路径')
    parser.add_argument('--trajectory', '-t', help='轨迹文件路径 (可选)')
    parser.add_argument('--duration', '-d', type=float, default=60.0, help='运行时长(秒)，0表示无限')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.pcd_file):
        print(f'❌ PCD文件不存在: {args.pcd_file}')
        sys.exit(1)
        
    if args.trajectory and not os.path.exists(args.trajectory):
        print(f'❌ 轨迹文件不存在: {args.trajectory}')
        sys.exit(1)
        
    rclpy.init()
    
    print(f'🚀 启动RViz2地图查看器...')
    print(f'📁 点云: {args.pcd_file}')
    if args.trajectory:
        print(f'📍 轨迹: {args.trajectory}')
    print(f'⏱️ 运行时长: {args.duration}秒 (0=无限)')
    print(f'')
    print(f'💡 请在另一个终端启动RViz2:')
    print(f'   ros2 run rviz2 rviz2 -d tools/rviz/saved_map_viewer.rviz')
    print(f'')
    
    try:
        node = SavedMapPublisher(args.pcd_file, args.trajectory)
        
        if args.duration > 0:
            # 运行指定时长
            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time) < args.duration:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            # 无限运行
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        print('🛑 用户中断')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print('✅ 节点已关闭')

if __name__ == '__main__':
    main()