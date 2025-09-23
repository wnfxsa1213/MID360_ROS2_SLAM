#!/usr/bin/env python3
"""
MID360 SLAM系统性能分析工具
专门用于系统集成验证和性能基准测试
"""

import os
import sys
import time
import psutil
import subprocess
import threading
import signal
import json
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
import yaml

@dataclass
class SystemMetrics:
    """系统性能指标"""
    timestamp: float
    cpu_percent: float
    memory_percent: float
    memory_mb: float
    network_sent_mbps: float
    network_recv_mbps: float
    disk_read_mbps: float
    disk_write_mbps: float
    process_count: int

@dataclass
class SlamMetrics:
    """SLAM系统指标"""
    timestamp: float
    fastlio_processing_time: float
    point_cloud_size: int
    imu_buffer_size: int
    lidar_buffer_size: int
    optimization_drift: float
    successful_optimizations: int
    failed_optimizations: int

@dataclass
class NetworkMetrics:
    """网络性能指标"""
    timestamp: float
    latency_ms: float
    packet_loss_percent: float
    bandwidth_mbps: float
    jitter_ms: float

class PerformanceAnalyzer:
    """MID360 SLAM性能分析器"""

    def __init__(self, analysis_duration: int = 60):
        self.analysis_duration = analysis_duration
        self.system_metrics: List[SystemMetrics] = []
        self.slam_metrics: List[SlamMetrics] = []
        self.network_metrics: List[NetworkMetrics] = []
        self.monitoring = False
        self.start_time = None

        # 网络配置
        self.lidar_ip = "192.168.1.3"
        self.host_ip = "192.168.1.50"

        # 性能阈值（基于穿越机要求）
        self.performance_thresholds = {
            'max_processing_time_ms': 50,      # 最大处理延迟50ms
            'target_processing_time_ms': 20,   # 目标处理延迟20ms
            'max_cpu_percent': 80,             # 最大CPU使用率80%
            'max_memory_percent': 70,          # 最大内存使用率70%
            'min_point_cloud_hz': 8,           # 最小点云频率8Hz
            'min_imu_hz': 800,                 # 最小IMU频率800Hz
            'max_network_latency_ms': 5,       # 最大网络延迟5ms
            'max_packet_loss_percent': 0.1,    # 最大丢包率0.1%
        }

    def start_analysis(self):
        """开始性能分析"""
        print(f"🔍 开始MID360 SLAM系统性能分析 (持续时间: {self.analysis_duration}秒)")
        print("="*60)

        self.monitoring = True
        self.start_time = time.time()

        # 启动监控线程
        threads = [
            threading.Thread(target=self._monitor_system_metrics, daemon=True),
            threading.Thread(target=self._monitor_slam_metrics, daemon=True),
            threading.Thread(target=self._monitor_network_metrics, daemon=True)
        ]

        for thread in threads:
            thread.start()

        # 实时显示分析进度
        self._display_realtime_analysis()

        # 等待分析完成
        time.sleep(max(0, self.analysis_duration - (time.time() - self.start_time)))
        self.monitoring = False

        # 等待线程结束
        for thread in threads:
            thread.join(timeout=1)

        return self._generate_analysis_report()

    def _monitor_system_metrics(self):
        """监控系统资源指标"""
        last_net_io = psutil.net_io_counters()
        last_disk_io = psutil.disk_io_counters()
        last_time = time.time()

        while self.monitoring:
            try:
                current_time = time.time()

                # 网络IO计算
                current_net_io = psutil.net_io_counters()
                net_sent_mbps = (current_net_io.bytes_sent - last_net_io.bytes_sent) / (current_time - last_time) / 1024 / 1024
                net_recv_mbps = (current_net_io.bytes_recv - last_net_io.bytes_recv) / (current_time - last_time) / 1024 / 1024

                # 磁盘IO计算
                current_disk_io = psutil.disk_io_counters()
                disk_read_mbps = (current_disk_io.read_bytes - last_disk_io.read_bytes) / (current_time - last_time) / 1024 / 1024
                disk_write_mbps = (current_disk_io.write_bytes - last_disk_io.write_bytes) / (current_time - last_time) / 1024 / 1024

                # 收集系统指标
                metrics = SystemMetrics(
                    timestamp=current_time,
                    cpu_percent=psutil.cpu_percent(),
                    memory_percent=psutil.virtual_memory().percent,
                    memory_mb=psutil.virtual_memory().used / 1024 / 1024,
                    network_sent_mbps=net_sent_mbps,
                    network_recv_mbps=net_recv_mbps,
                    disk_read_mbps=disk_read_mbps,
                    disk_write_mbps=disk_write_mbps,
                    process_count=len(psutil.pids())
                )

                self.system_metrics.append(metrics)

                last_net_io = current_net_io
                last_disk_io = current_disk_io
                last_time = current_time

                time.sleep(1)

            except Exception as e:
                print(f"系统监控错误: {e}")
                time.sleep(1)

    def _monitor_slam_metrics(self):
        """监控SLAM性能指标"""
        while self.monitoring:
            try:
                # 尝试从ROS2话题获取性能指标
                fastlio_metrics = self._get_fastlio_metrics()
                coordinator_metrics = self._get_coordinator_metrics()

                if fastlio_metrics or coordinator_metrics:
                    metrics = SlamMetrics(
                        timestamp=time.time(),
                        fastlio_processing_time=fastlio_metrics.get('processing_time', 0),
                        point_cloud_size=fastlio_metrics.get('point_count', 0),
                        imu_buffer_size=fastlio_metrics.get('imu_buffer_size', 0),
                        lidar_buffer_size=fastlio_metrics.get('lidar_buffer_size', 0),
                        optimization_drift=coordinator_metrics.get('drift', 0),
                        successful_optimizations=coordinator_metrics.get('successful', 0),
                        failed_optimizations=coordinator_metrics.get('failed', 0)
                    )
                    self.slam_metrics.append(metrics)

                time.sleep(0.5)  # 2Hz监控频率

            except Exception as e:
                print(f"SLAM监控错误: {e}")
                time.sleep(1)

    def _monitor_network_metrics(self):
        """监控网络性能指标"""
        while self.monitoring:
            try:
                # Ping测试延迟
                latency = self._measure_network_latency()

                # 网络带宽测试（简化版）
                bandwidth = self._estimate_network_bandwidth()

                metrics = NetworkMetrics(
                    timestamp=time.time(),
                    latency_ms=latency,
                    packet_loss_percent=0.0,  # 简化实现
                    bandwidth_mbps=bandwidth,
                    jitter_ms=0.0  # 简化实现
                )

                self.network_metrics.append(metrics)
                time.sleep(2)  # 0.5Hz监控频率

            except Exception as e:
                print(f"网络监控错误: {e}")
                time.sleep(2)

    def _get_fastlio_metrics(self) -> Dict:
        """获取FASTLIO2性能指标"""
        try:
            # 使用ros2 topic echo获取性能数据
            result = subprocess.run([
                'timeout', '1s', 'ros2', 'topic', 'echo',
                '/fastlio2/performance_metrics', '--once'
            ], capture_output=True, text=True, timeout=2)

            if result.returncode == 0:
                # 解析输出数据
                lines = result.stdout.strip().split('\n')
                data_line = None
                for line in lines:
                    if 'data:' in line:
                        data_line = line
                        break

                if data_line:
                    # 提取数组数据 [processing_time, point_count, imu_buffer, lidar_buffer]
                    data_str = data_line.split('data:')[1].strip()
                    data_str = data_str.strip('[]')
                    values = [float(x.strip()) for x in data_str.split(',')]

                    return {
                        'processing_time': values[0] if len(values) > 0 else 0,
                        'point_count': int(values[1]) if len(values) > 1 else 0,
                        'imu_buffer_size': int(values[2]) if len(values) > 2 else 0,
                        'lidar_buffer_size': int(values[3]) if len(values) > 3 else 0
                    }
        except:
            pass
        return {}

    def _get_coordinator_metrics(self) -> Dict:
        """获取协调器性能指标"""
        try:
            result = subprocess.run([
                'timeout', '1s', 'ros2', 'topic', 'echo',
                '/coordinator/metrics', '--once'
            ], capture_output=True, text=True, timeout=2)

            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                data_line = None
                for line in lines:
                    if 'data:' in line:
                        data_line = line
                        break

                if data_line:
                    data_str = data_line.split('data:')[1].strip()
                    data_str = data_str.strip('[]')
                    values = [float(x.strip()) for x in data_str.split(',')]

                    return {
                        'drift': values[0] if len(values) > 0 else 0,
                        'score': values[1] if len(values) > 1 else 0,
                        'successful': int(values[2]) if len(values) > 2 else 0,
                        'failed': int(values[3]) if len(values) > 3 else 0
                    }
        except:
            pass
        return {}

    def _measure_network_latency(self) -> float:
        """测量网络延迟"""
        try:
            result = subprocess.run([
                'ping', '-c', '1', '-W', '1000', self.lidar_ip
            ], capture_output=True, text=True)

            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'time=' in line:
                        time_str = line.split('time=')[1].split()[0]
                        return float(time_str)
        except:
            pass
        return 999.0  # 超时或错误

    def _estimate_network_bandwidth(self) -> float:
        """估算网络带宽"""
        # 简化实现：基于网络接口统计
        try:
            net_io = psutil.net_io_counters()
            # 返回一个估算值，实际应该进行带宽测试
            return 100.0  # 假设100Mbps
        except:
            return 0.0

    def _display_realtime_analysis(self):
        """实时显示分析进度"""
        while self.monitoring and (time.time() - self.start_time) < self.analysis_duration:
            try:
                elapsed = time.time() - self.start_time
                remaining = max(0, self.analysis_duration - elapsed)

                # 获取最新指标
                latest_system = self.system_metrics[-1] if self.system_metrics else None
                latest_slam = self.slam_metrics[-1] if self.slam_metrics else None
                latest_network = self.network_metrics[-1] if self.network_metrics else None

                # 清屏并显示实时数据
                os.system('clear')
                print(f"🔍 MID360 SLAM系统性能分析进行中 ({elapsed:.1f}/{self.analysis_duration}s)")
                print("="*60)

                if latest_system:
                    cpu_status = "✅" if latest_system.cpu_percent < 80 else "⚠️"
                    mem_status = "✅" if latest_system.memory_percent < 70 else "⚠️"
                    print(f"{cpu_status} CPU使用率: {latest_system.cpu_percent:.1f}%")
                    print(f"{mem_status} 内存使用率: {latest_system.memory_percent:.1f}% ({latest_system.memory_mb:.0f}MB)")
                    print(f"📡 网络: ↑{latest_system.network_sent_mbps:.2f} ↓{latest_system.network_recv_mbps:.2f} MB/s")

                if latest_slam:
                    proc_status = "✅" if latest_slam.fastlio_processing_time < 50 else "⚠️"
                    print(f"{proc_status} SLAM处理时间: {latest_slam.fastlio_processing_time:.1f}ms")
                    print(f"📊 点云大小: {latest_slam.point_cloud_size}")
                    print(f"📈 缓冲区: IMU({latest_slam.imu_buffer_size}) LiDAR({latest_slam.lidar_buffer_size})")

                if latest_network:
                    net_status = "✅" if latest_network.latency_ms < 10 else "⚠️"
                    print(f"{net_status} 网络延迟: {latest_network.latency_ms:.1f}ms")

                print(f"\n⏱️  剩余时间: {remaining:.1f}秒")
                print("按Ctrl+C提前结束分析...")

                time.sleep(2)

            except KeyboardInterrupt:
                print("\n\n用户中断分析...")
                self.monitoring = False
                break
            except Exception as e:
                print(f"显示错误: {e}")
                time.sleep(1)

    def _generate_analysis_report(self) -> Dict:
        """生成分析报告"""
        print("\n\n" + "="*60)
        print("📊 生成性能分析报告...")

        report = {
            'analysis_info': {
                'duration_seconds': self.analysis_duration,
                'start_time': datetime.fromtimestamp(self.start_time).isoformat(),
                'end_time': datetime.now().isoformat(),
                'samples_collected': {
                    'system_metrics': len(self.system_metrics),
                    'slam_metrics': len(self.slam_metrics),
                    'network_metrics': len(self.network_metrics)
                }
            },
            'system_performance': self._analyze_system_performance(),
            'slam_performance': self._analyze_slam_performance(),
            'network_performance': self._analyze_network_performance(),
            'integration_quality': self._analyze_integration_quality(),
            'recommendations': self._generate_recommendations()
        }

        return report

    def _analyze_system_performance(self) -> Dict:
        """分析系统性能"""
        if not self.system_metrics:
            return {'status': 'no_data'}

        cpu_values = [m.cpu_percent for m in self.system_metrics]
        memory_values = [m.memory_percent for m in self.system_metrics]

        return {
            'cpu_utilization': {
                'average': sum(cpu_values) / len(cpu_values),
                'maximum': max(cpu_values),
                'minimum': min(cpu_values),
                'above_threshold_percent': len([x for x in cpu_values if x > 80]) / len(cpu_values) * 100
            },
            'memory_utilization': {
                'average': sum(memory_values) / len(memory_values),
                'maximum': max(memory_values),
                'minimum': min(memory_values),
                'above_threshold_percent': len([x for x in memory_values if x > 70]) / len(memory_values) * 100
            },
            'status': 'excellent' if max(cpu_values) < 60 and max(memory_values) < 50 else
                     'good' if max(cpu_values) < 80 and max(memory_values) < 70 else 'needs_optimization'
        }

    def _analyze_slam_performance(self) -> Dict:
        """分析SLAM性能"""
        if not self.slam_metrics:
            return {'status': 'no_data'}

        processing_times = [m.fastlio_processing_time for m in self.slam_metrics if m.fastlio_processing_time > 0]

        if not processing_times:
            return {'status': 'insufficient_data'}

        avg_processing_time = sum(processing_times) / len(processing_times)
        max_processing_time = max(processing_times)

        return {
            'processing_performance': {
                'average_time_ms': avg_processing_time,
                'maximum_time_ms': max_processing_time,
                'realtime_capability': avg_processing_time < 50,
                'target_performance': avg_processing_time < 20
            },
            'data_flow': {
                'average_point_cloud_size': sum([m.point_cloud_size for m in self.slam_metrics]) / len(self.slam_metrics),
                'buffer_health': 'good'  # 简化评估
            },
            'status': 'excellent' if avg_processing_time < 20 else
                     'good' if avg_processing_time < 50 else 'needs_optimization'
        }

    def _analyze_network_performance(self) -> Dict:
        """分析网络性能"""
        if not self.network_metrics:
            return {'status': 'no_data'}

        latencies = [m.latency_ms for m in self.network_metrics if m.latency_ms < 999]

        if not latencies:
            return {'status': 'connection_issues'}

        avg_latency = sum(latencies) / len(latencies)

        return {
            'latency_analysis': {
                'average_ms': avg_latency,
                'maximum_ms': max(latencies),
                'minimum_ms': min(latencies)
            },
            'status': 'excellent' if avg_latency < 5 else
                     'good' if avg_latency < 10 else 'needs_optimization'
        }

    def _analyze_integration_quality(self) -> Dict:
        """分析系统集成质量"""
        # 基于数据可用性和一致性评估集成质量
        data_sources = {
            'system_monitoring': len(self.system_metrics) > 0,
            'slam_monitoring': len(self.slam_metrics) > 0,
            'network_monitoring': len(self.network_metrics) > 0
        }

        integration_score = sum(data_sources.values()) / len(data_sources) * 100

        return {
            'data_sources': data_sources,
            'integration_score': integration_score,
            'component_communication': 'functional' if integration_score > 60 else 'limited',
            'status': 'excellent' if integration_score > 80 else
                     'good' if integration_score > 60 else 'needs_improvement'
        }

    def _generate_recommendations(self) -> List[str]:
        """生成优化建议"""
        recommendations = []

        # 基于分析结果生成建议
        if self.system_metrics:
            avg_cpu = sum([m.cpu_percent for m in self.system_metrics]) / len(self.system_metrics)
            avg_memory = sum([m.memory_percent for m in self.system_metrics]) / len(self.system_metrics)

            if avg_cpu > 70:
                recommendations.append("CPU使用率较高，建议优化算法或减少可视化频率")
            if avg_memory > 60:
                recommendations.append("内存使用率较高，建议检查内存泄漏和缓冲区大小")

        if self.slam_metrics:
            processing_times = [m.fastlio_processing_time for m in self.slam_metrics if m.fastlio_processing_time > 0]
            if processing_times:
                avg_time = sum(processing_times) / len(processing_times)
                if avg_time > 30:
                    recommendations.append("SLAM处理时间超出目标，建议优化点云降采样和匹配算法")

        if not recommendations:
            recommendations.append("系统性能表现良好，建议继续监控长期稳定性")

        return recommendations

    def save_report(self, report: Dict, output_path: str):
        """保存分析报告"""
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"📄 分析报告已保存: {output_path}")

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='MID360 SLAM系统性能分析工具')
    parser.add_argument('--duration', '-d', type=int, default=60,
                       help='分析持续时间（秒），默认60秒')
    parser.add_argument('--output', '-o', type=str,
                       default='performance_analysis_report.json',
                       help='输出报告文件路径')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='详细输出模式')

    args = parser.parse_args()

    # 检查ROS2环境
    try:
        result = subprocess.run(['ros2', 'node', 'list'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            print("❌ ROS2环境未正确配置或SLAM系统未运行")
            print("请先启动SLAM系统: ./tools/slam_tools.sh start")
            sys.exit(1)
    except:
        print("❌ 无法访问ROS2环境")
        sys.exit(1)

    # 开始性能分析
    analyzer = PerformanceAnalyzer(analysis_duration=args.duration)

    try:
        report = analyzer.start_analysis()

        # 保存报告
        analyzer.save_report(report, args.output)

        # 显示摘要
        print("\n" + "="*60)
        print("📊 性能分析摘要")
        print("="*60)

        if 'system_performance' in report:
            sys_perf = report['system_performance']
            if 'status' in sys_perf:
                print(f"🖥️  系统性能: {sys_perf['status']}")

        if 'slam_performance' in report:
            slam_perf = report['slam_performance']
            if 'status' in slam_perf:
                print(f"🤖 SLAM性能: {slam_perf['status']}")

        if 'network_performance' in report:
            net_perf = report['network_performance']
            if 'status' in net_perf:
                print(f"📡 网络性能: {net_perf['status']}")

        if 'integration_quality' in report:
            int_quality = report['integration_quality']
            if 'status' in int_quality:
                print(f"🔗 集成质量: {int_quality['status']}")

        if 'recommendations' in report:
            print(f"\n💡 优化建议:")
            for i, rec in enumerate(report['recommendations'], 1):
                print(f"   {i}. {rec}")

        print(f"\n📄 详细报告: {args.output}")

    except KeyboardInterrupt:
        print("\n\n分析被用户中断")
    except Exception as e:
        print(f"❌ 分析过程中出现错误: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()