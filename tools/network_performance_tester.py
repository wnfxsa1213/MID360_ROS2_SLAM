#!/usr/bin/env python3
"""
MID360 SLAM系统网络性能测试工具
专门用于验证光纤传输性能和UDP通信效率
"""

import os
import sys
import time
import json
import subprocess
import threading
import psutil
import socket
import struct
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
import statistics

@dataclass
class NetworkTestResult:
    """网络测试结果"""
    test_name: str
    target_ip: str
    port: int
    latency_ms: float
    jitter_ms: float
    packet_loss_percent: float
    bandwidth_mbps: float
    status: str

@dataclass
class UdpPortTest:
    """UDP端口测试结果"""
    port: int
    description: str
    is_listening: bool
    response_time_ms: float
    packet_rate_hz: float

class NetworkPerformanceTester:
    """网络性能测试器"""

    def __init__(self):
        # MID360网络配置
        self.lidar_ip = "192.168.1.3"
        self.host_ip = "192.168.1.50"
        self.network_interface = "enp7s0"  # 从配置文件获取

        # 关键端口配置
        self.critical_ports = {
            56100: "命令数据端口",
            56200: "推送消息端口",
            56300: "点云数据端口",
            56400: "IMU数据端口",
            56500: "日志数据端口"
        }

        # 性能阈值
        self.performance_thresholds = {
            'max_latency_ms': 5.0,           # 最大延迟5ms (光纤)
            'max_jitter_ms': 2.0,            # 最大抖动2ms
            'max_packet_loss': 0.1,          # 最大丢包率0.1%
            'min_bandwidth_mbps': 100.0,     # 最小带宽100Mbps
            'min_point_cloud_rate': 8.0,     # 最小点云频率8Hz
            'min_imu_rate': 800.0            # 最小IMU频率800Hz
        }

    def run_comprehensive_network_test(self) -> Dict:
        """运行综合网络性能测试"""
        print("🌐 开始MID360 SLAM网络性能测试")
        print("="*60)

        test_results = {
            'test_info': {
                'start_time': datetime.now().isoformat(),
                'lidar_ip': self.lidar_ip,
                'host_ip': self.host_ip,
                'interface': self.network_interface
            },
            'basic_connectivity': {},
            'udp_port_tests': {},
            'latency_analysis': {},
            'bandwidth_tests': {},
            'data_flow_rates': {},
            'network_stability': {},
            'performance_score': 0.0,
            'recommendations': []
        }

        try:
            # 1. 基础连通性测试
            print("🔍 1. 基础网络连通性测试...")
            test_results['basic_connectivity'] = self._test_basic_connectivity()

            # 2. UDP端口测试
            print("📡 2. UDP端口功能测试...")
            test_results['udp_port_tests'] = self._test_udp_ports()

            # 3. 延迟和抖动分析
            print("⏱️  3. 网络延迟和抖动分析...")
            test_results['latency_analysis'] = self._test_network_latency()

            # 4. 带宽测试
            print("📊 4. 网络带宽测试...")
            test_results['bandwidth_tests'] = self._test_network_bandwidth()

            # 5. 数据流速率测试
            print("🔄 5. SLAM数据流速率测试...")
            test_results['data_flow_rates'] = self._test_slam_data_rates()

            # 6. 网络稳定性测试
            print("🎯 6. 网络稳定性测试...")
            test_results['network_stability'] = self._test_network_stability()

            # 7. 计算综合性能评分
            test_results['performance_score'] = self._calculate_network_score(test_results)

            # 8. 生成优化建议
            test_results['recommendations'] = self._generate_network_recommendations(test_results)

            test_results['test_info']['end_time'] = datetime.now().isoformat()
            test_results['test_info']['status'] = 'completed'

        except Exception as e:
            test_results['test_info']['status'] = 'failed'
            test_results['test_info']['error'] = str(e)
            print(f"❌ 网络测试过程中出现错误: {e}")

        return test_results

    def _test_basic_connectivity(self) -> Dict:
        """测试基础网络连通性"""
        results = {
            'ping_test': {},
            'interface_status': {},
            'route_check': {}
        }

        # Ping测试
        try:
            print(f"   测试到雷达({self.lidar_ip})的连通性...")
            ping_result = subprocess.run([
                'ping', '-c', '10', '-i', '0.1', self.lidar_ip
            ], capture_output=True, text=True, timeout=15)

            if ping_result.returncode == 0:
                # 解析ping结果
                output_lines = ping_result.stdout.split('\n')
                stats_line = [line for line in output_lines if 'packet loss' in line]
                rtt_line = [line for line in output_lines if 'rtt' in line]

                packet_loss = 0.0
                avg_rtt = 0.0

                if stats_line:
                    loss_str = stats_line[0].split(',')[2].strip()
                    packet_loss = float(loss_str.split('%')[0])

                if rtt_line:
                    rtt_parts = rtt_line[0].split('=')[1].strip().split('/')
                    avg_rtt = float(rtt_parts[1])

                results['ping_test'] = {
                    'reachable': True,
                    'packet_loss_percent': packet_loss,
                    'average_rtt_ms': avg_rtt,
                    'status': 'excellent' if avg_rtt < 2 and packet_loss == 0 else
                             'good' if avg_rtt < 5 and packet_loss < 1 else 'poor'
                }
            else:
                results['ping_test'] = {
                    'reachable': False,
                    'error': ping_result.stderr
                }

        except Exception as e:
            results['ping_test'] = {'reachable': False, 'error': str(e)}

        # 网络接口状态
        try:
            interfaces = psutil.net_if_stats()
            if self.network_interface in interfaces:
                iface_stats = interfaces[self.network_interface]
                results['interface_status'] = {
                    'is_up': iface_stats.isup,
                    'speed_mbps': iface_stats.speed if iface_stats.speed > 0 else 'unknown',
                    'mtu': iface_stats.mtu
                }
            else:
                results['interface_status'] = {'error': f'Interface {self.network_interface} not found'}

        except Exception as e:
            results['interface_status'] = {'error': str(e)}

        return results

    def _test_udp_ports(self) -> Dict:
        """测试UDP端口功能"""
        port_results = {}

        for port, description in self.critical_ports.items():
            print(f"   测试端口 {port} ({description})...")

            port_result = {
                'port': port,
                'description': description,
                'host_listening': self._is_port_listening(port),
                'lidar_responsive': self._test_lidar_port_response(port),
                'packet_capture': self._capture_port_traffic(port, duration=2)
            }

            port_results[str(port)] = port_result

        return port_results

    def _is_port_listening(self, port: int) -> bool:
        """检查本地端口是否监听"""
        try:
            connections = psutil.net_connections(kind='udp')
            return any(conn.laddr.port == port for conn in connections if conn.laddr)
        except:
            return False

    def _test_lidar_port_response(self, port: int) -> Dict:
        """测试雷达端口响应"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)

            # 发送简单的测试包
            test_data = b'test_packet'
            start_time = time.time()

            try:
                sock.sendto(test_data, (self.lidar_ip, port))
                # 尝试接收响应（可能不会有响应，这是正常的）
                try:
                    data, addr = sock.recvfrom(1024)
                    response_time = (time.time() - start_time) * 1000
                    return {'responsive': True, 'response_time_ms': response_time}
                except socket.timeout:
                    # 超时是正常的，雷达可能不响应测试包
                    return {'responsive': True, 'response_time_ms': -1, 'note': 'no_response_expected'}
            except Exception as e:
                return {'responsive': False, 'error': str(e)}
            finally:
                sock.close()

        except Exception as e:
            return {'responsive': False, 'error': str(e)}

    def _capture_port_traffic(self, port: int, duration: int = 2) -> Dict:
        """捕获端口流量"""
        try:
            # 使用netstat检查端口流量（简化版）
            result = subprocess.run([
                'timeout', f'{duration}s', 'netstat', '-u', '-n'
            ], capture_output=True, text=True)

            if result.returncode == 0:
                lines = result.stdout.split('\n')
                port_lines = [line for line in lines if f':{port}' in line]
                return {
                    'traffic_detected': len(port_lines) > 0,
                    'connections_count': len(port_lines)
                }
            else:
                return {'traffic_detected': False, 'error': 'netstat_failed'}

        except Exception as e:
            return {'traffic_detected': False, 'error': str(e)}

    def _test_network_latency(self) -> Dict:
        """测试网络延迟和抖动"""
        results = {
            'continuous_ping': {},
            'burst_test': {},
            'jitter_analysis': {}
        }

        try:
            print("   执行持续ping测试...")
            # 持续ping测试 (100个包)
            ping_result = subprocess.run([
                'ping', '-c', '100', '-i', '0.01', self.lidar_ip
            ], capture_output=True, text=True, timeout=10)

            if ping_result.returncode == 0:
                # 提取所有RTT值
                rtt_values = []
                for line in ping_result.stdout.split('\n'):
                    if 'time=' in line:
                        rtt_str = line.split('time=')[1].split()[0]
                        try:
                            rtt_values.append(float(rtt_str))
                        except:
                            pass

                if rtt_values:
                    results['continuous_ping'] = {
                        'sample_count': len(rtt_values),
                        'min_rtt_ms': min(rtt_values),
                        'max_rtt_ms': max(rtt_values),
                        'avg_rtt_ms': statistics.mean(rtt_values),
                        'median_rtt_ms': statistics.median(rtt_values),
                        'std_dev_ms': statistics.stdev(rtt_values) if len(rtt_values) > 1 else 0
                    }

                    # 抖动分析
                    jitter_values = []
                    for i in range(1, len(rtt_values)):
                        jitter_values.append(abs(rtt_values[i] - rtt_values[i-1]))

                    if jitter_values:
                        results['jitter_analysis'] = {
                            'avg_jitter_ms': statistics.mean(jitter_values),
                            'max_jitter_ms': max(jitter_values),
                            'jitter_std_dev': statistics.stdev(jitter_values) if len(jitter_values) > 1 else 0
                        }

        except Exception as e:
            results['continuous_ping']['error'] = str(e)

        return results

    def _test_network_bandwidth(self) -> Dict:
        """测试网络带宽"""
        results = {
            'interface_stats': {},
            'estimated_capacity': {}
        }

        try:
            # 获取网络接口统计
            net_io_start = psutil.net_io_counters(pernic=True)
            if self.network_interface in net_io_start:
                start_stats = net_io_start[self.network_interface]
                start_time = time.time()

                # 等待1秒收集数据
                time.sleep(1)

                net_io_end = psutil.net_io_counters(pernic=True)
                end_stats = net_io_end[self.network_interface]
                end_time = time.time()

                duration = end_time - start_time
                bytes_sent = end_stats.bytes_sent - start_stats.bytes_sent
                bytes_recv = end_stats.bytes_recv - start_stats.bytes_recv

                results['interface_stats'] = {
                    'send_rate_mbps': (bytes_sent * 8) / (duration * 1000000),
                    'recv_rate_mbps': (bytes_recv * 8) / (duration * 1000000),
                    'total_rate_mbps': ((bytes_sent + bytes_recv) * 8) / (duration * 1000000),
                    'packets_sent_rate': (end_stats.packets_sent - start_stats.packets_sent) / duration,
                    'packets_recv_rate': (end_stats.packets_recv - start_stats.packets_recv) / duration
                }

        except Exception as e:
            results['interface_stats']['error'] = str(e)

        return results

    def _test_slam_data_rates(self) -> Dict:
        """测试SLAM数据流速率"""
        results = {
            'ros_topics': {},
            'data_consistency': {}
        }

        # 关键SLAM话题
        slam_topics = {
            '/livox/lidar': {'expected_min_hz': 8, 'expected_max_hz': 15},
            '/livox/imu': {'expected_min_hz': 800, 'expected_max_hz': 1200},
            '/fastlio2/lio_odom': {'expected_min_hz': 8, 'expected_max_hz': 15},
            '/fastlio2/lio_path': {'expected_min_hz': 1, 'expected_max_hz': 15}
        }

        for topic, expectations in slam_topics.items():
            print(f"   测试话题频率: {topic}")

            try:
                # 使用ros2 topic hz测试频率
                hz_result = subprocess.run([
                    'timeout', '5s', 'ros2', 'topic', 'hz', topic
                ], capture_output=True, text=True)

                if hz_result.returncode == 0:
                    # 解析频率结果
                    for line in hz_result.stdout.split('\n'):
                        if 'average rate:' in line:
                            freq_str = line.split('average rate:')[1].strip()
                            try:
                                frequency = float(freq_str.split()[0])
                                meets_expectations = (expectations['expected_min_hz'] <=
                                                    frequency <= expectations['expected_max_hz'])

                                results['ros_topics'][topic] = {
                                    'frequency_hz': frequency,
                                    'expected_range': f"{expectations['expected_min_hz']}-{expectations['expected_max_hz']}",
                                    'meets_expectations': meets_expectations,
                                    'status': 'good' if meets_expectations else 'poor'
                                }
                                break
                            except:
                                pass

                if topic not in results['ros_topics']:
                    results['ros_topics'][topic] = {
                        'frequency_hz': 0,
                        'status': 'no_data',
                        'error': 'no_frequency_data'
                    }

            except Exception as e:
                results['ros_topics'][topic] = {
                    'frequency_hz': 0,
                    'status': 'error',
                    'error': str(e)
                }

        return results

    def _test_network_stability(self) -> Dict:
        """测试网络稳定性"""
        results = {
            'sustained_ping': {},
            'error_rates': {}
        }

        try:
            print("   执行5分钟稳定性测试...")
            # 较长时间的ping测试
            ping_result = subprocess.run([
                'ping', '-c', '300', '-i', '1', self.lidar_ip
            ], capture_output=True, text=True, timeout=350)

            if ping_result.returncode == 0:
                output_lines = ping_result.stdout.split('\n')

                # 提取统计信息
                stats_line = [line for line in output_lines if 'packet loss' in line]
                if stats_line:
                    loss_str = stats_line[0].split(',')[2].strip()
                    packet_loss = float(loss_str.split('%')[0])

                    results['sustained_ping'] = {
                        'duration_minutes': 5,
                        'packet_loss_percent': packet_loss,
                        'stability': 'excellent' if packet_loss == 0 else
                                   'good' if packet_loss < 1 else 'poor'
                    }

        except subprocess.TimeoutExpired:
            results['sustained_ping'] = {
                'status': 'timeout',
                'note': 'Test took too long, network may be unstable'
            }
        except Exception as e:
            results['sustained_ping'] = {'error': str(e)}

        return results

    def _calculate_network_score(self, test_results: Dict) -> float:
        """计算网络性能综合评分"""
        score = 0.0

        # 连通性评分 (30分)
        if 'basic_connectivity' in test_results and 'ping_test' in test_results['basic_connectivity']:
            ping_test = test_results['basic_connectivity']['ping_test']
            if ping_test.get('reachable', False):
                avg_rtt = ping_test.get('average_rtt_ms', 999)
                packet_loss = ping_test.get('packet_loss_percent', 100)

                if avg_rtt < 2 and packet_loss == 0:
                    score += 30
                elif avg_rtt < 5 and packet_loss < 1:
                    score += 20
                elif avg_rtt < 10 and packet_loss < 5:
                    score += 10

        # 延迟和抖动评分 (25分)
        if 'latency_analysis' in test_results and 'continuous_ping' in test_results['latency_analysis']:
            latency = test_results['latency_analysis']['continuous_ping']
            avg_rtt = latency.get('avg_rtt_ms', 999)
            std_dev = latency.get('std_dev_ms', 999)

            if avg_rtt < 2 and std_dev < 0.5:
                score += 25
            elif avg_rtt < 5 and std_dev < 1:
                score += 20
            elif avg_rtt < 10 and std_dev < 2:
                score += 10

        # 数据流速率评分 (25分)
        if 'data_flow_rates' in test_results and 'ros_topics' in test_results['data_flow_rates']:
            topics = test_results['data_flow_rates']['ros_topics']
            good_topics = sum(1 for topic in topics.values() if topic.get('status') == 'good')
            total_topics = len(topics)

            if total_topics > 0:
                score += (good_topics / total_topics) * 25

        # 稳定性评分 (20分)
        if 'network_stability' in test_results and 'sustained_ping' in test_results['network_stability']:
            stability = test_results['network_stability']['sustained_ping']
            packet_loss = stability.get('packet_loss_percent', 100)

            if packet_loss == 0:
                score += 20
            elif packet_loss < 1:
                score += 15
            elif packet_loss < 5:
                score += 10

        return min(score, 100.0)

    def _generate_network_recommendations(self, test_results: Dict) -> List[str]:
        """生成网络优化建议"""
        recommendations = []

        # 基于连通性测试
        if 'basic_connectivity' in test_results:
            ping_test = test_results['basic_connectivity'].get('ping_test', {})
            if not ping_test.get('reachable', False):
                recommendations.append("雷达不可达，请检查网络连接和IP配置")
            elif ping_test.get('average_rtt_ms', 0) > 5:
                recommendations.append("网络延迟较高，建议检查网络硬件和配置")

        # 基于延迟分析
        if 'latency_analysis' in test_results:
            latency = test_results['latency_analysis'].get('continuous_ping', {})
            if latency.get('std_dev_ms', 0) > 2:
                recommendations.append("网络抖动较大，建议使用专用网络接口")

        # 基于数据流测试
        if 'data_flow_rates' in test_results:
            topics = test_results['data_flow_rates'].get('ros_topics', {})
            for topic, data in topics.items():
                if data.get('status') == 'poor':
                    recommendations.append(f"话题 {topic} 频率异常，请检查SLAM系统运行状态")

        # 基于稳定性测试
        if 'network_stability' in test_results:
            stability = test_results['network_stability'].get('sustained_ping', {})
            if stability.get('packet_loss_percent', 0) > 1:
                recommendations.append("网络存在丢包，建议检查网络硬件质量")

        if not recommendations:
            recommendations.append("网络性能表现优秀，建议继续监控在高负载情况下的表现")

        return recommendations

    def save_test_results(self, results: Dict, output_path: str):
        """保存测试结果"""
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"📄 网络性能测试报告已保存: {output_path}")

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='MID360 SLAM网络性能测试工具')
    parser.add_argument('--output', '-o', type=str,
                       default='network_performance_test_report.json',
                       help='输出测试报告文件路径')
    parser.add_argument('--quick', '-q', action='store_true',
                       help='快速测试模式（跳过长时间稳定性测试）')

    args = parser.parse_args()

    # 运行网络性能测试
    tester = NetworkPerformanceTester()

    try:
        results = tester.run_comprehensive_network_test()

        # 保存测试结果
        tester.save_test_results(results, args.output)

        # 显示测试摘要
        print("\n" + "="*60)
        print("🌐 网络性能测试摘要")
        print("="*60)

        print(f"🔢 网络性能评分: {results['performance_score']:.1f}/100")

        if results['performance_score'] >= 80:
            print("✅ 网络性能: 优秀")
        elif results['performance_score'] >= 60:
            print("⚠️  网络性能: 良好")
        else:
            print("❌ 网络性能: 需要优化")

        # 显示关键指标
        if 'basic_connectivity' in results and 'ping_test' in results['basic_connectivity']:
            ping = results['basic_connectivity']['ping_test']
            if ping.get('reachable'):
                print(f"📡 平均延迟: {ping.get('average_rtt_ms', 0):.1f}ms")
                print(f"📊 丢包率: {ping.get('packet_loss_percent', 0):.1f}%")

        # 显示数据流状态
        if 'data_flow_rates' in results and 'ros_topics' in results['data_flow_rates']:
            topics = results['data_flow_rates']['ros_topics']
            good_topics = sum(1 for t in topics.values() if t.get('status') == 'good')
            total_topics = len(topics)
            print(f"🔄 数据流状态: {good_topics}/{total_topics} 正常")

        # 显示优化建议
        if 'recommendations' in results and results['recommendations']:
            print(f"\n💡 网络优化建议:")
            for i, rec in enumerate(results['recommendations'], 1):
                print(f"   {i}. {rec}")

        print(f"\n📄 详细报告: {args.output}")

    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    except Exception as e:
        print(f"❌ 测试过程中出现错误: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()