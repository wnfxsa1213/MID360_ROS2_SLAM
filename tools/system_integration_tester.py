#!/usr/bin/env python3
"""
MID360 SLAM系统集成测试工具
专门用于验证组件间通信、数据一致性和协同工作质量
"""

import os
import sys
import time
import json
import subprocess
import threading
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple, Any
import yaml

@dataclass
class ComponentStatus:
    """组件状态"""
    name: str
    is_running: bool
    pid: Optional[int]
    cpu_percent: float
    memory_mb: float
    response_time_ms: float

@dataclass
class TopicHealth:
    """话题健康状态"""
    topic_name: str
    msg_type: str
    frequency_hz: float
    subscribers: int
    publishers: int
    last_message_time: Optional[str]

@dataclass
class ServiceHealth:
    """服务健康状态"""
    service_name: str
    service_type: str
    is_available: bool
    response_time_ms: float

class SystemIntegrationTester:
    """系统集成测试器"""

    def __init__(self):
        self.test_results = {}
        self.component_statuses = []
        self.topic_healths = []
        self.service_healths = []

        # 定义期望的系统组件
        self.expected_components = {
            'lio_node': {
                'description': 'FASTLIO2核心算法',
                'critical': True,
                'topics_published': ['/fastlio2/lio_odom', '/fastlio2/lio_path'],
                'topics_subscribed': ['/livox/lidar', '/livox/imu'],
                'services_provided': ['fastlio2/save_maps', 'fastlio2/update_pose']
            },
            'livox_ros_driver2_node': {
                'description': 'Livox MID360驱动',
                'critical': True,
                'topics_published': ['/livox/lidar', '/livox/imu'],
                'topics_subscribed': [],
                'services_provided': []
            },
            'optimization_coordinator': {
                'description': '优化协调器',
                'critical': False,
                'topics_published': ['/coordinator/metrics'],
                'topics_subscribed': ['/fastlio2/performance_metrics'],
                'services_provided': ['coordinator/trigger_optimization']
            },
            'pgo_node': {
                'description': '位姿图优化',
                'critical': False,
                'topics_published': [],
                'topics_subscribed': ['/fastlio2/lio_odom'],
                'services_provided': ['pgo/optimize']
            },
            'hba_node': {
                'description': '分层束调整',
                'critical': False,
                'topics_published': [],
                'topics_subscribed': [],
                'services_provided': ['hba/refine_map']
            },
            'localizer_node': {
                'description': '定位器',
                'critical': False,
                'topics_published': [],
                'topics_subscribed': [],
                'services_provided': ['localizer/relocalize']
            }
        }

        # 关键数据流路径
        self.critical_data_flows = [
            {
                'name': '传感器数据流',
                'path': '/livox/lidar -> lio_node -> /fastlio2/lio_odom',
                'components': ['livox_ros_driver2_node', 'lio_node'],
                'min_frequency': 8.0
            },
            {
                'name': 'IMU数据流',
                'path': '/livox/imu -> lio_node',
                'components': ['livox_ros_driver2_node', 'lio_node'],
                'min_frequency': 800.0
            },
            {
                'name': '协同优化流',
                'path': 'lio_node -> coordinator -> pgo_node/hba_node',
                'components': ['lio_node', 'optimization_coordinator'],
                'min_frequency': 0.1
            }
        ]

    def run_comprehensive_test(self) -> Dict:
        """运行综合集成测试"""
        print("🔧 开始MID360 SLAM系统集成测试")
        print("="*60)

        test_results = {
            'test_info': {
                'start_time': datetime.now().isoformat(),
                'test_version': '1.0',
                'system_type': 'MID360_SLAM'
            },
            'component_tests': {},
            'communication_tests': {},
            'data_flow_tests': {},
            'performance_tests': {},
            'integration_score': 0.0,
            'recommendations': []
        }

        try:
            # 1. 组件存在性和运行状态测试
            print("🔍 1. 检查组件运行状态...")
            test_results['component_tests'] = self._test_component_status()

            # 2. 通信机制测试
            print("📡 2. 测试组件间通信...")
            test_results['communication_tests'] = self._test_communication()

            # 3. 数据流完整性测试
            print("🔄 3. 验证数据流完整性...")
            test_results['data_flow_tests'] = self._test_data_flows()

            # 4. 性能集成测试
            print("⚡ 4. 性能集成测试...")
            test_results['performance_tests'] = self._test_performance_integration()

            # 5. 计算集成评分
            test_results['integration_score'] = self._calculate_integration_score(test_results)

            # 6. 生成改进建议
            test_results['recommendations'] = self._generate_integration_recommendations(test_results)

            test_results['test_info']['end_time'] = datetime.now().isoformat()
            test_results['test_info']['status'] = 'completed'

        except Exception as e:
            test_results['test_info']['status'] = 'failed'
            test_results['test_info']['error'] = str(e)
            print(f"❌ 测试过程中出现错误: {e}")

        return test_results

    def _test_component_status(self) -> Dict:
        """测试组件运行状态"""
        results = {}

        # 获取当前运行的ROS2节点
        try:
            result = subprocess.run(['ros2', 'node', 'list'],
                                  capture_output=True, text=True, timeout=10)
            running_nodes = result.stdout.strip().split('\n') if result.returncode == 0 else []
        except:
            running_nodes = []

        for component_name, component_info in self.expected_components.items():
            print(f"   检查 {component_info['description']}...")

            is_running = any(component_name in node for node in running_nodes)

            component_result = {
                'name': component_name,
                'description': component_info['description'],
                'critical': component_info['critical'],
                'is_running': is_running,
                'status': 'running' if is_running else 'stopped',
                'topics_check': self._check_component_topics(component_info, running_nodes),
                'services_check': self._check_component_services(component_info)
            }

            results[component_name] = component_result

            status_icon = "✅" if is_running else "❌" if component_info['critical'] else "⚠️"
            print(f"     {status_icon} {component_info['description']}: {'运行中' if is_running else '未运行'}")

        return results

    def _check_component_topics(self, component_info: Dict, running_nodes: List[str]) -> Dict:
        """检查组件的话题发布和订阅"""
        topics_check = {
            'published_topics': {},
            'subscribed_topics': {}
        }

        try:
            # 获取所有话题列表
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)
            available_topics = result.stdout.strip().split('\n') if result.returncode == 0 else []

            # 检查发布的话题
            for topic in component_info['topics_published']:
                is_available = topic in available_topics
                topics_check['published_topics'][topic] = {
                    'available': is_available,
                    'frequency': self._get_topic_frequency(topic) if is_available else 0.0
                }

            # 检查订阅的话题
            for topic in component_info['topics_subscribed']:
                is_available = topic in available_topics
                topics_check['subscribed_topics'][topic] = {
                    'available': is_available,
                    'frequency': self._get_topic_frequency(topic) if is_available else 0.0
                }

        except:
            pass

        return topics_check

    def _check_component_services(self, component_info: Dict) -> Dict:
        """检查组件提供的服务"""
        services_check = {}

        try:
            # 获取所有服务列表
            result = subprocess.run(['ros2', 'service', 'list'],
                                  capture_output=True, text=True, timeout=5)
            available_services = result.stdout.strip().split('\n') if result.returncode == 0 else []

            for service in component_info['services_provided']:
                is_available = any(service in svc for svc in available_services)
                services_check[service] = {
                    'available': is_available,
                    'response_time': self._test_service_response(service) if is_available else -1
                }

        except:
            pass

        return services_check

    def _get_topic_frequency(self, topic_name: str) -> float:
        """获取话题发布频率"""
        try:
            result = subprocess.run(['timeout', '3s', 'ros2', 'topic', 'hz', topic_name],
                                  capture_output=True, text=True)

            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'average rate:' in line:
                        freq_str = line.split('average rate:')[1].strip()
                        return float(freq_str.split()[0])
            return 0.0
        except:
            return 0.0

    def _test_service_response(self, service_name: str) -> float:
        """测试服务响应时间"""
        try:
            start_time = time.time()
            result = subprocess.run(['ros2', 'service', 'type', service_name],
                                  capture_output=True, text=True, timeout=2)
            end_time = time.time()

            if result.returncode == 0:
                return (end_time - start_time) * 1000  # 转换为毫秒
            return -1
        except:
            return -1

    def _test_communication(self) -> Dict:
        """测试组件间通信"""
        communication_results = {
            'topic_communication': {},
            'service_communication': {},
            'transform_communication': {}
        }

        # 测试关键话题通信
        key_topics = ['/livox/lidar', '/livox/imu', '/fastlio2/lio_odom', '/fastlio2/lio_path']

        for topic in key_topics:
            print(f"   测试话题: {topic}")

            freq = self._get_topic_frequency(topic)

            # 尝试获取一个消息来验证数据流
            data_available = self._test_topic_data_availability(topic)

            communication_results['topic_communication'][topic] = {
                'frequency_hz': freq,
                'data_available': data_available,
                'status': 'healthy' if freq > 0 and data_available else 'unhealthy'
            }

        # 测试TF变换通信
        tf_status = self._test_tf_communication()
        communication_results['transform_communication'] = tf_status

        return communication_results

    def _test_topic_data_availability(self, topic_name: str) -> bool:
        """测试话题数据可用性"""
        try:
            result = subprocess.run(['timeout', '2s', 'ros2', 'topic', 'echo', topic_name, '--once'],
                                  capture_output=True, text=True)
            return result.returncode == 0 and len(result.stdout.strip()) > 0
        except:
            return False

    def _test_tf_communication(self) -> Dict:
        """测试TF变换通信"""
        tf_results = {
            'frames_available': [],
            'critical_transforms': {}
        }

        try:
            # 检查可用的TF框架
            result = subprocess.run(['timeout', '2s', 'ros2', 'run', 'tf2_tools', 'view_frames'],
                                  capture_output=True, text=True)

            # 检查关键变换
            critical_transforms = ['odom -> base_link', 'base_link -> livox_frame']

            for transform in critical_transforms:
                try:
                    frames = transform.split(' -> ')
                    result = subprocess.run(['timeout', '1s', 'ros2', 'run', 'tf2_ros', 'tf2_echo',
                                           frames[0], frames[1]],
                                          capture_output=True, text=True)
                    tf_results['critical_transforms'][transform] = {
                        'available': result.returncode == 0,
                        'data': result.stdout[:200] if result.returncode == 0 else None
                    }
                except:
                    tf_results['critical_transforms'][transform] = {
                        'available': False,
                        'data': None
                    }

        except:
            pass

        return tf_results

    def _test_data_flows(self) -> Dict:
        """测试数据流完整性"""
        data_flow_results = {}

        for flow in self.critical_data_flows:
            print(f"   验证数据流: {flow['name']}")

            flow_result = {
                'name': flow['name'],
                'path': flow['path'],
                'components_status': {},
                'frequency_check': {},
                'latency_ms': 0.0,
                'integrity_score': 0.0
            }

            # 检查流程中的组件状态
            for component in flow['components']:
                component_ok = component in [node.split('/')[-1] for node in self._get_running_nodes()]
                flow_result['components_status'][component] = component_ok

            # 检查数据流频率（简化版）
            if 'livox/lidar' in flow['path']:
                freq = self._get_topic_frequency('/livox/lidar')
                flow_result['frequency_check']['lidar'] = {
                    'measured_hz': freq,
                    'min_required_hz': flow.get('min_frequency', 1.0),
                    'meets_requirement': freq >= flow.get('min_frequency', 1.0)
                }

            if 'livox/imu' in flow['path']:
                freq = self._get_topic_frequency('/livox/imu')
                flow_result['frequency_check']['imu'] = {
                    'measured_hz': freq,
                    'min_required_hz': flow.get('min_frequency', 1.0),
                    'meets_requirement': freq >= flow.get('min_frequency', 1.0)
                }

            # 计算完整性评分
            component_score = sum(flow_result['components_status'].values()) / len(flow_result['components_status'])
            frequency_score = 1.0
            if flow_result['frequency_check']:
                frequency_score = sum([f['meets_requirement'] for f in flow_result['frequency_check'].values()]) / len(flow_result['frequency_check'])

            flow_result['integrity_score'] = (component_score + frequency_score) / 2.0

            data_flow_results[flow['name']] = flow_result

        return data_flow_results

    def _test_performance_integration(self) -> Dict:
        """测试性能集成"""
        performance_results = {
            'end_to_end_latency': {},
            'resource_efficiency': {},
            'scalability': {}
        }

        # 简化的端到端延迟测试
        try:
            # 测试传感器到SLAM输出的延迟
            start_time = time.time()

            # 获取一个激光雷达消息
            lidar_result = subprocess.run(['timeout', '1s', 'ros2', 'topic', 'echo',
                                         '/livox/lidar', '--once'],
                                        capture_output=True, text=True)

            # 获取一个里程计输出
            odom_result = subprocess.run(['timeout', '1s', 'ros2', 'topic', 'echo',
                                        '/fastlio2/lio_odom', '--once'],
                                       capture_output=True, text=True)

            end_time = time.time()

            performance_results['end_to_end_latency'] = {
                'sensor_to_odom_ms': (end_time - start_time) * 1000,
                'data_available': lidar_result.returncode == 0 and odom_result.returncode == 0
            }

        except:
            performance_results['end_to_end_latency'] = {
                'sensor_to_odom_ms': -1,
                'data_available': False
            }

        return performance_results

    def _get_running_nodes(self) -> List[str]:
        """获取运行中的节点列表"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'],
                                  capture_output=True, text=True, timeout=5)
            return result.stdout.strip().split('\n') if result.returncode == 0 else []
        except:
            return []

    def _calculate_integration_score(self, test_results: Dict) -> float:
        """计算系统集成评分"""
        score = 0.0
        max_score = 100.0

        # 组件运行状态评分 (40分)
        if 'component_tests' in test_results:
            running_critical = sum(1 for comp in test_results['component_tests'].values()
                                 if comp['critical'] and comp['is_running'])
            total_critical = sum(1 for comp in test_results['component_tests'].values()
                               if comp['critical'])

            if total_critical > 0:
                score += (running_critical / total_critical) * 40

        # 通信质量评分 (30分)
        if 'communication_tests' in test_results and 'topic_communication' in test_results['communication_tests']:
            healthy_topics = sum(1 for topic in test_results['communication_tests']['topic_communication'].values()
                               if topic['status'] == 'healthy')
            total_topics = len(test_results['communication_tests']['topic_communication'])

            if total_topics > 0:
                score += (healthy_topics / total_topics) * 30

        # 数据流完整性评分 (20分)
        if 'data_flow_tests' in test_results:
            total_integrity = sum(flow['integrity_score'] for flow in test_results['data_flow_tests'].values())
            flow_count = len(test_results['data_flow_tests'])

            if flow_count > 0:
                score += (total_integrity / flow_count) * 20

        # 性能集成评分 (10分)
        if 'performance_tests' in test_results:
            if test_results['performance_tests']['end_to_end_latency']['data_available']:
                score += 10

        return min(score, max_score)

    def _generate_integration_recommendations(self, test_results: Dict) -> List[str]:
        """生成集成改进建议"""
        recommendations = []

        # 基于组件状态生成建议
        if 'component_tests' in test_results:
            for comp_name, comp_data in test_results['component_tests'].items():
                if comp_data['critical'] and not comp_data['is_running']:
                    recommendations.append(f"关键组件 {comp_data['description']} 未运行，请检查启动配置")

        # 基于通信状态生成建议
        if 'communication_tests' in test_results:
            topic_comm = test_results['communication_tests'].get('topic_communication', {})
            for topic, status in topic_comm.items():
                if status['status'] == 'unhealthy':
                    recommendations.append(f"话题 {topic} 通信异常，频率: {status['frequency_hz']:.1f}Hz")

        # 基于数据流状态生成建议
        if 'data_flow_tests' in test_results:
            for flow_name, flow_data in test_results['data_flow_tests'].items():
                if flow_data['integrity_score'] < 0.8:
                    recommendations.append(f"数据流 '{flow_name}' 完整性不足 ({flow_data['integrity_score']:.1%})")

        if not recommendations:
            recommendations.append("系统集成状态良好，建议进行长期稳定性测试")

        return recommendations

    def save_test_results(self, results: Dict, output_path: str):
        """保存测试结果"""
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"📄 集成测试报告已保存: {output_path}")

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='MID360 SLAM系统集成测试工具')
    parser.add_argument('--output', '-o', type=str,
                       default='system_integration_test_report.json',
                       help='输出测试报告文件路径')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='详细输出模式')

    args = parser.parse_args()

    # 检查ROS2环境
    try:
        result = subprocess.run(['ros2', 'node', 'list'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            print("❌ ROS2环境未正确配置")
            print("请确保已加载ROS2环境: source /opt/ros/humble/setup.bash")
            sys.exit(1)
    except:
        print("❌ 无法访问ROS2环境")
        sys.exit(1)

    # 运行系统集成测试
    tester = SystemIntegrationTester()

    try:
        results = tester.run_comprehensive_test()

        # 保存测试结果
        tester.save_test_results(results, args.output)

        # 显示测试摘要
        print("\n" + "="*60)
        print("🔧 系统集成测试摘要")
        print("="*60)

        print(f"🔢 集成评分: {results['integration_score']:.1f}/100")

        if results['integration_score'] >= 80:
            print("✅ 系统集成质量: 优秀")
        elif results['integration_score'] >= 60:
            print("⚠️  系统集成质量: 良好")
        else:
            print("❌ 系统集成质量: 需要改进")

        # 显示关键组件状态
        if 'component_tests' in results:
            print(f"\n🔍 组件状态:")
            for comp_name, comp_data in results['component_tests'].items():
                if comp_data['critical']:
                    status_icon = "✅" if comp_data['is_running'] else "❌"
                    print(f"   {status_icon} {comp_data['description']}")

        # 显示改进建议
        if 'recommendations' in results and results['recommendations']:
            print(f"\n💡 改进建议:")
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