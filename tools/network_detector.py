#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
网络接口自动检测工具
为配置生成器提供自动网络接口检测功能
"""

import os
import sys
import socket
import subprocess
from typing import Dict, List, Optional

class NetworkDetector:
    """网络接口自动检测器"""

    def __init__(self):
        self.interfaces = {}
        self.active_interfaces = []

    def get_network_interfaces(self) -> Dict[str, Dict]:
        """获取所有网络接口信息"""
        try:
            # 在Linux系统上使用ip命令获取网络接口信息
            result = subprocess.run(['ip', 'addr', 'show'],
                                    capture_output=True, text=True, check=True)

            interfaces = {}
            current_interface = None

            for line in result.stdout.split('\n'):
                line = line.strip()

                # 解析接口名称
                if line and line[0].isdigit() and ':' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        interface_name = parts[1].strip()
                        # 过滤掉状态标志
                        interface_name = interface_name.split('@')[0]
                        current_interface = interface_name
                        interfaces[current_interface] = {
                            'name': current_interface,
                            'ips': [],
                            'status': 'DOWN',
                            'type': 'unknown'
                        }

                        # 检查接口状态
                        if 'UP' in line:
                            interfaces[current_interface]['status'] = 'UP'

                # 解析IP地址
                elif current_interface and 'inet ' in line:
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == 'inet' and i + 1 < len(parts):
                            ip_with_mask = parts[i + 1]
                            ip = ip_with_mask.split('/')[0]
                            if ip and not ip.startswith('127.'):  # 排除回环地址
                                interfaces[current_interface]['ips'].append(ip)

            # 过滤出有效的网络接口
            valid_interfaces = {}
            for name, info in interfaces.items():
                if (info['status'] == 'UP' and
                    info['ips'] and
                    not name.startswith('lo') and  # 排除loopback
                    not name.startswith('docker')):  # 排除docker接口

                    # 确定接口类型
                    if name.startswith('eth') or name.startswith('enp'):
                        info['type'] = 'ethernet'
                    elif name.startswith('wlan') or name.startswith('wlp'):
                        info['type'] = 'wireless'

                    valid_interfaces[name] = info

            return valid_interfaces

        except (subprocess.CalledProcessError, FileNotFoundError):
            # 备用方法：通过socket获取接口信息
            return self._get_interfaces_fallback()

    def _get_interfaces_fallback(self) -> Dict[str, Dict]:
        """备用方法获取网络接口信息"""
        try:
            # 尝试连接到外部地址来确定主要网络接口
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()

            # 简单的接口信息
            return {
                'auto': {
                    'name': 'auto',
                    'ips': [local_ip],
                    'status': 'UP',
                    'type': 'auto-detected'
                }
            }
        except:
            return {}

    def get_recommended_interface(self, target_subnet: str = "192.168.1") -> Optional[str]:
        """获取推荐的网络接口

        Args:
            target_subnet: 目标子网前缀，如 "192.168.1"

        Returns:
            推荐的接口名称，如果没有找到则返回None
        """
        interfaces = self.get_network_interfaces()

        # 优先查找同子网的接口
        for name, info in interfaces.items():
            for ip in info['ips']:
                if ip.startswith(target_subnet):
                    return name

        # 如果没有同子网的接口，返回第一个以太网接口
        for name, info in interfaces.items():
            if info['type'] == 'ethernet':
                return name

        # 如果没有以太网接口，返回第一个可用接口
        if interfaces:
            return list(interfaces.keys())[0]

        return None

    def check_network_connectivity(self, host_ip: str, lidar_ip: str) -> Dict[str, bool]:
        """检查网络连通性

        Args:
            host_ip: 主机IP地址
            lidar_ip: 雷达IP地址

        Returns:
            连通性检查结果
        """
        results = {
            'host_reachable': False,
            'lidar_reachable': False,
            'same_subnet': False
        }

        try:
            # 检查主机IP是否可达（实际上检查的是本机接口）
            results['host_reachable'] = self._ping_host(host_ip)

            # 检查雷达IP是否可达
            results['lidar_reachable'] = self._ping_host(lidar_ip)

            # 检查是否在同一子网
            host_parts = host_ip.split('.')
            lidar_parts = lidar_ip.split('.')
            if len(host_parts) == 4 and len(lidar_parts) == 4:
                # 假设使用标准的C类子网掩码
                results['same_subnet'] = (host_parts[:3] == lidar_parts[:3])

        except Exception as e:
            print(f"网络连通性检查失败: {e}")

        return results

    def _ping_host(self, host: str, timeout: int = 3) -> bool:
        """Ping主机检查连通性"""
        try:
            # 使用ping命令
            result = subprocess.run(['ping', '-c', '1', '-W', str(timeout), host],
                                    capture_output=True, text=True)
            return result.returncode == 0
        except:
            return False

    def generate_network_report(self, host_ip: str = "192.168.1.50",
                               lidar_ip: str = "192.168.1.3") -> str:
        """生成网络配置报告"""
        interfaces = self.get_network_interfaces()
        recommended = self.get_recommended_interface()
        connectivity = self.check_network_connectivity(host_ip, lidar_ip)

        report = []
        report.append("="*60)
        report.append("🌐 网络接口检测报告")
        report.append("="*60)

        if not interfaces:
            report.append("❌ 未检测到可用的网络接口")
            return "\n".join(report)

        report.append(f"检测到 {len(interfaces)} 个可用网络接口:")
        report.append("")

        for name, info in interfaces.items():
            status_icon = "✅" if info['status'] == 'UP' else "❌"
            report.append(f"{status_icon} {name} ({info['type']})")
            for ip in info['ips']:
                report.append(f"    IP: {ip}")
            report.append("")

        if recommended:
            report.append(f"🎯 推荐接口: {recommended}")
            if recommended in interfaces:
                recommended_ip = interfaces[recommended]['ips'][0] if interfaces[recommended]['ips'] else "未知"
                report.append(f"    推荐IP: {recommended_ip}")
        else:
            report.append("⚠️  未找到推荐的网络接口")

        report.append("")
        report.append("🔍 网络连通性检查:")

        connectivity_status = "✅" if connectivity['host_reachable'] else "❌"
        report.append(f"    主机IP {host_ip}: {connectivity_status}")

        connectivity_status = "✅" if connectivity['lidar_reachable'] else "❌"
        report.append(f"    雷达IP {lidar_ip}: {connectivity_status}")

        subnet_status = "✅" if connectivity['same_subnet'] else "❌"
        report.append(f"    同一子网: {subnet_status}")

        if not connectivity['same_subnet']:
            report.append("")
            report.append("⚠️  主机和雷达不在同一子网，可能导致通信问题")

        return "\n".join(report)

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='网络接口自动检测工具')
    parser.add_argument('--host-ip', default='192.168.1.50',
                        help='主机IP地址 (默认: 192.168.1.50)')
    parser.add_argument('--lidar-ip', default='192.168.1.3',
                        help='雷达IP地址 (默认: 192.168.1.3)')
    parser.add_argument('--recommend-only', action='store_true',
                        help='只输出推荐的接口名称')

    args = parser.parse_args()

    detector = NetworkDetector()

    if args.recommend_only:
        recommended = detector.get_recommended_interface()
        if recommended:
            print(recommended)
        else:
            print("auto")  # 默认值
    else:
        report = detector.generate_network_report(args.host_ip, args.lidar_ip)
        print(report)

if __name__ == "__main__":
    main()