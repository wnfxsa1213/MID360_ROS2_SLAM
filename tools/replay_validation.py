#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLAM改进效果验证工具 - 使用录制数据包进行对比测试
用法: python3 replay_validation.py [选项]
"""

import os
import sys
import yaml
import argparse
import subprocess
import time
import signal
from datetime import datetime
from pathlib import Path

class SLAMValidator:
    def __init__(self):
        self.script_dir = Path(__file__).parent
        self.project_root = self.script_dir.parent
        self.ws_livox = self.project_root / "ws_livox"
        self.config_dir = self.project_root / "config"
        self.results_dir = self.project_root / "validation_results"

        # 确保结果目录存在
        self.results_dir.mkdir(exist_ok=True)

        self.processes = []

    def load_config(self):
        """加载配置文件"""
        config_path = self.config_dir / "launch_config.yaml"
        if config_path.exists():
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        return {}

    def find_bag_files(self):
        """查找可用的bag文件"""
        data_dir = self.project_root / "data" / "rosbags"
        if not data_dir.exists():
            return []

        bag_dirs = []
        for item in data_dir.iterdir():
            if item.is_dir() and any(item.glob("*.db3*")):
                bag_dirs.append(item)
        return sorted(bag_dirs, key=lambda x: x.stat().st_mtime, reverse=True)

    def get_bag_info(self, bag_path):
        """获取bag文件信息"""
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', str(bag_path)],
                capture_output=True, text=True, check=True
            )
            return result.stdout
        except subprocess.CalledProcessError:
            return "无法读取bag信息"

    def backup_current_config(self, test_name):
        """备份当前配置"""
        config_file = self.ws_livox / "src" / "fastlio2" / "config" / "lio.yaml"
        backup_file = self.results_dir / f"{test_name}_lio_config.yaml"

        if config_file.exists():
            import shutil
            shutil.copy2(config_file, backup_file)
            print(f"📋 配置已备份到: {backup_file}")

    def setup_environment(self):
        """设置ROS2环境"""
        env = os.environ.copy()
        setup_script = self.ws_livox / "install" / "setup.bash"

        if setup_script.exists():
            # 获取setup.bash的环境变量
            result = subprocess.run(
                f'source {setup_script} && env',
                shell=True, capture_output=True, text=True
            )

            for line in result.stdout.split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    env[key] = value

        return env

    def kill_all_processes(self):
        """停止所有相关进程"""
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.processes.clear()

        # 额外确保SLAM相关进程停止
        subprocess.run(['pkill', '-f', 'fastlio2'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-f', 'rviz'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-f', 'livox'], stderr=subprocess.DEVNULL)
        time.sleep(2)

    def start_slam_system(self, use_bag=True):
        """启动SLAM系统"""
        env = self.setup_environment()

        print("🚀 启动SLAM系统...")

        # 启动FAST-LIO2节点
        fastlio_cmd = [
            'ros2', 'run', 'fastlio2', 'lio_node',
            '--ros-args', '-p', f'config_path:={self.ws_livox}/src/fastlio2/config/lio.yaml',
            '-p', f'use_sim_time:={str(use_bag).lower()}'
        ]

        fastlio_proc = subprocess.Popen(
            fastlio_cmd, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        self.processes.append(fastlio_proc)

        # 启动静态TF
        # Base Link (IMU) -> Livox Frame (LiDAR)
        # IMU in LiDAR frame: (0.011, 0.02329, -0.04412) => IMU->LiDAR: (-0.011, -0.02329, 0.04412)
        tf_proc = subprocess.Popen([
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '-0.011', '-0.02329', '0.04412', '0', '0', '0', 'base_link', 'livox_frame'
        ], env=env)
        self.processes.append(tf_proc)

        # 等待节点启动
        time.sleep(3)
        print("✅ SLAM系统已启动")

    def start_rviz(self):
        """启动RViz可视化"""
        env = self.setup_environment()
        rviz_config = self.ws_livox / "src" / "fastlio2" / "rviz" / "enhanced_fastlio2.rviz"

        rviz_proc = subprocess.Popen([
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', str(rviz_config)
        ], env=env)
        self.processes.append(rviz_proc)
        # 等待RViz启动后启用仿真时间，防止消息过滤器队列溢出
        time.sleep(2)
        try:
            subprocess.run([
                'ros2', 'param', 'set', '/rviz2', 'use_sim_time', 'true'
            ], env=env, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
        print("🖥️ RViz已启动（use_sim_time=true）")

    def play_bag(self, bag_path, rate=1.0, start_offset=0.0):
        """回放bag文件"""
        cmd = [
            'ros2', 'bag', 'play', str(bag_path),
            '--rate', str(rate),
            '--start-offset', str(start_offset),
            '--clock'
        ]

        print(f"▶️ 开始回放: {bag_path.name}")
        print(f"⚡ 回放速率: {rate}x")
        if start_offset > 0:
            print(f"⏩ 起始偏移: {start_offset}s")

        bag_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.processes.append(bag_proc)

        return bag_proc

    def save_test_map(self, test_name):
        """保存测试地图"""
        map_save_dir = self.results_dir / test_name
        map_save_dir.mkdir(exist_ok=True)

        # 调用地图保存服务
        save_cmd = [
            'ros2', 'service', 'call', '/save_map',
            'std_srvs/srv/Empty', '{}'
        ]

        try:
            result = subprocess.run(save_cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                # 移动保存的地图到测试目录
                saved_maps_dir = self.project_root / "saved_maps"
                if saved_maps_dir.exists():
                    import shutil
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    dest_file = map_save_dir / f"slam_map_{timestamp}.pcd"

                    # 查找最新的地图文件
                    map_files = list(saved_maps_dir.glob("*.pcd"))
                    if map_files:
                        latest_map = max(map_files, key=lambda x: x.stat().st_mtime)
                        shutil.copy2(latest_map, dest_file)
                        print(f"💾 地图已保存到: {dest_file}")
                        return dest_file
            else:
                print(f"❌ 地图保存失败: {result.stderr}")
        except subprocess.TimeoutExpired:
            print("❌ 地图保存超时")
        except Exception as e:
            print(f"❌ 地图保存出错: {e}")

        return None

    def run_validation_test(self, bag_path, test_name, rate=1.0, with_rviz=True):
        """运行单次验证测试"""
        print("="*60)
        print(f"🧪 开始测试: {test_name}")
        print(f"📦 数据包: {bag_path.name}")
        print("="*60)

        try:
            # 备份当前配置
            self.backup_current_config(test_name)

            # 启动SLAM系统
            self.start_slam_system(use_bag=True)

            # 可选启动RViz
            if with_rviz:
                self.start_rviz()

            # 回放数据包
            bag_proc = self.play_bag(bag_path, rate=rate)

            print("📊 回放进行中... (Ctrl+C 可随时停止)")

            # 等待回放完成
            bag_proc.wait()

            # 等待处理完成
            print("⏳ 等待SLAM处理完成...")
            time.sleep(5)

            # 保存测试结果地图
            map_file = self.save_test_map(test_name)

            print(f"✅ 测试 '{test_name}' 完成")
            if map_file:
                print(f"📍 结果地图: {map_file}")

        except KeyboardInterrupt:
            print("\n⚠️ 用户中断测试")
        except Exception as e:
            print(f"❌ 测试出错: {e}")
        finally:
            self.kill_all_processes()
            print("🛑 所有进程已停止")

    def compare_results(self):
        """比较测试结果"""
        print("\n📊 测试结果分析:")
        print("-" * 40)

        result_dirs = [d for d in self.results_dir.iterdir() if d.is_dir()]

        for result_dir in sorted(result_dirs):
            print(f"\n📁 测试: {result_dir.name}")

            # 查找地图文件
            map_files = list(result_dir.glob("*.pcd"))
            config_files = list(result_dir.glob("*_lio_config.yaml"))

            if map_files:
                latest_map = max(map_files, key=lambda x: x.stat().st_mtime)
                file_size = latest_map.stat().st_size / (1024*1024)  # MB
                print(f"  🗺️  地图文件: {latest_map.name} ({file_size:.1f} MB)")

            if config_files:
                print(f"  ⚙️  配置备份: {config_files[0].name}")

def main():
    parser = argparse.ArgumentParser(description='SLAM改进效果验证工具')
    parser.add_argument('-b', '--bag', help='指定bag文件路径')
    parser.add_argument('-r', '--rate', type=float, default=1.0, help='回放速率 (默认: 1.0)')
    parser.add_argument('-n', '--name', help='测试名称 (默认: 自动生成)')
    parser.add_argument('--no-rviz', action='store_true', help='不启动RViz')
    parser.add_argument('--list-bags', action='store_true', help='列出可用的bag文件')
    parser.add_argument('--compare', action='store_true', help='比较测试结果')
    parser.add_argument('--start-offset', type=float, default=0.0, help='回放起始偏移时间(秒)')

    args = parser.parse_args()

    validator = SLAMValidator()

    if args.compare:
        validator.compare_results()
        return

    if args.list_bags:
        bags = validator.find_bag_files()
        if bags:
            print("📦 可用的数据包:")
            for i, bag in enumerate(bags):
                print(f"{i+1}. {bag.name}")
                info = validator.get_bag_info(bag)
                # 提取关键信息
                for line in info.split('\n'):
                    if 'Duration:' in line or 'Messages:' in line or 'Bag size:' in line:
                        print(f"   {line.strip()}")
        else:
            print("❌ 未找到可用的数据包")
        return

    # 选择bag文件
    if args.bag:
        bag_path = Path(args.bag)
        if not bag_path.exists():
            print(f"❌ 数据包不存在: {bag_path}")
            sys.exit(1)
    else:
        bags = validator.find_bag_files()
        if not bags:
            print("❌ 未找到可用的数据包，请先录制数据")
            sys.exit(1)
        bag_path = bags[0]  # 使用最新的bag
        print(f"📦 使用最新数据包: {bag_path.name}")

    # 生成测试名称
    if args.name:
        test_name = args.name
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        test_name = f"validation_{timestamp}"

    # 运行验证测试
    validator.run_validation_test(
        bag_path=bag_path,
        test_name=test_name,
        rate=args.rate,
        with_rviz=not args.no_rviz
    )

    print(f"\n🎯 验证完成! 使用 --compare 参数查看结果对比")

if __name__ == '__main__':
    main()
