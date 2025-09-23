#!/usr/bin/env python3
"""
验证TF修复的完整测试脚本
检查配置文件读取、TF参数生成和系统集成
"""

import os
import sys
import yaml
import subprocess
import time

def check_dependencies():
    """检查必要的依赖"""
    print("检查系统依赖...")

    try:
        # 检查ROS2环境
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ ROS2环境: {result.stdout.strip()}")
        else:
            print("❌ ROS2环境未正确设置")
            return False

        # 检查tf2_ros包
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'tf2_ros' in result.stdout:
            print("✅ tf2_ros包可用")
        else:
            print("❌ tf2_ros包不可用")
            return False

        return True

    except Exception as e:
        print(f"❌ 依赖检查失败: {e}")
        return False

def test_config_reading():
    """测试配置文件读取功能"""
    print("\n" + "="*50)
    print("测试配置文件读取功能")
    print("="*50)

    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    master_config_path = os.path.join(project_root, 'config', 'master_config.yaml')

    print(f"配置文件路径: {master_config_path}")

    if not os.path.exists(master_config_path):
        print("❌ master_config.yaml不存在")
        return False

    try:
        with open(master_config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # 检查关键配置项
        required_keys = ['calibration', 'frames', 'topics']
        for key in required_keys:
            if key not in config:
                print(f"❌ 缺少配置项: {key}")
                return False

        # 检查外参配置
        calibration = config.get('calibration', {})
        lidar_to_baselink = calibration.get('lidar_to_baselink', {})

        if 'translation' not in lidar_to_baselink or 'rotation_rpy' not in lidar_to_baselink:
            print("❌ 外参配置不完整")
            return False

        translation = lidar_to_baselink['translation']
        rotation_rpy = lidar_to_baselink['rotation_rpy']

        print(f"✅ 外参配置: translation={translation}, rotation_rpy={rotation_rpy}")
        return True

    except Exception as e:
        print(f"❌ 配置文件读取失败: {e}")
        return False

def test_launch_file_integration():
    """测试launch文件集成"""
    print("\n" + "="*50)
    print("测试Launch文件集成")
    print("="*50)

    # 导入launch文件模块
    sys.path.append('/home/tianyu/codes/Mid360map/ws_livox/src/fastlio2/launch')

    try:
        # 重新设置路径以模拟正确的launch环境
        launch_file_path = '/home/tianyu/codes/Mid360map/ws_livox/src/fastlio2/launch/enhanced_visualization.launch.py'

        # 读取launch文件内容并执行配置函数
        with open(launch_file_path, 'r') as f:
            launch_content = f.read()

        # 创建临时环境来执行函数
        exec_env = {}
        exec(launch_content, exec_env)

        # 测试get_tf_params_from_config函数
        get_tf_params_func = exec_env.get('get_tf_params_from_config')
        if get_tf_params_func:
            master_config_path = '/home/tianyu/codes/Mid360map/config/master_config.yaml'
            tf_params = get_tf_params_func(master_config_path)

            print(f"✅ TF参数生成成功: {tf_params}")

            # 验证参数格式
            if len(tf_params) == 8:
                print("✅ TF参数数量正确 (8个)")
            else:
                print(f"❌ TF参数数量错误: {len(tf_params)}")
                return False

            # 验证数值参数
            for i in range(6):
                try:
                    float(tf_params[i])
                    print(f"✅ 参数 {i+1} ({tf_params[i]}) 是有效数值")
                except ValueError:
                    print(f"❌ 参数 {i+1} ({tf_params[i]}) 不是有效数值")
                    return False

            # 验证坐标系名称
            if tf_params[6] == 'base_link' and tf_params[7] == 'livox_frame':
                print("✅ 坐标系名称正确")
            else:
                print(f"❌ 坐标系名称错误: {tf_params[6]} -> {tf_params[7]}")
                return False

            return True
        else:
            print("❌ 无法找到get_tf_params_from_config函数")
            return False

    except Exception as e:
        print(f"❌ Launch文件集成测试失败: {e}")
        return False

def test_tf_publisher():
    """测试TF发布器功能"""
    print("\n" + "="*50)
    print("测试TF发布器功能")
    print("="*50)

    try:
        # 启动一个测试TF发布器
        cmd = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0.1', '0.05', '0.2', '0.0', '0.1', '0.05',
            'test_base_link', 'test_livox_frame'
        ]

        print(f"启动测试TF发布器: {' '.join(cmd)}")

        # 启动发布器进程
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 等待一下让发布器启动
        time.sleep(2)

        # 检查TF是否发布
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '/tf_static', '--once'],
                capture_output=True, text=True, timeout=5
            )

            if 'test_base_link' in result.stdout and 'test_livox_frame' in result.stdout:
                print("✅ TF发布器正常工作")
                success = True
            else:
                print("❌ TF未正确发布")
                success = False

        except subprocess.TimeoutExpired:
            print("❌ TF话题读取超时")
            success = False

        # 终止测试进程
        process.terminate()
        process.wait()

        return success

    except Exception as e:
        print(f"❌ TF发布器测试失败: {e}")
        return False

def generate_test_report():
    """生成测试报告"""
    print("\n" + "="*60)
    print("TF修复验证报告")
    print("="*60)

    tests = [
        ("系统依赖检查", check_dependencies),
        ("配置文件读取", test_config_reading),
        ("Launch文件集成", test_launch_file_integration),
        ("TF发布器功能", test_tf_publisher),
    ]

    results = []

    for test_name, test_func in tests:
        print(f"\n正在执行: {test_name}")
        try:
            result = test_func()
            results.append((test_name, result))
            status = "✅ 通过" if result else "❌ 失败"
            print(f"{test_name}: {status}")
        except Exception as e:
            results.append((test_name, False))
            print(f"{test_name}: ❌ 异常 - {e}")

    print("\n" + "="*60)
    print("测试结果汇总")
    print("="*60)

    passed = 0
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name:<20}: {status}")
        if result:
            passed += 1

    print(f"\n总体结果: {passed}/{len(results)} 测试通过")

    if passed == len(results):
        print("\n🎉 所有测试通过！TF修复验证成功！")
        print("\n后续步骤:")
        print("1. 编译系统: ./tools/slam_tools.sh build")
        print("2. 启动系统: ./tools/slam_tools.sh start")
        print("3. 验证TF: ros2 run tf2_tools view_frames")
        print("4. 监控TF: ros2 topic echo /tf_static")
    else:
        print(f"\n⚠️  {len(results)-passed} 个测试失败，请检查相关问题")

    return passed == len(results)

if __name__ == "__main__":
    print("TF修复验证工具")
    print("全面测试配置读取、TF参数生成和系统集成")

    success = generate_test_report()

    if success:
        sys.exit(0)
    else:
        sys.exit(1)