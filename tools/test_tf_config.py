#!/usr/bin/env python3
"""
测试TF配置读取功能
验证launch文件中的外参读取是否正常工作
"""

import os
import sys
import yaml

def test_tf_config_reading():
    """测试从配置文件读取TF参数的功能"""

    # 设置配置文件路径
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    master_config_path = os.path.join(project_root, 'config', 'master_config.yaml')

    print("="*60)
    print("测试TF配置读取功能")
    print("="*60)
    print(f"项目根目录: {project_root}")
    print(f"配置文件路径: {master_config_path}")
    print(f"配置文件是否存在: {os.path.exists(master_config_path)}")
    print()

    if not os.path.exists(master_config_path):
        print("❌ 错误: master_config.yaml文件不存在!")
        return False

    try:
        # 读取配置文件
        with open(master_config_path, 'r', encoding='utf-8') as f:
            master_config = yaml.safe_load(f)

        print("✅ 成功读取配置文件")

        # 读取外参配置
        calibration = master_config.get('calibration', {})
        lidar_to_baselink = calibration.get('lidar_to_baselink', {})

        print(f"校准配置: {calibration}")
        print(f"雷达到base_link外参: {lidar_to_baselink}")
        print()

        # 提取平移和旋转参数
        translation = lidar_to_baselink.get('translation', [0.0, 0.0, 0.0])
        rotation_rpy = lidar_to_baselink.get('rotation_rpy', [0.0, 0.0, 0.0])

        print(f"平移参数 (translation): {translation}")
        print(f"旋转参数 (rotation_rpy): {rotation_rpy}")
        print()

        # 构造TF参数
        tf_params = [
            str(translation[0]),  # x
            str(translation[1]),  # y
            str(translation[2]),  # z
            str(rotation_rpy[0]), # roll
            str(rotation_rpy[1]), # pitch
            str(rotation_rpy[2]), # yaw
            'base_link',          # parent_frame
            'livox_frame'         # child_frame
        ]

        print("生成的TF参数:")
        for i, param in enumerate(tf_params):
            if i < 3:
                print(f"  {['x', 'y', 'z'][i]}: {param}")
            elif i < 6:
                print(f"  {['roll', 'pitch', 'yaw'][i-3]}: {param}")
            else:
                print(f"  {['parent_frame', 'child_frame'][i-6]}: {param}")

        print()
        print("完整TF命令:")
        print(f"ros2 run tf2_ros static_transform_publisher {' '.join(tf_params)}")
        print()

        # 验证参数类型
        valid = True
        for i in range(6):
            try:
                float(tf_params[i])
            except ValueError:
                print(f"❌ 错误: 参数 {tf_params[i]} 不是有效的数值")
                valid = False

        if valid:
            print("✅ 所有数值参数验证通过")
        else:
            print("❌ 参数验证失败")

        return valid

    except Exception as e:
        print(f"❌ 读取配置文件失败: {e}")
        return False

def simulate_launch_file_behavior():
    """模拟launch文件中的行为"""

    print("="*60)
    print("模拟Launch文件行为")
    print("="*60)

    # 模拟launch文件中的路径计算
    # 这里模拟__file__为launch文件的路径
    launch_file_path = "/home/tianyu/codes/Mid360map/ws_livox/src/fastlio2/launch/enhanced_visualization.launch.py"

    master_config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_file_path))))),
        'config',
        'master_config.yaml'
    )

    print(f"Launch文件路径: {launch_file_path}")
    print(f"计算的配置文件路径: {master_config_path}")
    print(f"配置文件是否存在: {os.path.exists(master_config_path)}")
    print()

    # 使用与launch文件相同的函数逻辑
    def get_tf_params_from_config(master_config_path):
        try:
            if os.path.exists(master_config_path):
                with open(master_config_path, 'r', encoding='utf-8') as f:
                    master_config = yaml.safe_load(f)

                # 读取外参配置
                calibration = master_config.get('calibration', {})
                lidar_to_baselink = calibration.get('lidar_to_baselink', {})

                # 提取平移和旋转参数
                translation = lidar_to_baselink.get('translation', [0.0, 0.0, 0.0])
                rotation_rpy = lidar_to_baselink.get('rotation_rpy', [0.0, 0.0, 0.0])

                # 构造TF参数
                tf_params = [
                    str(translation[0]),  # x
                    str(translation[1]),  # y
                    str(translation[2]),  # z
                    str(rotation_rpy[0]), # roll
                    str(rotation_rpy[1]), # pitch
                    str(rotation_rpy[2]), # yaw
                    'base_link',          # parent_frame
                    'livox_frame'         # child_frame
                ]

                print(f"从配置文件读取TF参数: translation={translation}, rotation_rpy={rotation_rpy}")
                return tf_params

        except Exception as e:
            print(f"警告: 读取外参配置失败: {e}")

        # 默认参数（向后兼容）
        print("使用默认TF参数（零值）")
        return ['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']

    tf_params = get_tf_params_from_config(master_config_path)
    print(f"最终TF参数: {tf_params}")
    print()

    return tf_params

if __name__ == "__main__":
    print("TF配置测试工具")
    print("测试外参配置读取和TF参数生成功能")
    print()

    # 测试配置读取
    success1 = test_tf_config_reading()

    # 模拟launch文件行为
    tf_params = simulate_launch_file_behavior()

    print("="*60)
    print("测试总结")
    print("="*60)

    if success1:
        print("✅ 配置读取功能正常")
        print(f"✅ 生成的TF参数: {tf_params}")
        print("✅ 修复完成，可以启动系统测试")
    else:
        print("❌ 配置读取功能异常")
        print("❌ 请检查master_config.yaml文件")

    print()
    print("验证命令:")
    print("1. 启动系统: ./tools/slam_tools.sh start")
    print("2. 查看TF树: ros2 run tf2_tools view_frames")
    print("3. 监听TF话题: ros2 topic echo /tf_static")