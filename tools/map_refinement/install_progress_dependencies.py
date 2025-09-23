#!/usr/bin/env python3
"""
进度条依赖安装脚本

安装地图精细化工具进度显示功能所需的依赖库。
"""

import subprocess
import sys
import importlib


def check_package(package_name):
    """检查包是否已安装"""
    try:
        importlib.import_module(package_name)
        return True
    except ImportError:
        return False


def install_package(package_name):
    """安装Python包"""
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
        print(f"✅ 成功安装 {package_name}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ 安装 {package_name} 失败: {e}")
        return False


def main():
    """主安装流程"""
    print("🔧 开始安装地图精细化工具进度显示依赖...")

    required_packages = {
        'tqdm': 'tqdm',  # 包名: pip安装名
    }

    installed_count = 0
    total_count = len(required_packages)

    for package_name, pip_name in required_packages.items():
        print(f"\n📦 检查 {package_name}...")

        if check_package(package_name):
            print(f"✅ {package_name} 已安装")
            installed_count += 1
        else:
            print(f"⚠️  {package_name} 未安装，开始安装...")
            if install_package(pip_name):
                installed_count += 1

    print(f"\n📊 安装结果: {installed_count}/{total_count} 个包安装成功")

    if installed_count == total_count:
        print("🎉 所有依赖安装完成！")
        print("\n💡 使用说明:")
        print("   - 如果安装了tqdm，将显示美观的彩色进度条")
        print("   - 如果没有tqdm，将使用简单的文本进度显示")
        print("   - 可在配置文件中设置 progress_display: true 启用进度显示")

        # 测试进度条功能
        print("\n🧪 测试进度条功能...")
        try:
            from tqdm import tqdm
            import time

            print("测试tqdm进度条:")
            for i in tqdm(range(20), desc="测试进度"):
                time.sleep(0.05)
            print("✅ tqdm进度条测试成功")

        except ImportError:
            print("⚠️  tqdm未安装，将使用简单进度显示")

        return True
    else:
        print("❌ 部分依赖安装失败，进度显示功能可能受限")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)