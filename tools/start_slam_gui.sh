#!/bin/bash

# SLAM_GUI启动脚本
# 作者：Claude AI Assistant
# 日期：2025-09-13

echo "🚀 启动SLAM可视化控制中心..."

# 工作空间路径
WORKSPACE_PATH="/home/tianyu/codes/Mid360map/ws_livox"

# 检查工作空间
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "❌ 工作空间不存在: $WORKSPACE_PATH"
    exit 1
fi

cd "$WORKSPACE_PATH"

# 检查编译
if [ ! -f "install/slam_gui/lib/slam_gui/slam_gui" ]; then
    echo "❌ SLAM_GUI未编译，正在编译..."
    colcon build --packages-select slam_gui --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -ne 0 ]; then
        echo "❌ 编译失败"
        exit 1
    fi
    echo "✅ 编译完成"
fi

# 设置ROS2环境
echo "📦 设置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查显示环境
if [ -z "$DISPLAY" ]; then
    echo "⚠️  未检测到显示环境，尝试设置..."
    export DISPLAY=:0
fi

# 检查ROS2是否可用
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2命令不可用"
    exit 1
fi

echo "✅ 环境检查完成"

# 启动选项
echo ""
echo "请选择启动方式："
echo "1) 直接启动 (ros2 run)"
echo "2) Launch文件启动 (推荐)"
echo "3) 查看帮助信息"
echo "4) 退出"
echo ""

read -p "请输入选择 [1-4]: " choice

case $choice in
    1)
        echo "🖥️ 直接启动SLAM_GUI..."
        ros2 run slam_gui slam_gui
        ;;
    2)
        echo "🖥️ 使用Launch文件启动SLAM_GUI..."
        ros2 launch slam_gui slam_gui.launch.py
        ;;
    3)
        echo ""
        echo "📖 SLAM_GUI使用说明："
        echo "- 这是一个基于Qt5的ROS2图形界面程序"
        echo "- 用于实时监控和控制SLAM系统"
        echo "- 支持FastLIO2、PGO、HBA、Localizer等组件"
        echo "- 提供参数调整、地图保存、轨迹导出等功能"
        echo ""
        echo "💡 启动提示："
        echo "- 需要图形界面环境（X11或Wayland）"
        echo "- 建议先启动SLAM系统再启动GUI"
        echo "- 如遇到段错误，请检查显示环境配置"
        echo ""
        ;;
    4)
        echo "👋 退出"
        exit 0
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

echo "👋 SLAM_GUI已退出"