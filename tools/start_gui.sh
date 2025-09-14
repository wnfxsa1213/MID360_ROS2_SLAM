#!/bin/bash

# 点云处理器GUI启动脚本

echo "🚀 启动Advanced Point Cloud Processor GUI..."

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "❌ 未找到python3"
    exit 1
fi

# 检查PyQt5
if ! python3 -c "import PyQt5" 2>/dev/null; then
    echo "❌ PyQt5未安装，请运行: pip install PyQt5"
    exit 1
fi

# 检查其他依赖
echo "📦 检查依赖库..."
python3 -c "
import numpy
import matplotlib
import open3d
print('✅ 所有依赖库已安装')
" || {
    echo "❌ 缺少必要的依赖库"
    echo "请运行以下命令安装："
    echo "pip install numpy matplotlib open3d PyQt5"
    exit 1
}

# 切换到工具目录
cd "$(dirname "$0")"

# 启动GUI
echo "🖥️ 启动图形界面..."
python3 point_cloud_processor_gui.py

echo "👋 GUI已退出"