# 点云处理器GUI安装指南

## 快速启动

### 1️⃣ 检查环境
```bash
# 检查Python版本（需要3.8+）
python3 --version

# 检查已安装的依赖
python3 -c "import PyQt5, numpy, matplotlib, open3d; print('✅ 所有依赖已安装')"
```

### 2️⃣ 安装缺失依赖
```bash
# 安装PyQt5
pip install PyQt5

# 安装其他依赖
pip install numpy matplotlib open3d
```

### 3️⃣ 启动GUI
```bash
# 方法1：使用启动脚本（推荐）
cd /path/to/Mid360map
./tools/start_gui.sh

# 方法2：直接启动
cd /path/to/Mid360map/tools
python3 point_cloud_processor_gui.py
```

## 详细安装步骤

### Ubuntu系统
```bash
# 更新包列表
sudo apt update

# 安装系统依赖
sudo apt install python3-pip python3-pyqt5

# 安装Python库
pip3 install numpy matplotlib open3d
```

### 其他Linux发行版
```bash
# 使用pip安装所有依赖
pip3 install PyQt5 numpy matplotlib open3d
```

## 验证安装

### 测试基本GUI功能
```bash
cd /path/to/Mid360map/tools
python3 test_gui_basic.py
```
如果看到测试窗口，说明PyQt5安装正确。

### 测试完整功能
```bash
cd /path/to/Mid360map/tools
python3 -c "
from point_cloud_processor_gui import PointCloudProcessorGUI
print('✅ GUI可以正常加载')
"
```

## 生成的文件说明

安装完成后，工具目录包含以下GUI相关文件：

```
tools/
├── point_cloud_processor_gui.py     # 主GUI程序
├── test_gui_basic.py                # GUI基本功能测试
├── start_gui.sh                     # 启动脚本
├── GUI_README.md                    # GUI使用说明
├── INSTALL_GUI.md                   # 本安装指南
└── advanced_point_cloud_processor.py # 后端处理器
```

## 使用示例

### 第一次使用
1. 运行 `./tools/start_gui.sh`
2. 点击"浏览..."选择测试PCD文件
3. 保持默认参数设置
4. 点击"🚀 开始处理"
5. 查看处理日志和结果统计

### 高级功能测试
1. 启用"表面重建"
2. 选择"Alpha Shape"方法
3. 设置Alpha值为0.05
4. 开始处理并查看生成的PLY网格文件

## GUI界面预览

界面分为以下主要区域：

**左侧控制面板**：
- 📁 文件设置（输入/输出路径）
- ⚙️ 处理参数（强度过滤、去噪选项）
- 🏗️ 表面重建（4种算法选择）
- 🎮 操作控制（开始/停止/重置）
- 📊 进度显示

**右侧结果面板**：
- 📋 处理日志（实时状态更新）
- 📈 结果统计（详细处理报告）
- 📖 使用说明（内置帮助文档）

## 故障排除

### 常见错误及解决方案

**1. 导入错误**
```
错误: ImportError: No module named 'PyQt5'
解决: pip install PyQt5
```

**2. 权限错误**
```
错误: Permission denied: ./start_gui.sh
解决: chmod +x tools/start_gui.sh
```

**3. 显示问题**
```
错误: cannot connect to X server
解决: 确保运行在图形化环境中，或设置DISPLAY变量
```

**4. 依赖冲突**
```
错误: 版本不兼容
解决: 使用虚拟环境隔离依赖
python3 -m venv venv
source venv/bin/activate  
pip install PyQt5 numpy matplotlib open3d
```

## 性能优化建议

1. **系统资源**：建议至少8GB内存用于处理大型点云
2. **显卡加速**：Open3D支持GPU加速，如有独立显卡可提升性能
3. **多线程**：GUI使用后台线程，避免界面卡顿
4. **文件大小**：超过100万点的点云建议先下采样

现在你可以使用功能完整的图形化点云处理工具了！