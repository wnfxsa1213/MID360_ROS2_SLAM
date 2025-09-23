# matplotlib中文字体解决方案

## 概述

本解决方案为SLAM系统中的Python可视化工具提供完整的中文字体支持，解决了matplotlib中文标签无法正常显示的问题。

### 主要特性

- ✅ 自动检测系统可用的中文字体
- ✅ 智能选择最佳中文字体
- ✅ 统一的字体配置管理
- ✅ 完整的测试验证体系
- ✅ 兼容现有可视化工具
- ✅ 支持多种图表类型

## 解决方案架构

### 核心组件

1. **字体配置模块** (`chinese_font_config.py`)
   - 自动检测和配置中文字体
   - 提供统一的字体管理接口
   - 支持字体状态检查和测试

2. **可视化工具集成** (`visualize_dynamic_filter.py`)
   - 在原有工具中集成中文字体支持
   - 保持向后兼容性
   - 自动加载字体配置

3. **测试验证系统**
   - 基础字体测试 (`test_chinese_font.py`)
   - 完整可视化测试 (`test_visualization_chinese_labels.py`)
   - 多种图表类型验证

## 文件结构

```
tools/
├── chinese_font_config.py              # 核心字体配置模块
├── visualize_dynamic_filter.py         # 更新后的可视化工具
├── test_chinese_font.py                # 基础字体测试
├── test_visualization_chinese_labels.py # 完整可视化测试
├── MATPLOTLIB_CHINESE_FONT_SOLUTION.md # 本说明文档
├── chinese_font_test.png               # 基础测试输出
├── chinese_terms_test.png              # 术语测试输出
└── visualization_chinese_test_*.png    # 完整测试输出
```

## 安装和配置

### 前提条件

确保系统已安装Python依赖：
```bash
pip install matplotlib numpy
```

### 快速开始

1. **自动配置（推荐）**
   ```python
   from chinese_font_config import configure_chinese_font

   # 自动配置中文字体
   success = configure_chinese_font()
   if success:
       print("中文字体配置成功")
   ```

2. **手动指定字体**
   ```python
   from chinese_font_config import configure_chinese_font

   # 指定特定字体
   success = configure_chinese_font('WenQuanYi Micro Hei')
   ```

3. **检查配置状态**
   ```python
   from chinese_font_config import print_font_status, test_chinese_display

   # 打印字体状态
   print_font_status()

   # 测试中文显示
   test_ok = test_chinese_display()
   print(f"中文显示测试: {'通过' if test_ok else '失败'}")
   ```

## 使用方法

### 在新项目中使用

```python
#!/usr/bin/env python3
import matplotlib.pyplot as plt
from chinese_font_config import configure_chinese_font

# 配置中文字体
configure_chinese_font()

# 正常使用matplotlib
plt.figure(figsize=(10, 6))
plt.plot([1, 2, 3, 4], [1, 4, 2, 3])
plt.title('中文标题测试')
plt.xlabel('时间 (秒)')
plt.ylabel('数值')
plt.show()
```

### 在现有项目中集成

只需在matplotlib导入后添加一行：
```python
import matplotlib.pyplot as plt
from chinese_font_config import configure_chinese_font

configure_chinese_font()  # 添加这一行

# 其余代码保持不变
```

## 支持的中文字体

系统会按优先级自动选择以下字体：

1. **Noto Sans CJK SC** (推荐 - Google开发的现代字体)
2. **WenQuanYi Micro Hei** (文泉驿微米黑 - 开源字体)
3. **Noto Serif CJK SC** (Noto衬线字体)
4. **AR PL UMing CN** (文鼎PL新宋)
5. **AR PL UKai CN** (文鼎PL中楷)
6. **SimHei** (黑体 - Windows系统)
7. **Microsoft YaHei** (微软雅黑 - Windows系统)
8. **PingFang SC** (苹方 - macOS系统)

## 测试验证

### 运行基础测试

```bash
cd tools
python3 test_chinese_font.py
```

### 运行完整可视化测试

```bash
cd tools
python3 test_visualization_chinese_labels.py
```

### 运行可视化工具测试

```bash
cd tools
python3 -c "import visualize_dynamic_filter"
```

## 测试输出说明

测试会生成以下图片文件，用于验证中文显示效果：

1. **chinese_font_test.png** - 基础中文字体测试
2. **chinese_terms_test.png** - SLAM相关术语测试
3. **visualization_chinese_test_realtime.png** - 实时统计图表
4. **visualization_chinese_test_performance.png** - 性能监控图表
5. **visualization_chinese_test_pointcloud.png** - 点云对比图表
6. **visualization_chinese_test_all_labels.png** - 完整标签测试
7. **visualization_chinese_test_mixed_charts.png** - 混合图表类型

## 常见问题排除

### 问题1: 中文显示为方块

**原因**: 系统缺少中文字体或字体配置失败

**解决方案**:
```bash
# 检查系统中文字体
fc-list :lang=zh-cn

# 如果没有中文字体，安装文泉驿字体
sudo apt-get install fonts-wqy-microhei fonts-wqy-zenhei
```

### 问题2: 字体配置失败

**原因**: matplotlib缓存问题

**解决方案**:
```python
import matplotlib
matplotlib.font_manager._rebuild()  # 重建字体缓存
```

### 问题3: 负号显示异常

**原因**: matplotlib默认设置问题

**解决方案**: 字体配置模块已自动处理此问题
```python
plt.rcParams['axes.unicode_minus'] = False
```

### 问题4: 导入模块失败

**确保文件路径正确**:
```python
import sys
sys.path.append('/path/to/tools')  # 添加tools目录到路径
from chinese_font_config import configure_chinese_font
```

## API参考

### 主要函数

- `configure_chinese_font(font_name=None, force_reconfigure=False)` - 配置中文字体
- `get_available_fonts()` - 获取可用中文字体列表
- `get_current_font()` - 获取当前选择的字体
- `is_chinese_font_configured()` - 检查是否已配置
- `test_chinese_display()` - 测试中文显示
- `print_font_status()` - 打印字体状态

### ChineseFontManager类

```python
from chinese_font_config import ChineseFontManager

manager = ChineseFontManager()
available_fonts = manager.get_available_chinese_fonts()
success = manager.configure_matplotlib_chinese_font()
info = manager.get_font_info()
```

## 性能考虑

- 字体配置只在首次导入时执行，对性能影响最小
- 支持缓存机制，避免重复配置
- 自动选择系统最佳字体，无需手动干预

## 兼容性

- **Python**: 3.6+
- **matplotlib**: 3.0+
- **操作系统**: Linux, Windows, macOS
- **字体系统**: fontconfig, Windows字体系统

## 贡献和支持

如果遇到问题或需要改进，请：

1. 运行测试脚本确认问题
2. 检查字体状态 (`print_font_status()`)
3. 查看错误日志
4. 提供系统和环境信息

## 更新日志

- **v1.0** (2024-09-22)
  - 初始版本发布
  - 支持自动字体检测和配置
  - 完整的测试验证系统
  - 与现有可视化工具集成

## 许可证

本解决方案遵循项目的开源许可证。

---

**注意**: 本解决方案专门为Mid360map SLAM系统设计，但也可用于其他需要matplotlib中文支持的项目。