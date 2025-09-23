#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
matplotlib中文字体配置模块

该模块提供统一的中文字体配置功能，支持：
- 自动检测系统可用的中文字体
- 配置matplotlib使用最佳中文字体
- 提供字体配置状态检查
- 支持多种中文字体备选方案

作者: SLAM系统
日期: 2024
"""

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from typing import List, Optional
import warnings

class ChineseFontManager:
    """中文字体管理器"""

    # 优先考虑的中文字体列表（按优先级排序）
    PREFERRED_FONTS = [
        'Noto Sans CJK SC',        # Noto Sans 简体中文（推荐）
        'WenQuanYi Micro Hei',     # 文泉驿微米黑（开源字体）
        'Noto Serif CJK SC',       # Noto Serif 简体中文
        'AR PL UMing CN',          # 文鼎PL新宋
        'AR PL UKai CN',           # 文鼎PL中楷
        'SimHei',                  # 黑体（Windows系统）
        'Microsoft YaHei',         # 微软雅黑（Windows系统）
        'PingFang SC',             # 苹方（macOS系统）
        'Hiragino Sans GB'         # 冬青黑体（macOS系统）
    ]

    def __init__(self):
        self.available_fonts: List[str] = []
        self.selected_font: Optional[str] = None
        self.is_configured: bool = False

    def get_available_chinese_fonts(self) -> List[str]:
        """获取系统中可用的中文字体"""
        if self.available_fonts:
            return self.available_fonts

        chinese_fonts = []
        available_fonts = [font.name for font in fm.fontManager.ttflist]

        # 按优先级查找可用字体
        for preferred in self.PREFERRED_FONTS:
            for available in available_fonts:
                if preferred in available:
                    if available not in chinese_fonts:
                        chinese_fonts.append(available)

        # 如果没找到首选字体，查找任何包含中文关键词的字体
        if not chinese_fonts:
            for font in available_fonts:
                if any(keyword in font.lower() for keyword in
                      ['cjk', 'chinese', 'hei', 'song', 'kai', 'ming', 'yahei', 'pingfang']):
                    if font not in chinese_fonts:
                        chinese_fonts.append(font)

        self.available_fonts = chinese_fonts
        return chinese_fonts

    def configure_matplotlib_chinese_font(self, font_name: Optional[str] = None) -> bool:
        """
        配置matplotlib使用中文字体

        Args:
            font_name: 指定的字体名称，如果为None则自动选择最佳字体

        Returns:
            bool: 配置是否成功
        """
        try:
            # 获取可用字体
            available_fonts = self.get_available_chinese_fonts()

            if not available_fonts:
                warnings.warn("系统中未找到可用的中文字体，中文可能无法正常显示")
                return False

            # 选择字体
            if font_name and font_name in available_fonts:
                selected_font = font_name
            else:
                selected_font = available_fonts[0]  # 选择优先级最高的字体

            # 配置matplotlib
            font_list = [selected_font, 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
            plt.rcParams['font.sans-serif'] = font_list
            plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号
            plt.rcParams['font.size'] = 10              # 设置基础字体大小

            # 更新内部状态
            self.selected_font = selected_font
            self.is_configured = True

            return True

        except Exception as e:
            warnings.warn(f"配置中文字体失败: {e}")
            return False

    def get_font_info(self) -> dict:
        """获取当前字体配置信息"""
        return {
            'available_fonts': self.available_fonts,
            'selected_font': self.selected_font,
            'is_configured': self.is_configured,
            'current_sans_serif': list(plt.rcParams['font.sans-serif']),
            'unicode_minus_fixed': not plt.rcParams['axes.unicode_minus']
        }

    def test_chinese_rendering(self) -> bool:
        """测试中文字体渲染是否正常"""
        try:
            # 创建一个简单的图形来测试字体渲染
            fig, ax = plt.subplots(figsize=(4, 2))
            test_text = "中文字体测试"
            ax.text(0.5, 0.5, test_text, fontsize=12, ha='center', va='center')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
            plt.close(fig)  # 关闭图形避免显示
            return True
        except Exception:
            return False

# 全局字体管理器实例
_font_manager = ChineseFontManager()

def configure_chinese_font(font_name: Optional[str] = None, force_reconfigure: bool = False) -> bool:
    """
    配置matplotlib中文字体支持（全局函数）

    Args:
        font_name: 指定的字体名称，如果为None则自动选择最佳字体
        force_reconfigure: 是否强制重新配置

    Returns:
        bool: 配置是否成功
    """
    global _font_manager

    # 如果已经配置且不强制重新配置，直接返回
    if _font_manager.is_configured and not force_reconfigure:
        return True

    return _font_manager.configure_matplotlib_chinese_font(font_name)

def get_available_fonts() -> List[str]:
    """获取可用的中文字体列表"""
    global _font_manager
    return _font_manager.get_available_chinese_fonts()

def get_current_font() -> Optional[str]:
    """获取当前选择的中文字体"""
    global _font_manager
    return _font_manager.selected_font

def is_chinese_font_configured() -> bool:
    """检查中文字体是否已配置"""
    global _font_manager
    return _font_manager.is_configured

def test_chinese_display() -> bool:
    """测试中文显示是否正常"""
    global _font_manager
    return _font_manager.test_chinese_rendering()

def get_font_status() -> dict:
    """获取字体配置状态信息"""
    global _font_manager
    return _font_manager.get_font_info()

def print_font_status():
    """打印字体配置状态"""
    status = get_font_status()

    print("matplotlib中文字体配置状态:")
    print("-" * 40)
    print(f"配置状态: {'✓ 已配置' if status['is_configured'] else '✗ 未配置'}")

    if status['selected_font']:
        print(f"当前字体: {status['selected_font']}")

    print(f"可用字体数量: {len(status['available_fonts'])}")

    if status['available_fonts']:
        print("可用中文字体:")
        for i, font in enumerate(status['available_fonts'][:5], 1):
            marker = "★" if font == status['selected_font'] else " "
            print(f"  {marker} {font}")

        if len(status['available_fonts']) > 5:
            print(f"  ... 还有 {len(status['available_fonts']) - 5} 个字体")

    print(f"负号修复: {'✓ 已修复' if status['unicode_minus_fixed'] else '✗ 未修复'}")

    # 测试渲染
    render_ok = test_chinese_display()
    print(f"渲染测试: {'✓ 正常' if render_ok else '✗ 异常'}")

# 自动配置（模块导入时）
def auto_configure():
    """自动配置中文字体（在模块导入时调用）"""
    try:
        success = configure_chinese_font()
        if success:
            selected = get_current_font()
            print(f"✓ 自动配置中文字体成功: {selected}")
        else:
            print("✗ 自动配置中文字体失败，请手动配置")
    except Exception as e:
        print(f"✗ 自动配置中文字体时出错: {e}")

# 模块导入时自动配置
if __name__ != "__main__":
    # 只在作为模块导入时自动配置，避免在直接运行时重复配置
    auto_configure()

# 如果直接运行此模块，显示配置状态
if __name__ == "__main__":
    print("matplotlib中文字体配置模块")
    print("=" * 50)

    # 手动配置
    print("正在配置中文字体...")
    success = configure_chinese_font()

    if success:
        print("✓ 中文字体配置成功")
    else:
        print("✗ 中文字体配置失败")

    # 显示状态
    print_font_status()

    # 运行测试
    print("\n正在测试中文显示...")
    test_ok = test_chinese_display()
    print(f"中文显示测试: {'✓ 通过' if test_ok else '✗ 失败'}")