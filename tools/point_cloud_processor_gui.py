#!/usr/bin/env python3
"""
点云处理器GUI界面
基于PyQt5的高级点云处理工具图形界面
作者: Claude Code Assistant
"""

import sys
import os
import threading
import time
from pathlib import Path
from typing import Optional

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QLabel, QLineEdit, QPushButton, QFileDialog,
        QComboBox, QCheckBox, QSpinBox, QDoubleSpinBox, QTextEdit,
        QProgressBar, QGroupBox, QTabWidget, QSplitter, QMessageBox,
        QFrame, QScrollArea
    )
    from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
    from PyQt5.QtGui import QFont, QPixmap, QPalette
    HAS_PYQT5 = True
except ImportError:
    HAS_PYQT5 = False
    print("❌ PyQt5未安装，请运行: pip install PyQt5")

import numpy as np
from advanced_point_cloud_processor import IntensityAwarePointCloudProcessor

class PointCloudProcessorThread(QThread):
    """点云处理后台线程"""
    progress_updated = pyqtSignal(str)  # 进度文本更新
    progress_value = pyqtSignal(int)    # 进度条数值更新
    finished_processing = pyqtSignal(bool, dict)  # 处理完成信号
    
    def __init__(self, input_path, output_path, options):
        super().__init__()
        self.input_path = input_path
        self.output_path = output_path
        self.options = options
        self.processor = None
        
    def run(self):
        """后台处理线程"""
        try:
            self.progress_updated.emit("🚀 初始化点云处理器...")
            self.progress_value.emit(5)
            
            # 创建处理器
            self.processor = IntensityAwarePointCloudProcessor(self.input_path)
            
            self.progress_updated.emit("📁 加载PCD文件...")
            self.progress_value.emit(15)
            
            # 执行处理流水线
            success, mesh_stats = self.processor.process_full_pipeline(
                self.output_path, self.options
            )
            
            self.progress_value.emit(100)
            
            # 返回结果
            result_info = {
                'success': success,
                'mesh_stats': mesh_stats,
                'input_path': self.input_path,
                'output_path': self.output_path,
                'options': self.options
            }
            
            self.finished_processing.emit(success, result_info)
            
        except Exception as e:
            self.progress_updated.emit(f"❌ 处理失败: {str(e)}")
            self.finished_processing.emit(False, {'error': str(e)})

class PointCloudProcessorGUI(QMainWindow):
    """点云处理器主界面"""
    
    def __init__(self):
        super().__init__()
        self.current_thread = None
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("Advanced Point Cloud Processor - 高级点云处理工具")
        self.setGeometry(100, 100, 1200, 800)
        self.setMinimumSize(1000, 700)
        
        # 创建中心窗口和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # 左侧控制面板
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # 右侧结果面板
        result_panel = self.create_result_panel()
        splitter.addWidget(result_panel)
        
        # 设置分割器比例
        splitter.setSizes([400, 600])
        
        # 设置状态栏
        self.statusBar().showMessage("就绪")
        
    def create_control_panel(self):
        """创建左侧控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 文件选择区域
        file_group = self.create_file_selection_group()
        layout.addWidget(file_group)
        
        # 处理参数区域
        params_group = self.create_parameters_group()
        layout.addWidget(params_group)
        
        # 表面重建区域
        surface_group = self.create_surface_reconstruction_group()
        layout.addWidget(surface_group)
        
        # 控制按钮区域
        button_group = self.create_button_group()
        layout.addWidget(button_group)
        
        # 进度显示区域
        progress_group = self.create_progress_group()
        layout.addWidget(progress_group)
        
        layout.addStretch()
        return panel
        
    def create_file_selection_group(self):
        """创建文件选择组"""
        group = QGroupBox("文件设置")
        layout = QGridLayout(group)
        
        # 输入文件
        layout.addWidget(QLabel("输入PCD文件:"), 0, 0)
        self.input_path_edit = QLineEdit()
        self.input_path_edit.setPlaceholderText("选择输入的PCD点云文件...")
        layout.addWidget(self.input_path_edit, 0, 1)
        
        self.browse_input_btn = QPushButton("浏览...")
        self.browse_input_btn.clicked.connect(self.browse_input_file)
        layout.addWidget(self.browse_input_btn, 0, 2)
        
        # 输出文件
        layout.addWidget(QLabel("输出文件:"), 1, 0)
        self.output_path_edit = QLineEdit()
        self.output_path_edit.setPlaceholderText("自动生成输出文件名...")
        layout.addWidget(self.output_path_edit, 1, 1)
        
        self.browse_output_btn = QPushButton("浏览...")
        self.browse_output_btn.clicked.connect(self.browse_output_file)
        layout.addWidget(self.browse_output_btn, 1, 2)
        
        return group
        
    def create_parameters_group(self):
        """创建处理参数组"""
        group = QGroupBox("处理参数")
        layout = QGridLayout(group)
        
        # 强度过滤方法
        layout.addWidget(QLabel("强度过滤:"), 0, 0)
        self.intensity_method_combo = QComboBox()
        self.intensity_method_combo.addItems(['balanced', 'conservative', 'aggressive'])
        self.intensity_method_combo.setCurrentText('balanced')
        layout.addWidget(self.intensity_method_combo, 0, 1)
        
        # 离群点移除
        self.outlier_removal_cb = QCheckBox("离群点移除")
        self.outlier_removal_cb.setChecked(True)
        layout.addWidget(self.outlier_removal_cb, 1, 0)
        
        # 曲率增强
        self.curvature_enhance_cb = QCheckBox("曲率细节增强")
        self.curvature_enhance_cb.setChecked(True)
        layout.addWidget(self.curvature_enhance_cb, 1, 1)
        
        # 彩色点云生成
        self.colored_cloud_cb = QCheckBox("生成彩色点云")
        self.colored_cloud_cb.setChecked(True)
        layout.addWidget(self.colored_cloud_cb, 2, 0)
        
        return group
        
    def create_surface_reconstruction_group(self):
        """创建表面重建参数组"""
        group = QGroupBox("表面重建")
        layout = QGridLayout(group)
        
        # 启用表面重建
        self.surface_reconstruction_cb = QCheckBox("启用表面重建")
        self.surface_reconstruction_cb.stateChanged.connect(self.toggle_surface_options)
        layout.addWidget(self.surface_reconstruction_cb, 0, 0, 1, 2)
        
        # 重建方法
        layout.addWidget(QLabel("重建方法:"), 1, 0)
        self.reconstruction_method_combo = QComboBox()
        self.reconstruction_method_combo.addItems([
            'poisson', 'alpha_shape', 'ball_pivoting', 'delaunay_2d'
        ])
        layout.addWidget(self.reconstruction_method_combo, 1, 1)
        
        # Poisson深度
        layout.addWidget(QLabel("Poisson深度:"), 2, 0)
        self.poisson_depth_spin = QSpinBox()
        self.poisson_depth_spin.setRange(6, 12)
        self.poisson_depth_spin.setValue(9)
        layout.addWidget(self.poisson_depth_spin, 2, 1)
        
        # Alpha值
        layout.addWidget(QLabel("Alpha值:"), 3, 0)
        self.alpha_value_spin = QDoubleSpinBox()
        self.alpha_value_spin.setRange(0.01, 1.0)
        self.alpha_value_spin.setSingleStep(0.01)
        self.alpha_value_spin.setValue(0.03)
        self.alpha_value_spin.setDecimals(3)
        layout.addWidget(self.alpha_value_spin, 3, 1)
        
        # 密度阈值
        layout.addWidget(QLabel("密度阈值:"), 4, 0)
        self.density_threshold_spin = QDoubleSpinBox()
        self.density_threshold_spin.setRange(0.0, 1.0)
        self.density_threshold_spin.setSingleStep(0.1)
        self.density_threshold_spin.setValue(0.1)
        self.density_threshold_spin.setDecimals(2)
        layout.addWidget(self.density_threshold_spin, 4, 1)
        
        # 默认禁用表面重建选项
        self.toggle_surface_options(False)
        
        return group
        
    def create_button_group(self):
        """创建控制按钮组"""
        group = QGroupBox("操作")
        layout = QVBoxLayout(group)
        
        # 开始处理按钮
        self.process_btn = QPushButton("🚀 开始处理")
        self.process_btn.setMinimumHeight(40)
        self.process_btn.clicked.connect(self.start_processing)
        layout.addWidget(self.process_btn)
        
        # 停止处理按钮
        self.stop_btn = QPushButton("🛑 停止处理")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_processing)
        layout.addWidget(self.stop_btn)
        
        # 重置按钮
        self.reset_btn = QPushButton("🔄 重置参数")
        self.reset_btn.clicked.connect(self.reset_parameters)
        layout.addWidget(self.reset_btn)
        
        return group
        
    def create_progress_group(self):
        """创建进度显示组"""
        group = QGroupBox("处理进度")
        layout = QVBoxLayout(group)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)
        
        # 状态标签
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        return group
        
    def create_result_panel(self):
        """创建右侧结果面板"""
        panel = QTabWidget()
        
        # 处理日志标签页
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 10))
        panel.addTab(self.log_text, "处理日志")
        
        # 结果统计标签页
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        panel.addTab(self.result_text, "结果统计")
        
        # 使用说明标签页
        help_text = QTextEdit()
        help_text.setReadOnly(True)
        help_text.setHtml(self.get_help_text())
        panel.addTab(help_text, "使用说明")
        
        return panel
        
    def setup_connections(self):
        """设置信号连接"""
        # 输入文件变化时自动生成输出文件名
        self.input_path_edit.textChanged.connect(self.auto_generate_output_path)
        
    def toggle_surface_options(self, enabled):
        """切换表面重建选项的启用状态"""
        if isinstance(enabled, int):
            enabled = bool(enabled)
            
        self.reconstruction_method_combo.setEnabled(enabled)
        self.poisson_depth_spin.setEnabled(enabled)
        self.alpha_value_spin.setEnabled(enabled)
        self.density_threshold_spin.setEnabled(enabled)
        
    def browse_input_file(self):
        """浏览选择输入文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择输入PCD文件", "", "PCD Files (*.pcd);;All Files (*)"
        )
        if file_path:
            self.input_path_edit.setText(file_path)
            
    def browse_output_file(self):
        """浏览选择输出文件"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "选择输出PCD文件", "", "PCD Files (*.pcd);;All Files (*)"
        )
        if file_path:
            self.output_path_edit.setText(file_path)
            
    def auto_generate_output_path(self):
        """自动生成输出文件路径"""
        input_path = self.input_path_edit.text().strip()
        if input_path and os.path.exists(input_path):
            input_file = Path(input_path)
            output_file = input_file.parent / f"enhanced_{input_file.name}"
            self.output_path_edit.setText(str(output_file))
            
    def get_processing_options(self):
        """获取处理选项"""
        return {
            'outlier_removal': self.outlier_removal_cb.isChecked(),
            'intensity_filtering': self.intensity_method_combo.currentText(),
            'curvature_enhancement': self.curvature_enhance_cb.isChecked(),
            'create_colored_cloud': self.colored_cloud_cb.isChecked(),
            'surface_reconstruction': self.surface_reconstruction_cb.isChecked(),
            'reconstruction_method': self.reconstruction_method_combo.currentText(),
            'poisson_depth': self.poisson_depth_spin.value(),
            'alpha_value': self.alpha_value_spin.value(),
            'density_threshold': self.density_threshold_spin.value()
        }
        
    def start_processing(self):
        """开始处理"""
        # 验证输入
        input_path = self.input_path_edit.text().strip()
        if not input_path or not os.path.exists(input_path):
            QMessageBox.warning(self, "错误", "请选择有效的输入PCD文件！")
            return
            
        output_path = self.output_path_edit.text().strip()
        if not output_path:
            QMessageBox.warning(self, "错误", "请设置输出文件路径！")
            return
            
        # 获取处理选项
        options = self.get_processing_options()
        
        # 更新界面状态
        self.process_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setValue(0)
        self.status_label.setText("正在处理...")
        self.log_text.clear()
        
        # 启动后台处理线程
        self.current_thread = PointCloudProcessorThread(input_path, output_path, options)
        self.current_thread.progress_updated.connect(self.update_progress_text)
        self.current_thread.progress_value.connect(self.update_progress_value)
        self.current_thread.finished_processing.connect(self.processing_finished)
        self.current_thread.start()
        
        self.log_text.append("🚀 开始点云处理...")
        self.log_text.append(f"📁 输入文件: {input_path}")
        self.log_text.append(f"📁 输出文件: {output_path}")
        self.log_text.append(f"⚙️ 处理选项: {options}")
        self.log_text.append("-" * 50)
        
    def stop_processing(self):
        """停止处理"""
        if self.current_thread and self.current_thread.isRunning():
            self.current_thread.terminate()
            self.current_thread.wait()
            
        self.process_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("已停止")
        self.log_text.append("🛑 处理已停止")
        
    def reset_parameters(self):
        """重置参数"""
        self.intensity_method_combo.setCurrentText('balanced')
        self.outlier_removal_cb.setChecked(True)
        self.curvature_enhance_cb.setChecked(True)
        self.colored_cloud_cb.setChecked(True)
        self.surface_reconstruction_cb.setChecked(False)
        self.reconstruction_method_combo.setCurrentIndex(0)
        self.poisson_depth_spin.setValue(9)
        self.alpha_value_spin.setValue(0.03)
        self.density_threshold_spin.setValue(0.1)
        self.progress_bar.setValue(0)
        self.status_label.setText("参数已重置")
        
    def update_progress_text(self, text):
        """更新进度文本"""
        self.status_label.setText(text)
        self.log_text.append(text)
        self.log_text.moveCursor(self.log_text.textCursor().End)
        
    def update_progress_value(self, value):
        """更新进度条数值"""
        self.progress_bar.setValue(value)
        
    def processing_finished(self, success, result_info):
        """处理完成"""
        # 重置界面状态
        self.process_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        
        if success:
            self.status_label.setText("✅ 处理完成")
            self.progress_bar.setValue(100)
            self.log_text.append("✅ 点云处理完成！")
            
            # 显示结果统计
            self.show_processing_results(result_info)
            
            # 显示成功消息
            QMessageBox.information(self, "成功", "点云处理完成！\n\n请查看结果统计标签页。")
            
        else:
            self.status_label.setText("❌ 处理失败")
            error_msg = result_info.get('error', '未知错误')
            self.log_text.append(f"❌ 处理失败: {error_msg}")
            QMessageBox.critical(self, "错误", f"处理失败：\n{error_msg}")
            
    def show_processing_results(self, result_info):
        """显示处理结果统计"""
        mesh_stats = result_info.get('mesh_stats')
        options = result_info.get('options', {})
        
        result_text = "# 点云处理结果统计\n\n"
        result_text += f"## 文件信息\n"
        result_text += f"- 输入文件: {result_info['input_path']}\n"
        result_text += f"- 输出文件: {result_info['output_path']}\n\n"
        
        result_text += f"## 处理参数\n"
        result_text += f"- 强度过滤: {options.get('intensity_filtering', 'N/A')}\n"
        result_text += f"- 离群点移除: {'是' if options.get('outlier_removal') else '否'}\n"
        result_text += f"- 曲率增强: {'是' if options.get('curvature_enhancement') else '否'}\n"
        result_text += f"- 表面重建: {'是' if options.get('surface_reconstruction') else '否'}\n"
        
        if options.get('surface_reconstruction'):
            result_text += f"- 重建方法: {options.get('reconstruction_method', 'N/A')}\n"
            
        result_text += f"\n## 生成的文件\n"
        output_path = Path(result_info['output_path'])
        result_text += f"- 增强点云: {output_path}\n"
        result_text += f"- 彩色点云: {str(output_path).replace('.pcd', '_colored.pcd')}\n"
        
        if mesh_stats:
            mesh_file = str(output_path).replace('.pcd', f"_mesh_{options.get('reconstruction_method', 'unknown')}.ply")
            result_text += f"- 网格模型: {mesh_file}\n"
            result_text += f"\n## 网格统计\n"
            result_text += f"- 顶点数: {mesh_stats.get('vertex_count', 0):,}\n"
            result_text += f"- 三角面数: {mesh_stats.get('triangle_count', 0):,}\n"
            result_text += f"- 表面积: {mesh_stats.get('total_surface_area', 0):.3f}\n"
            result_text += f"- 封闭性: {'是' if mesh_stats.get('is_watertight') else '否'}\n"
        
        result_text += f"\n## 使用建议\n"
        result_text += f"- RViz查看: `python3 tools/rviz_map_viewer.py {result_info['output_path']}`\n"
        result_text += f"- CloudCompare查看网格: `cloudcompare <网格文件>`\n"
        
        self.result_text.setPlainText(result_text)
        
    def get_help_text(self):
        """获取帮助文本"""
        return """
        <h2>Advanced Point Cloud Processor GUI</h2>
        <h3>使用指南</h3>
        <p><b>1. 文件设置</b></p>
        <ul>
            <li>点击"浏览..."选择输入PCD文件</li>
            <li>输出文件会自动生成，也可手动修改</li>
        </ul>
        
        <p><b>2. 处理参数</b></p>
        <ul>
            <li><b>强度过滤</b>：根据激光反射强度优化点云质量</li>
            <li><b>离群点移除</b>：去除噪声和异常点</li>
            <li><b>曲率细节增强</b>：在几何特征丰富区域增加采样密度</li>
            <li><b>彩色点云</b>：生成基于强度的彩色可视化文件</li>
        </ul>
        
        <p><b>3. 表面重建</b></p>
        <ul>
            <li><b>Poisson</b>：适用于封闭表面</li>
            <li><b>Alpha Shape</b>：适用于复杂几何结构</li>
            <li><b>Ball Pivoting</b>：适用于均匀采样点云</li>
            <li><b>Delaunay 2D</b>：适用于相对平坦表面</li>
        </ul>
        
        <p><b>4. 处理结果</b></p>
        <ul>
            <li>增强点云：优化后的主要点云文件</li>
            <li>彩色点云：用于可视化的彩色文件</li>
            <li>网格模型：表面重建生成的PLY文件</li>
        </ul>
        
        <p><b>5. 常见问题</b></p>
        <ul>
            <li>处理时间取决于点云规模和重建方法</li>
            <li>Ball Pivoting方法处理时间较长</li>
            <li>Alpha Shape适合复杂场景快速重建</li>
            <li>Poisson重建可能因数据质量失败</li>
        </ul>
        """

def main():
    """主函数"""
    if not HAS_PYQT5:
        print("请安装PyQt5: pip install PyQt5")
        return
        
    app = QApplication(sys.argv)
    app.setApplicationName("Advanced Point Cloud Processor")
    
    # 设置应用程序样式
    app.setStyleSheet("""
        QMainWindow {
            background-color: #f0f0f0;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #cccccc;
            border-radius: 8px;
            margin-top: 10px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 10px 0 10px;
        }
        QPushButton {
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 8px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #45a049;
        }
        QPushButton:disabled {
            background-color: #cccccc;
            color: #666666;
        }
        QProgressBar {
            border: 2px solid #cccccc;
            border-radius: 5px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: #4CAF50;
            border-radius: 3px;
        }
    """)
    
    window = PointCloudProcessorGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()