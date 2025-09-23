#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态对象过滤器可视化工具

该工具提供实时的动态对象过滤器可视化功能，包括：
- 实时点云对比显示（原始vs过滤后）
- 过滤统计图表显示
- 交互式参数调优界面
- 3D可视化支持
- 性能监控
- 数据录制和回放功能

作者: SLAM系统
日期: 2024
"""

import sys
import os
import threading
import time
import json
import pickle
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Dict, List, Tuple, Any, Callable
from collections import deque
from datetime import datetime, timedelta

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.dates as mdates

# 配置matplotlib基本参数和中文字体支持
import matplotlib.font_manager as fm

# 导入中文字体配置模块
try:
    from chinese_font_config import configure_chinese_font, print_font_status
    # 配置中文字体支持
    font_configured = configure_chinese_font()
    if font_configured:
        print("✓ 中文字体配置成功")
    else:
        print("✗ 中文字体配置失败，中文标签可能无法正常显示")
except ImportError:
    print("警告: 中文字体配置模块不可用，使用默认字体设置")
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    plt.rcParams['font.size'] = 10

# 定义标签文本
LABELS = {
    'filter_ratio': '过滤比例',
    'processing_time': '处理时间(ms)',
    'memory_usage': '内存使用(MB)',
    'point_count': '点云数量',
    'time': '时间',
    'ratio': '比例',
    'ms': 'ms',
    'mb': 'MB'
}

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from interface.msg import FilterStats
    import sensor_msgs_py.point_cloud2 as pc2
    ROS2_AVAILABLE = True
except ImportError:
    print("警告: ROS2不可用，仅提供数据回放功能")
    ROS2_AVAILABLE = False

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    print("警告: Open3D不可用，禁用3D可视化功能")
    OPEN3D_AVAILABLE = False


@dataclass
class FilterConfig:
    """动态过滤器配置参数"""
    motion_threshold: float = 0.1
    history_size: int = 5
    stability_threshold: float = 0.8
    max_time_diff: float = 0.5
    search_radius: float = 0.2
    min_neighbors: int = 10
    normal_consistency_thresh: float = 0.8
    density_ratio_thresh: float = 0.5
    downsample_ratio: int = 2
    max_points_per_frame: int = 50000
    voxel_size_base: float = 0.01


@dataclass
class VisualizationData:
    """可视化数据结构"""
    timestamp: float
    raw_points: Optional[np.ndarray] = None
    filtered_points: Optional[np.ndarray] = None
    total_points: int = 0
    dynamic_points: int = 0
    static_points: int = 0
    filter_ratio: float = 0.0
    processing_time_ms: float = 0.0
    memory_usage_mb: float = 0.0


class DataRecorder:
    """数据录制器"""

    def __init__(self, output_dir: str = "recorded_data"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.recording = False
        self.data_buffer: List[VisualizationData] = []
        self.start_time: Optional[float] = None

    def start_recording(self) -> None:
        """开始录制"""
        self.recording = True
        self.start_time = time.time()
        self.data_buffer.clear()
        print("开始录制数据...")

    def stop_recording(self) -> str:
        """停止录制并保存数据"""
        if not self.recording:
            return ""

        self.recording = False
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"filter_data_{timestamp}.pkl"

        with open(filename, 'wb') as f:
            pickle.dump(self.data_buffer, f)

        print(f"数据已保存到: {filename}")
        return str(filename)

    def add_data(self, data: VisualizationData) -> None:
        """添加数据到缓冲区"""
        if self.recording:
            self.data_buffer.append(data)

    def load_data(self, filename: str) -> List[VisualizationData]:
        """加载录制的数据"""
        with open(filename, 'rb') as f:
            return pickle.load(f)


class ROS2Subscriber(Node if ROS2_AVAILABLE else object):
    """ROS2订阅器"""

    def __init__(self, data_callback: Callable[[VisualizationData], None]):
        if not ROS2_AVAILABLE:
            return

        super().__init__('dynamic_filter_visualizer')
        self.data_callback = data_callback

        # QoS设置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 订阅器
        self.raw_cloud_sub = self.create_subscription(
            PointCloud2,
            '/slam/body_cloud',
            self.raw_cloud_callback,
            qos_profile
        )

        self.filtered_cloud_sub = self.create_subscription(
            PointCloud2,
            '/localizer/filtered_cloud',
            self.filtered_cloud_callback,
            qos_profile
        )

        self.stats_sub = self.create_subscription(
            FilterStats,
            '/localizer/filter_stats',
            self.stats_callback,
            qos_profile
        )

        # 数据缓存
        self.current_data = VisualizationData(timestamp=time.time())
        self.data_lock = threading.Lock()

        self.get_logger().info("动态过滤器可视化器节点已启动")

    def raw_cloud_callback(self, msg: PointCloud2) -> None:
        """原始点云回调"""
        try:
            points = self.pointcloud2_to_array(msg)
            with self.data_lock:
                self.current_data.raw_points = points
                self.current_data.timestamp = time.time()
                self._check_and_emit_data()
        except Exception as e:
            self.get_logger().error(f"处理原始点云时出错: {e}")

    def filtered_cloud_callback(self, msg: PointCloud2) -> None:
        """过滤后点云回调"""
        try:
            points = self.pointcloud2_to_array(msg)
            with self.data_lock:
                self.current_data.filtered_points = points
                self.current_data.timestamp = time.time()
                self._check_and_emit_data()
        except Exception as e:
            self.get_logger().error(f"处理过滤点云时出错: {e}")

    def stats_callback(self, msg: FilterStats) -> None:
        """统计信息回调"""
        with self.data_lock:
            self.current_data.total_points = msg.total_points
            self.current_data.dynamic_points = msg.dynamic_points
            self.current_data.static_points = msg.static_points
            self.current_data.filter_ratio = msg.filter_ratio
            self.current_data.processing_time_ms = msg.processing_time_ms
            self.current_data.memory_usage_mb = msg.memory_usage_mb
            self.current_data.timestamp = time.time()
            self._check_and_emit_data()

    def _check_and_emit_data(self) -> None:
        """检查数据完整性并发送"""
        # 如果有完整数据则发送
        if (self.current_data.raw_points is not None or
            self.current_data.filtered_points is not None):
            self.data_callback(self.current_data)

    @staticmethod
    def pointcloud2_to_array(cloud_msg: PointCloud2) -> np.ndarray:
        """将PointCloud2消息转换为numpy数组"""
        points_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list) if points_list else np.empty((0, 3))


class PerformanceMonitor:
    """性能监控器"""

    def __init__(self, history_size: int = 100):
        self.history_size = history_size
        self.reset()

    def reset(self) -> None:
        """重置监控数据"""
        self.timestamps = deque(maxlen=self.history_size)
        self.processing_times = deque(maxlen=self.history_size)
        self.memory_usage = deque(maxlen=self.history_size)
        self.filter_ratios = deque(maxlen=self.history_size)
        self.point_counts = deque(maxlen=self.history_size)

    def add_data(self, data: VisualizationData) -> None:
        """添加性能数据"""
        self.timestamps.append(datetime.fromtimestamp(data.timestamp))
        self.processing_times.append(data.processing_time_ms)
        self.memory_usage.append(data.memory_usage_mb)
        self.filter_ratios.append(data.filter_ratio)
        self.point_counts.append(data.total_points)

    def get_average_processing_time(self) -> float:
        """获取平均处理时间"""
        return np.mean(self.processing_times) if self.processing_times else 0.0

    def get_max_memory_usage(self) -> float:
        """获取最大内存使用量"""
        return max(self.memory_usage) if self.memory_usage else 0.0

    def get_average_filter_ratio(self) -> float:
        """获取平均过滤比例"""
        return np.mean(self.filter_ratios) if self.filter_ratios else 0.0


class Open3DVisualizer:
    """Open3D 3D可视化器"""

    def __init__(self):
        if not OPEN3D_AVAILABLE:
            raise ImportError("Open3D不可用")

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="动态对象过滤器 - 3D可视化", width=800, height=600)

        # 点云对象
        self.raw_cloud = o3d.geometry.PointCloud()
        self.filtered_cloud = o3d.geometry.PointCloud()

        # 添加到可视化器
        self.vis.add_geometry(self.raw_cloud)
        self.vis.add_geometry(self.filtered_cloud)

        # 设置渲染选项
        render_option = self.vis.get_render_option()
        render_option.point_size = 2.0
        render_option.background_color = np.array([0.1, 0.1, 0.1])

        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()

    def update_pointclouds(self, data: VisualizationData) -> None:
        """更新点云显示"""
        try:
            # 更新原始点云（红色）
            if data.raw_points is not None and len(data.raw_points) > 0:
                self.raw_cloud.points = o3d.utility.Vector3dVector(data.raw_points)
                self.raw_cloud.colors = o3d.utility.Vector3dVector(
                    np.tile([1.0, 0.0, 0.0], (len(data.raw_points), 1))
                )

            # 更新过滤后点云（绿色）
            if data.filtered_points is not None and len(data.filtered_points) > 0:
                # 稍微偏移以便区分
                offset_points = data.filtered_points + np.array([0, 0, 0.1])
                self.filtered_cloud.points = o3d.utility.Vector3dVector(offset_points)
                self.filtered_cloud.colors = o3d.utility.Vector3dVector(
                    np.tile([0.0, 1.0, 0.0], (len(data.filtered_points), 1))
                )

        except Exception as e:
            print(f"更新3D可视化时出错: {e}")

    def _update_loop(self) -> None:
        """更新循环"""
        while self.running:
            try:
                self.vis.update_geometry(self.raw_cloud)
                self.vis.update_geometry(self.filtered_cloud)
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.1)
            except:
                break

    def close(self) -> None:
        """关闭可视化器"""
        self.running = False
        if hasattr(self, 'vis'):
            self.vis.destroy_window()


class DynamicFilterVisualizer:
    """动态对象过滤器主可视化器"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("动态对象过滤器可视化工具")
        self.root.geometry("1400x900")

        # 配置中文字体支持
        try:
            # 尝试设置中文字体
            import tkinter.font as tkFont
            self.chinese_font = tkFont.Font(family="Noto Sans CJK SC", size=10)
            self.root.option_add("*Font", self.chinese_font)
        except:
            # 如果中文字体不可用，使用默认字体
            print("警告: 无法加载中文字体，将使用默认字体")

        # 数据和组件
        self.current_data = VisualizationData(timestamp=time.time())
        self.config = FilterConfig()
        self.recorder = DataRecorder()
        self.performance_monitor = PerformanceMonitor()

        # ROS2订阅器
        self.ros_subscriber: Optional[ROS2Subscriber] = None
        self.ros_thread: Optional[threading.Thread] = None

        # Open3D可视化器
        self.open3d_visualizer: Optional[Open3DVisualizer] = None

        # GUI组件
        self.setup_gui()

        # 数据回放
        self.replay_data: List[VisualizationData] = []
        self.replay_index = 0
        self.replay_running = False

        # 启动ROS2
        if ROS2_AVAILABLE:
            self.start_ros2()

    def setup_gui(self) -> None:
        """设置GUI界面"""
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 左侧面板（控制和统计）
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # 右侧面板（图表显示）
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 设置左侧面板
        self.setup_control_panel(left_panel)

        # 设置右侧图表
        self.setup_charts(right_panel)

    def setup_control_panel(self, parent: ttk.Frame) -> None:
        """设置控制面板"""
        # 连接状态
        status_frame = ttk.LabelFrame(parent, text="连接状态", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))

        self.connection_label = ttk.Label(status_frame, text="离线模式")
        self.connection_label.pack()

        # 录制控制
        record_frame = ttk.LabelFrame(parent, text="数据录制", padding=10)
        record_frame.pack(fill=tk.X, pady=(0, 10))

        self.record_button = ttk.Button(record_frame, text="开始录制",
                                       command=self.toggle_recording)
        self.record_button.pack(fill=tk.X, pady=(0, 5))

        ttk.Button(record_frame, text="加载数据",
                  command=self.load_recorded_data).pack(fill=tk.X, pady=(0, 5))

        ttk.Button(record_frame, text="导出数据",
                  command=self.export_data).pack(fill=tk.X)

        # 回放控制
        replay_frame = ttk.LabelFrame(parent, text="数据回放", padding=10)
        replay_frame.pack(fill=tk.X, pady=(0, 10))

        self.replay_button = ttk.Button(replay_frame, text="开始回放",
                                       command=self.toggle_replay)
        self.replay_button.pack(fill=tk.X, pady=(0, 5))

        self.replay_progress = ttk.Progressbar(replay_frame, mode='determinate')
        self.replay_progress.pack(fill=tk.X, pady=(0, 5))

        self.replay_speed_scale = ttk.Scale(replay_frame, from_=0.1, to=5.0,
                                           orient=tk.HORIZONTAL, value=1.0)
        self.replay_speed_scale.pack(fill=tk.X)
        ttk.Label(replay_frame, text="回放速度").pack()

        # 参数调优
        self.setup_parameter_tuning(parent)

        # 实时统计
        self.setup_statistics_display(parent)

        # 3D可视化控制
        if OPEN3D_AVAILABLE:
            vis3d_frame = ttk.LabelFrame(parent, text="3D可视化", padding=10)
            vis3d_frame.pack(fill=tk.X, pady=(0, 10))

            ttk.Button(vis3d_frame, text="打开3D视图",
                      command=self.toggle_3d_visualization).pack(fill=tk.X)

    def setup_parameter_tuning(self, parent: ttk.Frame) -> None:
        """设置参数调优面板"""
        params_frame = ttk.LabelFrame(parent, text="参数调优", padding=10)
        params_frame.pack(fill=tk.X, pady=(0, 10))

        # 创建滚动框架
        canvas = tk.Canvas(params_frame, height=200)
        scrollbar = ttk.Scrollbar(params_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # 参数控件
        self.param_vars = {}
        params = [
            ("运动阈值", "motion_threshold", 0.01, 1.0, 0.01),
            ("历史帧数", "history_size", 1, 20, 1),
            ("稳定性阈值", "stability_threshold", 0.0, 1.0, 0.01),
            ("最大时间差", "max_time_diff", 0.1, 2.0, 0.1),
            ("搜索半径", "search_radius", 0.05, 1.0, 0.01),
            ("最小邻居数", "min_neighbors", 1, 50, 1),
            ("法向量一致性", "normal_consistency_thresh", 0.0, 1.0, 0.01),
            ("密度比值阈值", "density_ratio_thresh", 0.0, 1.0, 0.01),
        ]

        for i, (label, attr, min_val, max_val, resolution) in enumerate(params):
            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill=tk.X, pady=2)

            ttk.Label(frame, text=label, width=15).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=getattr(self.config, attr))
            self.param_vars[attr] = var

            scale = ttk.Scale(frame, from_=min_val, to=max_val,
                             orient=tk.HORIZONTAL, variable=var,
                             command=lambda v, a=attr: self.update_parameter(a, v))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            value_label = ttk.Label(frame, text=f"{var.get():.3f}", width=8)
            value_label.pack(side=tk.RIGHT)

            # 绑定更新标签
            var.trace_add('write', lambda *args, l=value_label, v=var:
                         l.config(text=f"{v.get():.3f}"))

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def setup_statistics_display(self, parent: ttk.Frame) -> None:
        """设置统计信息显示"""
        stats_frame = ttk.LabelFrame(parent, text="实时统计", padding=10)
        stats_frame.pack(fill=tk.X, pady=(0, 10))

        self.stats_labels = {}
        stats_items = [
            ("总点数", "total_points"),
            ("动态点数", "dynamic_points"),
            ("静态点数", "static_points"),
            ("过滤比例", "filter_ratio"),
            ("处理时间(ms)", "processing_time_ms"),
            ("内存使用(MB)", "memory_usage_mb"),
        ]

        for label, key in stats_items:
            frame = ttk.Frame(stats_frame)
            frame.pack(fill=tk.X, pady=1)

            ttk.Label(frame, text=f"{label}:", width=12).pack(side=tk.LEFT)

            value_label = ttk.Label(frame, text="0", width=10)
            value_label.pack(side=tk.RIGHT)

            self.stats_labels[key] = value_label

    def setup_charts(self, parent: ttk.Frame) -> None:
        """设置图表显示"""
        # 创建notebook用于多个图表标签页
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True)

        # 实时统计图表
        self.setup_realtime_charts(notebook)

        # 性能监控图表
        self.setup_performance_charts(notebook)

        # 点云对比图表
        self.setup_pointcloud_charts(notebook)

    def setup_realtime_charts(self, notebook: ttk.Notebook) -> None:
        """设置实时统计图表"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="实时统计")

        # 创建matplotlib图表
        self.fig_realtime, ((self.ax_points, self.ax_ratio),
                           (self.ax_time, self.ax_memory)) = plt.subplots(2, 2, figsize=(12, 8))

        self.fig_realtime.suptitle("动态过滤器实时统计", fontsize=14)

        # 配置子图
        self.ax_points.set_title("点云统计")
        self.ax_points.set_ylabel("点数")
        self.ax_points.legend()

        self.ax_ratio.set_title("过滤比例")
        self.ax_ratio.set_ylabel("比例")
        self.ax_ratio.set_ylim(0, 1)

        self.ax_time.set_title("处理时间")
        self.ax_time.set_ylabel("时间 (ms)")

        self.ax_memory.set_title("内存使用")
        self.ax_memory.set_ylabel("内存 (MB)")

        # 将图表嵌入tkinter
        canvas_realtime = FigureCanvasTkAgg(self.fig_realtime, frame)
        canvas_realtime.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 启动动画
        self.anim_realtime = FuncAnimation(self.fig_realtime, self.update_realtime_charts,
                                          interval=1000, blit=False)

    def setup_performance_charts(self, notebook: ttk.Notebook) -> None:
        """设置性能监控图表"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="性能监控")

        self.fig_performance, (self.ax_perf_time, self.ax_perf_memory) = plt.subplots(2, 1, figsize=(12, 8))

        self.fig_performance.suptitle("性能监控", fontsize=14)

        self.ax_perf_time.set_title("处理时间趋势")
        self.ax_perf_time.set_ylabel("时间 (ms)")

        self.ax_perf_memory.set_title("内存使用趋势")
        self.ax_perf_memory.set_ylabel("内存 (MB)")

        canvas_performance = FigureCanvasTkAgg(self.fig_performance, frame)
        canvas_performance.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.anim_performance = FuncAnimation(self.fig_performance, self.update_performance_charts,
                                            interval=2000, blit=False)

    def setup_pointcloud_charts(self, notebook: ttk.Notebook) -> None:
        """设置点云对比图表"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="点云对比")

        self.fig_pointcloud, (self.ax_raw_pc, self.ax_filtered_pc) = plt.subplots(1, 2, figsize=(12, 6))

        self.fig_pointcloud.suptitle("点云对比 (X-Y平面投影)", fontsize=14)

        self.ax_raw_pc.set_title("原始点云")
        self.ax_raw_pc.set_xlabel("X (m)")
        self.ax_raw_pc.set_ylabel("Y (m)")
        self.ax_raw_pc.set_aspect('equal')

        self.ax_filtered_pc.set_title("过滤后点云")
        self.ax_filtered_pc.set_xlabel("X (m)")
        self.ax_filtered_pc.set_ylabel("Y (m)")
        self.ax_filtered_pc.set_aspect('equal')

        canvas_pointcloud = FigureCanvasTkAgg(self.fig_pointcloud, frame)
        canvas_pointcloud.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.anim_pointcloud = FuncAnimation(self.fig_pointcloud, self.update_pointcloud_charts,
                                           interval=500, blit=False)

    def start_ros2(self) -> None:
        """启动ROS2订阅"""
        if not ROS2_AVAILABLE:
            return

        try:
            rclpy.init()
            self.ros_subscriber = ROS2Subscriber(self.on_data_received)

            # 在单独线程中运行ROS2
            self.ros_thread = threading.Thread(
                target=lambda: rclpy.spin(self.ros_subscriber),
                daemon=True
            )
            self.ros_thread.start()

            self.connection_label.config(text="ROS2已连接")
            print("ROS2订阅器已启动")

        except Exception as e:
            print(f"启动ROS2时出错: {e}")
            self.connection_label.config(text="ROS2连接失败")

    def on_data_received(self, data: VisualizationData) -> None:
        """数据接收回调"""
        self.current_data = data

        # 添加到性能监控
        self.performance_monitor.add_data(data)

        # 添加到录制器
        self.recorder.add_data(data)

        # 更新统计显示
        self.update_statistics_display()

        # 更新3D可视化
        if self.open3d_visualizer:
            self.open3d_visualizer.update_pointclouds(data)

    def update_statistics_display(self) -> None:
        """更新统计信息显示"""
        data = self.current_data

        self.stats_labels["total_points"].config(text=str(data.total_points))
        self.stats_labels["dynamic_points"].config(text=str(data.dynamic_points))
        self.stats_labels["static_points"].config(text=str(data.static_points))
        self.stats_labels["filter_ratio"].config(text=f"{data.filter_ratio:.3f}")
        self.stats_labels["processing_time_ms"].config(text=f"{data.processing_time_ms:.2f}")
        self.stats_labels["memory_usage_mb"].config(text=f"{data.memory_usage_mb:.2f}")

    def update_realtime_charts(self, frame) -> None:
        """更新实时统计图表"""
        if not hasattr(self, 'performance_monitor') or not self.performance_monitor.timestamps:
            return

        try:
            # 清除旧数据
            for ax in [self.ax_points, self.ax_ratio, self.ax_time, self.ax_memory]:
                ax.clear()

            times = list(self.performance_monitor.timestamps)

            # 点云统计
            self.ax_points.plot(times, list(self.performance_monitor.point_counts), 'b-', label='总点数')
            self.ax_points.set_title("点云统计")
            self.ax_points.set_ylabel("点数")
            self.ax_points.legend()
            self.ax_points.tick_params(axis='x', rotation=45)

            # 过滤比例
            self.ax_ratio.plot(times, list(self.performance_monitor.filter_ratios), 'r-')
            self.ax_ratio.set_title("过滤比例")
            self.ax_ratio.set_ylabel("比例")
            self.ax_ratio.set_ylim(0, 1)
            self.ax_ratio.tick_params(axis='x', rotation=45)

            # 处理时间
            self.ax_time.plot(times, list(self.performance_monitor.processing_times), 'g-')
            self.ax_time.set_title("处理时间")
            self.ax_time.set_ylabel("时间 (ms)")
            self.ax_time.tick_params(axis='x', rotation=45)

            # 内存使用
            self.ax_memory.plot(times, list(self.performance_monitor.memory_usage), 'm-')
            self.ax_memory.set_title("内存使用")
            self.ax_memory.set_ylabel("内存 (MB)")
            self.ax_memory.tick_params(axis='x', rotation=45)

            # 格式化时间轴
            for ax in [self.ax_points, self.ax_ratio, self.ax_time, self.ax_memory]:
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                ax.xaxis.set_major_locator(mdates.SecondLocator(interval=30))

            plt.tight_layout()

        except Exception as e:
            print(f"更新实时图表时出错: {e}")

    def update_performance_charts(self, frame) -> None:
        """更新性能监控图表"""
        if not hasattr(self, 'performance_monitor') or not self.performance_monitor.timestamps:
            return

        try:
            self.ax_perf_time.clear()
            self.ax_perf_memory.clear()

            times = list(self.performance_monitor.timestamps)

            # 处理时间趋势
            self.ax_perf_time.plot(times, list(self.performance_monitor.processing_times), 'b-')
            avg_time = self.performance_monitor.get_average_processing_time()
            self.ax_perf_time.axhline(y=avg_time, color='r', linestyle='--',
                                     label=f'平均: {avg_time:.2f}ms')
            self.ax_perf_time.set_title("处理时间趋势")
            self.ax_perf_time.set_ylabel("时间 (ms)")
            self.ax_perf_time.legend()
            self.ax_perf_time.tick_params(axis='x', rotation=45)

            # 内存使用趋势
            self.ax_perf_memory.plot(times, list(self.performance_monitor.memory_usage), 'g-')
            max_memory = self.performance_monitor.get_max_memory_usage()
            self.ax_perf_memory.axhline(y=max_memory, color='r', linestyle='--',
                                       label=f'最大: {max_memory:.2f}MB')
            self.ax_perf_memory.set_title("内存使用趋势")
            self.ax_perf_memory.set_ylabel("内存 (MB)")
            self.ax_perf_memory.legend()
            self.ax_perf_memory.tick_params(axis='x', rotation=45)

            # 格式化时间轴
            for ax in [self.ax_perf_time, self.ax_perf_memory]:
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                ax.xaxis.set_major_locator(mdates.SecondLocator(interval=60))

            plt.tight_layout()

        except Exception as e:
            print(f"更新性能图表时出错: {e}")

    def update_pointcloud_charts(self, frame) -> None:
        """更新点云对比图表"""
        data = self.current_data

        try:
            self.ax_raw_pc.clear()
            self.ax_filtered_pc.clear()

            # 原始点云 (X-Y投影)
            if data.raw_points is not None and len(data.raw_points) > 0:
                # 降采样显示以提高性能
                indices = np.random.choice(len(data.raw_points),
                                         min(5000, len(data.raw_points)),
                                         replace=False)
                sample_points = data.raw_points[indices]

                self.ax_raw_pc.scatter(sample_points[:, 0], sample_points[:, 1],
                                      c='red', s=1, alpha=0.6)
                self.ax_raw_pc.set_title(f"原始点云 ({len(data.raw_points)} 点)")
            else:
                self.ax_raw_pc.set_title("原始点云 (无数据)")

            # 过滤后点云 (X-Y投影)
            if data.filtered_points is not None and len(data.filtered_points) > 0:
                # 降采样显示以提高性能
                indices = np.random.choice(len(data.filtered_points),
                                         min(5000, len(data.filtered_points)),
                                         replace=False)
                sample_points = data.filtered_points[indices]

                self.ax_filtered_pc.scatter(sample_points[:, 0], sample_points[:, 1],
                                           c='green', s=1, alpha=0.6)
                self.ax_filtered_pc.set_title(f"过滤后点云 ({len(data.filtered_points)} 点)")
            else:
                self.ax_filtered_pc.set_title("过滤后点云 (无数据)")

            # 设置坐标轴
            for ax in [self.ax_raw_pc, self.ax_filtered_pc]:
                ax.set_xlabel("X (m)")
                ax.set_ylabel("Y (m)")
                ax.set_aspect('equal')
                ax.grid(True, alpha=0.3)

        except Exception as e:
            print(f"更新点云图表时出错: {e}")

    def update_parameter(self, param_name: str, value: str) -> None:
        """更新参数"""
        try:
            val = float(value)
            setattr(self.config, param_name, val)

            # 如果连接到ROS2，可以发布参数更新
            # 这里可以添加参数发布逻辑

        except ValueError:
            pass

    def toggle_recording(self) -> None:
        """切换录制状态"""
        if self.recorder.recording:
            filename = self.recorder.stop_recording()
            self.record_button.config(text="开始录制")
            messagebox.showinfo("录制完成", f"数据已保存到:\n{filename}")
        else:
            self.recorder.start_recording()
            self.record_button.config(text="停止录制")

    def load_recorded_data(self) -> None:
        """加载录制的数据"""
        filename = filedialog.askopenfilename(
            title="选择录制数据文件",
            filetypes=[("Pickle files", "*.pkl"), ("All files", "*.*")]
        )

        if filename:
            try:
                self.replay_data = self.recorder.load_data(filename)
                self.replay_index = 0
                self.replay_progress['maximum'] = len(self.replay_data)
                messagebox.showinfo("加载成功", f"已加载 {len(self.replay_data)} 条数据记录")
            except Exception as e:
                messagebox.showerror("加载失败", f"无法加载数据文件:\n{e}")

    def toggle_replay(self) -> None:
        """切换回放状态"""
        if not self.replay_data:
            messagebox.showwarning("警告", "请先加载录制数据")
            return

        if self.replay_running:
            self.replay_running = False
            self.replay_button.config(text="开始回放")
        else:
            self.replay_running = True
            self.replay_button.config(text="停止回放")
            threading.Thread(target=self._replay_loop, daemon=True).start()

    def _replay_loop(self) -> None:
        """回放循环"""
        while self.replay_running and self.replay_index < len(self.replay_data):
            data = self.replay_data[self.replay_index]
            self.on_data_received(data)

            self.replay_progress['value'] = self.replay_index

            speed = self.replay_speed_scale.get()
            time.sleep(0.1 / speed)  # 基础延时除以速度倍数

            self.replay_index += 1

        if self.replay_index >= len(self.replay_data):
            self.replay_running = False
            self.replay_button.config(text="开始回放")
            self.replay_index = 0

    def export_data(self) -> None:
        """导出数据为JSON格式"""
        if not self.performance_monitor.timestamps:
            messagebox.showwarning("警告", "没有数据可导出")
            return

        filename = filedialog.asksaveasfilename(
            title="导出数据",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )

        if filename:
            try:
                export_data = {
                    'timestamps': [t.isoformat() for t in self.performance_monitor.timestamps],
                    'processing_times': list(self.performance_monitor.processing_times),
                    'memory_usage': list(self.performance_monitor.memory_usage),
                    'filter_ratios': list(self.performance_monitor.filter_ratios),
                    'point_counts': list(self.performance_monitor.point_counts),
                    'config': asdict(self.config)
                }

                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(export_data, f, indent=2, ensure_ascii=False)

                messagebox.showinfo("导出成功", f"数据已导出到:\n{filename}")

            except Exception as e:
                messagebox.showerror("导出失败", f"无法导出数据:\n{e}")

    def toggle_3d_visualization(self) -> None:
        """切换3D可视化"""
        if not OPEN3D_AVAILABLE:
            messagebox.showerror("错误", "Open3D不可用，无法开启3D可视化")
            return

        try:
            if self.open3d_visualizer is None:
                self.open3d_visualizer = Open3DVisualizer()
                messagebox.showinfo("3D可视化", "3D可视化窗口已打开")
            else:
                self.open3d_visualizer.close()
                self.open3d_visualizer = None
                messagebox.showinfo("3D可视化", "3D可视化窗口已关闭")
        except Exception as e:
            messagebox.showerror("错误", f"切换3D可视化时出错:\n{e}")

    def run(self) -> None:
        """运行主循环"""
        try:
            self.root.mainloop()
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """清理资源"""
        # 关闭3D可视化
        if self.open3d_visualizer:
            self.open3d_visualizer.close()

        # 关闭ROS2
        if ROS2_AVAILABLE and self.ros_subscriber:
            try:
                rclpy.shutdown()
            except:
                pass


def main():
    """主函数"""
    print("动态对象过滤器可视化工具启动中...")

    # 检查依赖
    missing_deps = []
    if not ROS2_AVAILABLE:
        missing_deps.append("ROS2 (rclpy, sensor_msgs, interface)")
    if not OPEN3D_AVAILABLE:
        missing_deps.append("Open3D")

    if missing_deps:
        print(f"警告: 以下依赖缺失: {', '.join(missing_deps)}")
        print("部分功能将不可用")

    # 创建并运行可视化器
    try:
        visualizer = DynamicFilterVisualizer()
        visualizer.run()
    except KeyboardInterrupt:
        print("\n用户中断，正在退出...")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()