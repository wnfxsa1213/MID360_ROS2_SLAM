#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MID360 SLAM 管理面板（PyQt5 原型）

当前功能：
  - 系统控制：启动/停止/重启/查看状态、实时/回放预设、选择 bag 回放
  - 配置管理：在 GUI 中浏览/修改 master_config.yaml 的核心参数、保存、生成、验证
  - 性能监控：订阅 /fastlio2/performance_metrics 与 /coordinator/metrics 实时显示
  - 工具联动：一键启动 gridmap_gui
依赖：
  pip install PyQt5
  （性能监控功能需安装 rclpy: sudo apt install ros-humble-rclpy）
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml
from PyQt5.QtCore import Qt, QProcess, pyqtSignal, QEvent, QObject
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QInputDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTabWidget,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SLAM_TOOLS = PROJECT_ROOT / "tools" / "slam_tools.sh"
MASTER_CONFIG = PROJECT_ROOT / "config" / "master_config.yaml"
GRIDMAP_GUI = PROJECT_ROOT / "tools" / "gridmap_gui.py"
MAP_REFINEMENT = PROJECT_ROOT / "tools" / "map_refinement.py"
MAP_RECONSTRUCT = PROJECT_ROOT / "tools" / "reconstruct_map_from_patches.py"

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError:  # pragma: no cover - 仅在 GUI 环境运行
    rclpy = None
    Float64MultiArray = None

CONFIG_HINTS = {
    # --- System / network / calibration ---
    "system.name": "系统名称标识，用于区分不同部署环境。",
    "system.version": "系统版本号，建议与发布版本保持一致。",
    "system.description": "系统描述信息，方便维护人员了解用途。",
    "network.host.ip": "主机 IP 地址，需要与雷达在同一网段才能通信。",
    "network.host.interface": "主机网卡名称，例如 enp7s0。必须对应真实物理接口。",
    "network.host.gateway": "主机默认网关地址，影响远程访问与路由。",
    "network.host.netmask": "子网掩码，必须与雷达端配置匹配。",
    "network.lidar.ip": "雷达 IP。若修改需同步配置雷达设备。",
    "network.lidar.lidar_ports.cmd_data": "雷达发送命令响应所使用的端口。",
    "network.lidar.lidar_ports.push_msg": "雷达事件/状态推送端口。",
    "network.lidar.lidar_ports.point_data": "雷达点云数据传输端口。",
    "network.lidar.lidar_ports.imu_data": "雷达 IMU 数据端口。",
    "network.lidar.lidar_ports.log_data": "雷达日志数据端口，用于调试。",
    "network.lidar.host_ports.cmd_data": "主机监听命令响应的端口。",
    "network.lidar.host_ports.push_msg": "主机监听雷达状态推送的端口。",
    "network.lidar.host_ports.point_data": "主机点云接收端口。",
    "network.lidar.host_ports.imu_data": "主机 IMU 接收端口。",
    "network.lidar.host_ports.log_data": "主机日志接收端口。",
    "calibration.lidar_to_imu.translation": "雷达到 IMU 的平移外参 (米)。根据安装位置测量。",
    "calibration.lidar_to_imu.rotation": "雷达到 IMU 的旋转矩阵（行优先）。保持正交。",
    "calibration.lidar_to_baselink.translation": "雷达到机器人基坐标系的平移 (米)。",
    "calibration.lidar_to_baselink.rotation_rpy": "雷达到机器人基坐标系的欧拉角 (弧度)。",
    "frames.world_frame": "SLAM 世界坐标系名称（map/odom）。",
    "frames.base_frame": "机器人主体坐标系（通常 base_link）。",
    "frames.lidar_frame": "雷达坐标系名称（点云 frame_id）。",

    # --- Topics ---
    "topics.imu": "IMU 数据话题，用于 LIO 融合。",
    "topics.lidar": "SLAM 主流程使用的点云话题。",
    "topics.odometry": "输出的 LIO 里程计话题。",
    "topics.path": "SLAM 路径话题，用于可视化。",
    "topics.body_cloud": "机体坐标系点云，用于 PGO/HBA。",
    "topics.world_cloud": "世界坐标系点云，可视化全局地图。",
    "topics.loop_markers": "PGO 回环标记话题。",
    "topics.hba_map": "HBA 输出的点云地图话题。",
    "topics.performance": "性能指标话题，监控 CPU/点数/缓存等。",
    "topics.diagnostics": "系统诊断话题。",

    # --- Livox driver ---
    "livox_driver.xfer_format": "数据格式：0=PointCloud2，1=CustomMsg。启用过滤桥需设为1。",
    "livox_driver.multi_topic": "多话题模式：0=单话题，1=按线束拆分多个话题。",
    "livox_driver.data_src": "数据源：0=实时雷达，1=文件/回放。",
    "livox_driver.publish_freq": "雷达点云发布频率 (Hz)。与雷达实际频率匹配。",
    "livox_driver.output_data_type": "输出数据类型：0=PointXYZRTL，1=PointXYZI。",
    "livox_driver.frame_id": "雷达点云 frame_id。",
    "livox_driver.cmdline_input_bd_code": "雷达设备码，多雷达使用时区分。",
    "livox_driver.lidar_config.pcl_data_type": "PCL 点云类型：0=PointXYZ，1=PointXYZI。",
    "livox_driver.lidar_config.pattern_mode": "扫描模式：0=非重复，1=重复，2=低功耗。",

    # --- FAST-LIO2 ---
    "fastlio2.lidar_min_range": "最小有效距离 (米)。增大可忽略近距离噪声，过大可能丢失周边结构。",
    "fastlio2.lidar_max_range": "最大有效距离 (米)。增大可保留远点，但配准耗时增加；过小可能导致回环失败。",
    "fastlio2.scan_resolution": "点云下采样体素大小 (米)。减小 = 更细腻但速度慢，增大 = 快但精度降。",
    "fastlio2.map_resolution": "地图体素大小 (米)。控制全局地图密度，越小越细致但占用更大。",
    "fastlio2.cube_len": "局部地图立方体边长 (米)。越大视野越广，但内存占用增大。",
    "fastlio2.det_range": "匹配时的搜索半径 (米)。增大可提升稳定性，代价是耗时增加。",
    "fastlio2.move_thresh": "触发地图滑动更新的距离阈值 (米)。过小频繁更新，过大可能漏掉局部重建。",
    "fastlio2.algorithm.imu_init_num": "IMU 初始化样本数。增大提升零偏估计精度，但启动更慢。",
    "fastlio2.algorithm.near_search_num": "LIO 最近邻搜索点数。越大越稳健，但计算量增大。",
    "fastlio2.algorithm.ieskf_max_iter": "IESKF 每帧最大迭代次数。增大提升收敛准确度，但耗时增加。",
    "fastlio2.algorithm.gravity_align": "是否执行重力对齐逻辑。",
    "fastlio2.algorithm.esti_il": "是否在线估计雷达-IMU 外参。",
    "fastlio2.point_cloud.lidar_cov_inv": "激光量测协方差逆权重。越大代表越信任激光测量。",
    "fastlio2.point_cloud.lidar_cov_scale": "协方差缩放因子。>1 放大误差、减小影响；<1 更信任点云。",
    "fastlio2.point_cloud.point_filter_num": "点滤波器级数。用于多级滤波，越大越平滑但延迟高。",
    "fastlio2.point_cloud.converge_thresh": "迭代收敛阈值。越小要求越严格，可能耗时更长。",
    "fastlio2.buffer_management.max_imu_buffer_size": "IMU 缓冲区大小。过小易丢数据，过大占内存。",
    "fastlio2.buffer_management.max_lidar_buffer_size": "点云缓冲最大数量。过小易丢帧，过大耗内存。",
    "fastlio2.buffer_management.enable_buffer_monitoring": "启用缓冲区监控，有助于诊断丢帧。",

    # --- Visualization / rosbag / monitoring ---
    "visualization.rviz.auto_start": "是否自动启动 RViz 可视化界面。",
    "visualization.rviz.config_file": "默认加载的 RViz 配置文件名称。",
    "visualization.rviz.display.pointcloud": "是否在 RViz 中显示点云。",
    "visualization.rviz.display.trajectory": "是否显示轨迹线。",
    "visualization.rviz.display.tf_frames": "是否显示 TF 坐标系。",
    "visualization.rviz.display.imu_pose": "是否显示 IMU 姿态箭头。",
    "visualization.rviz.display.diagnostics": "是否显示诊断信息面板。",
    "rosbag.auto_record": "启动后是否自动录制 bag 数据。",
    "rosbag.topics": "录制时要保存的话题列表。",
    "rosbag.output_dir": "bag 文件保存目录。",
    "rosbag.compression_format": "压缩格式：none / zstd / lz4。",
    "rosbag.max_duration": "单个 bag 最大时长（秒），0 为不限。",
    "monitoring.performance.enable_metrics": "是否发布性能指标。",
    "monitoring.performance.publish_frequency": "性能指标发布频率 (Hz)。",
    "monitoring.diagnostics.enable_diagnostics": "是否发布诊断消息。",
    "monitoring.diagnostics.check_frequency": "诊断检查频率 (Hz)。",
    "monitoring.logging.level": "日志级别：DEBUG、INFO、WARN、ERROR。",
    "monitoring.logging.verbose": "是否输出详细日志。",

    # --- Localizer ---
    "localizer.cloud_topic": "定位器输入点云话题。",
    "localizer.odom_topic": "定位器配套里程计话题。",
    "localizer.map_frame": "定位器使用的地图坐标系。",
    "localizer.local_frame": "定位器的局部坐标系。",
    "localizer.update_hz": "重定位更新频率 (Hz)。越高越及时但更耗资源。",
    "localizer.rough_localization.scan_resolution": "粗定位点云分辨率。",
    "localizer.rough_localization.map_resolution": "粗定位地图分辨率。",
    "localizer.rough_localization.max_iteration": "粗定位最大迭代次数。",
    "localizer.rough_localization.score_thresh": "粗定位匹配得分阈值。",
    "localizer.refine_localization.scan_resolution": "精定位点云分辨率。",
    "localizer.refine_localization.map_resolution": "精定位地图分辨率。",
    "localizer.refine_localization.max_iteration": "精定位最大迭代次数。",
    "localizer.refine_localization.score_thresh": "精定位匹配得分阈值。",
    "localizer.dynamic_filter.enable": "是否启用定位阶段的动态过滤。",
    "localizer.dynamic_filter.history_size": "历史帧数量。越大越稳健，但延迟、内存增加。",
    "localizer.dynamic_filter.stability_threshold": "稳定性阈值。越高越严格，可能误删静态点。",
    "localizer.dynamic_filter.search_radius": "邻域搜索半径 (米)。",
    "localizer.dynamic_filter.min_neighbors": "判断静态点的最小邻居数量。",
    "localizer.dynamic_filter.normal_consistency_thresh": "法向一致性阈值。",
    "localizer.dynamic_filter.density_ratio_thresh": "密度比阈值，判定稀疏点。",
    "localizer.dynamic_filter.downsample_ratio": "输入降采样比例，减轻运算负担。",
    "localizer.dynamic_filter.max_points_per_frame": "每帧最大点数，防止历史数据膨胀。",
    "localizer.dynamic_filter.voxel_size_base": "动态过滤体素大小 (米)。越小保留更多细节。",
    "localizer.dynamic_filter.motion_threshold": "判定动态点的位移阈值 (米)。",
    "localizer.dynamic_filter.max_time_diff": "历史帧时间差容忍范围 (秒)。",
    "localizer.dynamic_filter.max_processing_hz": "过滤处理频率上限，控制 CPU 占用。",

    # --- Point cloud filter bridge ---
    "point_cloud_filter_bridge.input_topic": "过滤桥输入点云话题，通常 /livox/lidar。",
    "point_cloud_filter_bridge.output_topic": "过滤桥输出点云话题，通常 /livox/lidar_filtered。",
    "point_cloud_filter_bridge.debug_topic": "调试点云话题。",
    "point_cloud_filter_bridge.stats_topic": "过滤统计话题。",
    "point_cloud_filter_bridge.debug_enabled": "是否开启调试模式（输出额外日志/点云）。",
    "point_cloud_filter_bridge.publish_stats": "是否发布过滤统计。",
    "point_cloud_filter_bridge.max_processing_hz": "过滤桥最大处理频率。",
    "point_cloud_filter_bridge.dynamic_filter.enable": "是否启用实时动态过滤桥。",
    "point_cloud_filter_bridge.dynamic_filter.history_size": "过滤桥历史帧数。",
    "point_cloud_filter_bridge.dynamic_filter.stability_threshold": "稳定性阈值，越高越严格。",
    "point_cloud_filter_bridge.dynamic_filter.search_radius": "搜索邻域半径 (米)。",
    "point_cloud_filter_bridge.dynamic_filter.min_neighbors": "判定静态点的最小邻居数。",
    "point_cloud_filter_bridge.dynamic_filter.normal_consistency_thresh": "法向一致性阈值。",
    "point_cloud_filter_bridge.dynamic_filter.density_ratio_thresh": "密度比阈值。",
    "point_cloud_filter_bridge.dynamic_filter.downsample_ratio": "输入降采样比例。",
    "point_cloud_filter_bridge.dynamic_filter.max_points_per_frame": "每帧最大输出点数。",
    "point_cloud_filter_bridge.dynamic_filter.voxel_size_base": "过滤桥体素大小。",
    "point_cloud_filter_bridge.dynamic_filter.motion_threshold": "动态点检测的位移阈值。",
    "point_cloud_filter_bridge.dynamic_filter.max_time_diff": "历史帧时间差容忍范围。",
    "point_cloud_filter_bridge.dynamic_filter.max_processing_hz": "动态过滤的最大处理频率。",

    # --- Cooperation ---
    "cooperation.enable": "是否启用协同优化协调器。",
    "cooperation.drift_threshold": "普通漂移阈值 (米)，超过触发常规优化。",
    "cooperation.time_threshold": "常规优化最小时间间隔 (秒)。",
    "cooperation.emergency_threshold": "紧急漂移阈值 (米)，超过立即触发紧急优化。",
    "cooperation.auto_optimization": "是否允许协调器自动触发优化。",
    "cooperation.multi_agent.max_agents": "多机器人协同时允许的最大机器人数量。",
    "cooperation.multi_agent.communication_freq": "多机器人协同通信频率 (Hz)。",
}


# --------------------------------------------------------------------------- 工具
class CommandRunner:
    """封装 QProcess 运行命令并回调输出。"""

    def __init__(self, append_log):
        self.process = QProcess()
        self.append_log = append_log
        self.process.readyReadStandardOutput.connect(self._handle_stdout)
        self.process.readyReadStandardError.connect(self._handle_stderr)
        self.process.finished.connect(self._handle_finished)

    def run(self, cmd, cwd=None):
        if self.process.state() != QProcess.NotRunning:
            self.append_log("⚠️ 上一个命令仍在执行，已忽略新的请求。\n")
            return
        self.append_log(f"$ {' '.join(cmd)}\n")
        self.process.setProgram(cmd[0])
        self.process.setArguments(cmd[1:])
        if cwd:
            self.process.setWorkingDirectory(str(cwd))
        self.process.start()

    def _handle_stdout(self):
        self.append_log(
            bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="ignore")
        )

    def _handle_stderr(self):
        self.append_log(
            bytes(self.process.readAllStandardError()).decode("utf-8", errors="ignore")
        )

    def _handle_finished(self, exit_code, _exit_status):
        status = "成功" if exit_code == 0 else f"失败 (exit={exit_code})"
        self.append_log(f"👉 命令执行完成：{status}\n")


# --------------------------------------------------------------------------- GUI 主窗口
class SlamManagerWindow(QMainWindow):
    # 界面与行为常量
    DEFAULT_WIDTH = 1100
    DEFAULT_HEIGHT = 720
    MAX_LOG_LINES = 1000
    ROS_SPIN_TIMEOUT = 0.5
    THREAD_CHECK_INTERVAL_MS = 100

    # 数值范围辅助
    MAX_INT_THRESHOLD = 100
    MAX_INT_HISTORY = 500
    MAX_INT_BUFFER = 100_000
    MAX_INT_POINTS = 1_000_000
    MAX_INT_SIZE = 10_000
    MAX_INT_DEFAULT = 1_000_000
    MAX_DOUBLE_THRESHOLD = 100.0
    MAX_DOUBLE_RATIO = 10.0
    MAX_DOUBLE_FREQ = 100.0
    MAX_DOUBLE_RESOLUTION = 100.0
    MAX_DOUBLE_DEFAULT = 1_000_000.0

    metrics_signal = pyqtSignal(str)
    coordinator_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MID360 SLAM 管理面板 (原型)")
        self.resize(self.DEFAULT_WIDTH, self.DEFAULT_HEIGHT)

        self.runner = CommandRunner(self.append_log)
        self.metrics_signal.connect(self._append_metrics_text)
        self.coordinator_signal.connect(self._append_coord_text)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_control_tab(), "系统控制")
        self.tabs.addTab(self._build_config_tab(), "配置管理")
        self.tabs.addTab(self._build_monitor_tab(), "性能监控")
        self.tabs.addTab(self._build_tools_tab(), "工具联动")
        layout.addWidget(self.tabs)

        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setPlaceholderText("命令输出将在此显示...")
        layout.addWidget(self.log_edit, 1)

        self.monitor_thread: Optional[threading.Thread] = None
        self.monitor_stop_event: Optional[threading.Event] = None
        self.cleanup_timer_id: Optional[int] = None
        self.rclpy_initialized = False
        self.ros_node: Optional["Node"] = None  # type: ignore
        self.monitor_subscriptions = []
        self._closing_via_timer = False

    # --------------------------- Tab: 系统控制 ---------------------------
    def _build_control_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 基本控制
        control_box = QGroupBox("系统控制")
        control_layout = QHBoxLayout(control_box)
        btn_start = QPushButton("启动 (默认)")
        btn_start.clicked.connect(lambda: self.run_slam_tools(["start"]))
        btn_stop = QPushButton("停止")
        btn_stop.clicked.connect(lambda: self.run_slam_tools(["stop"]))
        btn_restart = QPushButton("重启")
        btn_restart.clicked.connect(lambda: self.run_slam_tools(["restart"]))
        btn_status = QPushButton("状态")
        btn_status.clicked.connect(lambda: self.run_slam_tools(["status"]))
        control_layout.addWidget(btn_start)
        control_layout.addWidget(btn_stop)
        control_layout.addWidget(btn_restart)
        control_layout.addWidget(btn_status)
        layout.addWidget(control_box)

        # 预设控制
        preset_box = QGroupBox("预设模式")
        preset_layout = QHBoxLayout(preset_box)
        btn_realtime = QPushButton("实时模式")
        btn_realtime.clicked.connect(lambda: self.run_slam_tools(["start", "realtime"]))
        btn_replay = QPushButton("回放模式 (仅启动)")
        btn_replay.clicked.connect(lambda: self.run_slam_tools(["start", "replay"]))
        btn_nofilter = QPushButton("禁用过滤桥")
        btn_nofilter.clicked.connect(lambda: self.run_slam_tools(["start", "nofilter"]))
        preset_layout.addWidget(btn_realtime)
        preset_layout.addWidget(btn_replay)
        preset_layout.addWidget(btn_nofilter)
        layout.addWidget(preset_box)

        # 回放控制
        replay_box = QGroupBox("数据回放")
        replay_layout = QGridLayout(replay_box)
        self.bag_path_edit = QLineEdit()
        self.bag_path_edit.setPlaceholderText("选择 bag 目录或文件")
        btn_browse = QPushButton("浏览")
        btn_browse.clicked.connect(self.choose_bag)
        btn_play = QPushButton("启动回放")
        btn_play.clicked.connect(self.start_replay_with_bag)
        replay_layout.addWidget(QLabel("Bag 路径:"), 0, 0)
        replay_layout.addWidget(self.bag_path_edit, 0, 1)
        replay_layout.addWidget(btn_browse, 0, 2)
        replay_layout.addWidget(btn_play, 1, 0, 1, 3)
        layout.addWidget(replay_box)

        layout.addStretch(1)
        return tab

    # --------------------------- Tab: 配置管理 ---------------------------
    def _build_config_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter, 1)

        self.config_tree = QListWidget()
        self.config_tree.itemClicked.connect(self.on_config_item_clicked)
        splitter.addWidget(self.config_tree)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        splitter.addWidget(right_panel)

        self.form_stack = QStackedWidget()
        self.placeholder_form = QWidget()
        placeholder_layout = QVBoxLayout(self.placeholder_form)
        placeholder_label = QLabel("选择左侧模块以加载可编辑参数。")
        placeholder_label.setWordWrap(True)
        placeholder_layout.addWidget(placeholder_label)
        placeholder_layout.addStretch(1)
        self.form_stack.addWidget(self.placeholder_form)
        right_layout.addWidget(self.form_stack, 1)
        self.hint_label = QLabel("选择左侧模块或在右侧输入框悬停可查看参数说明。")
        self.hint_label.setWordWrap(True)
        right_layout.addWidget(self.hint_label)
        self.default_hint_text = self.hint_label.text()
        self.section_hint = self.default_hint_text

        buttons = QHBoxLayout()
        btn_reload = QPushButton("重新加载")
        btn_reload.clicked.connect(self.load_master_config)
        btn_save = QPushButton("保存配置")
        btn_save.clicked.connect(self.save_master_config)
        btn_generate = QPushButton("生成配置")
        btn_generate.clicked.connect(lambda: self.run_slam_tools(["config", "generate"]))
        btn_validate = QPushButton("验证配置")
        btn_validate.clicked.connect(lambda: self.run_slam_tools(["config", "validate"]))
        btn_status = QPushButton("查看状态")
        btn_status.clicked.connect(lambda: self.run_slam_tools(["config", "status"]))
        buttons.addWidget(btn_reload)
        buttons.addWidget(btn_save)
        buttons.addWidget(btn_generate)
        buttons.addWidget(btn_validate)
        buttons.addWidget(btn_status)
        right_layout.addLayout(buttons)

        info = QLabel(
            "说明：左侧选择模块，右侧编辑关键参数并点击“保存配置”。\n"
            "保存后再执行“生成配置”即可同步子配置文件。"
        )
        info.setWordWrap(True)
        right_layout.addWidget(info)

        self.config_data: Dict[str, Any] = {}
        self.form_cache: Dict[str, QWidget] = {}
        self.form_widgets_map: Dict[str, List[QWidget]] = {}
        self.current_section: Optional[str] = None
        self.load_master_config()
        return tab

    # --------------------------- Tab: 性能监控 ---------------------------
    def _build_monitor_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        if rclpy is None:
            layout.addWidget(
                QLabel("未安装 rclpy，无法启用性能监控。请 `sudo apt install ros-humble-rclpy`。")
            )
            return tab

        perf_box = QGroupBox("性能指标 ( /fastlio2/performance_metrics )")
        perf_layout = QVBoxLayout(perf_box)
        layout.addWidget(perf_box)

        self.metrics_edit = QTextEdit()
        self.metrics_edit.setReadOnly(True)
        self.metrics_edit.setPlaceholderText("等待性能指标...")
        perf_layout.addWidget(self.metrics_edit)

        coord_box = QGroupBox("协同指标 ( /coordinator/metrics )")
        coord_layout = QVBoxLayout(coord_box)
        layout.addWidget(coord_box)

        self.coordinator_edit = QTextEdit()
        self.coordinator_edit.setReadOnly(True)
        self.coordinator_edit.setPlaceholderText("等待协同指标...")
        coord_layout.addWidget(self.coordinator_edit)

        buttons = QHBoxLayout()
        btn_start = QPushButton("启动监听")
        btn_start.clicked.connect(self.start_monitoring)
        btn_stop = QPushButton("停止监听")
        btn_stop.clicked.connect(self.stop_monitoring)
        buttons.addWidget(btn_start)
        buttons.addWidget(btn_stop)
        layout.addLayout(buttons)

        return tab

    # --------------------------- Tab: 工具联动 ---------------------------
    def _build_tools_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        tool_box = QGroupBox("外部工具")
        tool_layout = QHBoxLayout(tool_box)

        btn_gridmap_gui = QPushButton("打开 Gridmap GUI")
        btn_gridmap_gui.clicked.connect(self.launch_gridmap_gui)
        tool_layout.addWidget(btn_gridmap_gui)

        btn_map_refine = QPushButton("地图精修 (map_refinement)")
        btn_map_refine.clicked.connect(self.launch_map_refinement)
        tool_layout.addWidget(btn_map_refine)

        btn_reconstruct = QPushButton("补丁重建 (reconstruct)")
        btn_reconstruct.clicked.connect(self.launch_map_reconstruct)
        tool_layout.addWidget(btn_reconstruct)

        layout.addWidget(tool_box)

        info = QLabel(
            "Gridmap GUI 依赖 PyQt5/OpenCV；地图精修与补丁重建会调用对应 Python 脚本，"
            "需要提前准备好输入点云或 PGO 输出目录。"
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        layout.addStretch(1)
        return tab

    # --------------------------- 命令执行 helpers ---------------------------
    def run_slam_tools(self, args):
        if not SLAM_TOOLS.exists():
            QMessageBox.critical(self, "错误", f"未找到 {SLAM_TOOLS}")
            return
        cmd = ["/bin/bash", str(SLAM_TOOLS)] + args
        self.runner.run(cmd, cwd=PROJECT_ROOT)

    def choose_bag(self):
        path = QFileDialog.getExistingDirectory(
            self, "选择 bag 目录", str((PROJECT_ROOT / "data").resolve())
        )
        if path:
            self.bag_path_edit.setText(path)

    def start_replay_with_bag(self):
        bag_path = self.bag_path_edit.text().strip()
        if not bag_path:
            QMessageBox.warning(self, "提示", "请先选择 bag 路径。")
            return
        path_obj = Path(bag_path)
        if not path_obj.exists():
            QMessageBox.warning(self, "提示", f"路径不存在：{bag_path}")
            return
        self.run_slam_tools(["start", "replay", "--bag", bag_path])

    def launch_gridmap_gui(self):
        if not GRIDMAP_GUI.exists():
            QMessageBox.warning(self, "提示", f"未找到 {GRIDMAP_GUI}")
            return
        subprocess.Popen([sys.executable, str(GRIDMAP_GUI)], cwd=PROJECT_ROOT)

    def launch_map_refinement(self):
        if not MAP_REFINEMENT.exists():
            QMessageBox.warning(self, "提示", f"未找到 {MAP_REFINEMENT}")
            return

        input_file, _ = QFileDialog.getOpenFileName(
            self,
            "选择输入点云",
            str((PROJECT_ROOT / "saved_maps").resolve()),
            "Point Cloud (*.pcd *.ply);;All Files (*)",
        )
        if not input_file:
            return

        input_path = Path(input_file)
        default_output = input_path.parent / f"{input_path.stem}_refined{input_path.suffix}"
        output_file, _ = QFileDialog.getSaveFileName(
            self,
            "选择输出点云",
            str(default_output),
            "Point Cloud (*.pcd *.ply);;All Files (*)",
        )
        if not output_file:
            return

        pipelines = ["full", "basic", "denoise", "structure", "edges", "geometry", "surface", "details", "density"]
        pipeline, ok = QInputDialog.getItem(
            self,
            "选择处理管道",
            "处理管道：",
            pipelines,
            pipelines.index("full"),
            False,
        )
        if not ok or not pipeline:
            return

        if Path(output_file).exists():
            confirm = QMessageBox.question(
                self,
                "确认覆盖",
                f"输出文件已存在，是否覆盖？\n{output_file}",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if confirm != QMessageBox.Yes:
                return

        quality_choice = QMessageBox.question(
            self,
            "质量报告",
            "是否生成质量评估报告？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        cmd = [sys.executable, str(MAP_REFINEMENT), "--pipeline", pipeline]

        sub_cmd = [
            "refine",
            input_file,
            "--output",
            output_file,
            "--overwrite",
        ]
        if quality_choice == QMessageBox.Yes:
            sub_cmd.append("--quality-report")

        self.runner.run(cmd + sub_cmd, cwd=PROJECT_ROOT)

    def launch_map_reconstruct(self):
        if not MAP_RECONSTRUCT.exists():
            QMessageBox.warning(self, "提示", f"未找到 {MAP_RECONSTRUCT}")
            return

        mode_options = ["PGO 目录（含 patches/poses.txt）", "补丁目录 + 自定义 poses"]
        mode, ok = QInputDialog.getItem(
            self,
            "选择输入类型",
            "请选择重建数据来源：",
            mode_options,
            0,
            False,
        )
        if not ok or not mode:
            return

        args = [sys.executable, str(MAP_RECONSTRUCT)]

        if mode == mode_options[0]:
            pgo_dir = QFileDialog.getExistingDirectory(
                self,
                "选择 PGO 输出目录",
                str((PROJECT_ROOT / "saved_maps").resolve()),
            )
            if not pgo_dir:
                return
            args.extend(["--pgo-dir", pgo_dir])
            default_output = Path(pgo_dir).with_name(Path(pgo_dir).name + "_reconstructed.pcd")
        else:
            patches_dir = QFileDialog.getExistingDirectory(
                self,
                "选择 patches 目录",
                str((PROJECT_ROOT / "saved_maps").resolve()),
            )
            if not patches_dir:
                return
            poses_file, _ = QFileDialog.getOpenFileName(
                self,
                "选择 poses.txt",
                str(Path(patches_dir).parent),
                "Text Files (*.txt);;All Files (*)",
            )
            if not poses_file:
                return
            args.extend(["--patches-dir", patches_dir, "--poses", poses_file])
            default_output = Path(patches_dir).with_name(Path(patches_dir).name + "_reconstructed.pcd")

        output_file, _ = QFileDialog.getSaveFileName(
            self,
            "选择输出点云",
            str(default_output),
            "Point Cloud (*.pcd *.ply);;All Files (*)",
        )
        if not output_file:
            return

        if Path(output_file).exists():
            confirm = QMessageBox.question(
                self,
                "确认覆盖",
                f"输出文件已存在，是否覆盖？\n{output_file}",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if confirm != QMessageBox.Yes:
                return

        voxel, voxel_ok = QInputDialog.getDouble(
            self,
            "体素下采样",
            "体素大小 (米，0 表示不下采样)：",
            0.0,
            0.0,
            1.0,
            2,
        )

        args.extend(["--output", output_file])
        if voxel_ok and voxel > 0.0:
            args.extend(["--downsample-voxel", f"{voxel:.2f}"])

        self.runner.run(args, cwd=PROJECT_ROOT)

    def append_log(self, text: str):
        self.log_edit.moveCursor(QTextCursor.End)
        self.log_edit.insertPlainText(text)
        self.log_edit.moveCursor(QTextCursor.End)
        self._trim_text_edit(self.log_edit)

    def _trim_text_edit(self, text_edit: QTextEdit) -> None:
        doc = text_edit.document()
        if doc.blockCount() <= self.MAX_LOG_LINES:
            return

        cursor = QTextCursor(doc)
        cursor.movePosition(QTextCursor.Start)
        blocks_to_remove = doc.blockCount() - self.MAX_LOG_LINES
        cursor.movePosition(
            QTextCursor.Down,
            QTextCursor.KeepAnchor,
            blocks_to_remove,
        )
        cursor.removeSelectedText()
        cursor.movePosition(QTextCursor.Start)
        if cursor.block().length() == 1:
            cursor.deleteChar()

    # --------------------------- 配置管理 ---------------------------
    def _reset_form_cache(self) -> None:
        for widgets in self.form_widgets_map.values():
            for widget in widgets:
                if widget is None:
                    continue
                try:
                    widget.removeEventFilter(self)
                except RuntimeError:
                    pass
        self.form_widgets_map.clear()

        for widget in list(self.form_cache.values()):
            self.form_stack.removeWidget(widget)
            widget.deleteLater()
        self.form_cache.clear()
        self.current_section = None
        self.section_hint = self.default_hint_text
        self.hint_label.setText(self.section_hint)
        self.form_stack.setCurrentWidget(self.placeholder_form)

    def load_master_config(self) -> None:
        if not MASTER_CONFIG.exists():
            QMessageBox.warning(self, "提示", f"未找到 {MASTER_CONFIG}")
            return
        try:
            with open(MASTER_CONFIG, "r", encoding="utf-8") as f:
                self.config_data = yaml.safe_load(f) or {}
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载配置失败: {e}")
            return

        self._reset_form_cache()
        self.config_tree.clear()
        for key in self.config_data.keys():
            item = QListWidgetItem(key)
            item.setToolTip(CONFIG_HINTS.get(key, ""))
            self.config_tree.addItem(item)
        if self.config_tree.count() > 0:
            first_item = self.config_tree.item(0)
            self.config_tree.setCurrentItem(first_item)
            self.on_config_item_clicked(first_item)
        else:
            self.form_stack.setCurrentWidget(self.placeholder_form)

    def on_config_item_clicked(self, item: QListWidgetItem) -> None:
        key = item.text()
        value = self.config_data.get(key)
        if key == self.current_section:
            return

        if isinstance(value, dict):
            widget = self.form_cache.get(key)
            if widget is None:
                widget = self._build_form_widget(key, value)
                self.form_cache[key] = widget
                self.form_stack.addWidget(widget)
            self.form_stack.setCurrentWidget(widget)
            self.section_hint = CONFIG_HINTS.get(key, self.default_hint_text)
            self.hint_label.setText(self.section_hint)
        else:
            message = "该模块不是字典结构，请直接编辑 master_config.yaml。"
            widget = self.form_cache.get(key)
            if widget is None:
                widget = self._build_message_widget(message)
                self.form_cache[key] = widget
                self.form_stack.addWidget(widget)
            self.form_stack.setCurrentWidget(widget)
            self.section_hint = message
            self.hint_label.setText(self.section_hint)

        self.current_section = key

    def _build_form_widget(self, section: str, data: Dict[str, Any]) -> QWidget:
        container = QWidget()
        form_layout = QFormLayout(container)
        form_layout.setContentsMargins(0, 0, 0, 0)
        self._populate_form(form_layout, section, section, data)
        return container

    def _build_message_widget(self, message: str) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        label = QLabel(message)
        label.setWordWrap(True)
        layout.addWidget(label)
        layout.addStretch(1)
        return container

    def _populate_form(
        self,
        form_layout: QFormLayout,
        section: str,
        prefix: str,
        data: Dict[str, Any],
    ) -> None:
        for key, val in data.items():
            full_key = f"{prefix}.{key}"
            hint = CONFIG_HINTS.get(full_key, "")
            if isinstance(val, dict):
                title = QLabel(f"<b>{full_key}</b>")
                title.setToolTip(hint)
                form_layout.addRow(title, QLabel("(子树)"))
                self._populate_form(form_layout, section, full_key, val)
                continue

            if isinstance(val, bool):
                checkbox = QCheckBox(full_key)
                checkbox.setChecked(val)
                checkbox.stateChanged.connect(
                    lambda state, path=full_key: self.update_config_value(path, state == Qt.Checked)
                )
                checkbox.setToolTip(hint)
                form_layout.addRow(checkbox)
                self._register_hint_widget(checkbox, hint, section)
                continue

            if isinstance(val, int) and not isinstance(val, bool):
                spin = QSpinBox()
                spin.setMaximum(self._guess_int_max(full_key))
                spin.setValue(val)
                spin.valueChanged.connect(
                    lambda new_val, path=full_key: self.update_config_value(path, int(new_val))
                )
                spin.setToolTip(hint)
                form_layout.addRow(full_key, spin)
                self._register_hint_widget(spin, hint, section)
                continue

            if isinstance(val, float):
                dspin = QDoubleSpinBox()
                dspin.setDecimals(4)
                dspin.setMaximum(self._guess_double_max(full_key))
                dspin.setValue(val)
                dspin.valueChanged.connect(
                    lambda new_val, path=full_key: self.update_config_value(path, float(new_val))
                )
                dspin.setToolTip(hint)
                form_layout.addRow(full_key, dspin)
                self._register_hint_widget(dspin, hint, section)
                continue

            edit = QLineEdit(str(val))
            edit.editingFinished.connect(
                lambda path=full_key, widget=edit: self.update_config_value(path, widget.text())
            )
            edit.setToolTip(hint)
            form_layout.addRow(full_key, edit)
            self._register_hint_widget(edit, hint, section)

    def _guess_int_max(self, key: str) -> int:
        lower = key.lower()
        if "threshold" in lower:
            return self.MAX_INT_THRESHOLD
        if "history" in lower or "count" in lower:
            return self.MAX_INT_HISTORY
        if "buffer" in lower:
            return self.MAX_INT_BUFFER
        if "points" in lower:
            return self.MAX_INT_POINTS
        if "size" in lower or "num" in lower:
            return self.MAX_INT_SIZE
        return self.MAX_INT_DEFAULT

    def _guess_double_max(self, key: str) -> float:
        lower = key.lower()
        if "threshold" in lower:
            return self.MAX_DOUBLE_THRESHOLD
        if "ratio" in lower:
            return self.MAX_DOUBLE_RATIO
        if "frequency" in lower or "hz" in lower:
            return self.MAX_DOUBLE_FREQ
        if "resolution" in lower or "size" in lower:
            return self.MAX_DOUBLE_RESOLUTION
        return self.MAX_DOUBLE_DEFAULT

    def _register_hint_widget(self, widget: QWidget, hint: str, section: str) -> None:
        widget.setProperty("hint_text", hint)
        widget.installEventFilter(self)
        widget.setFocusPolicy(Qt.StrongFocus)
        self.form_widgets_map.setdefault(section, []).append(widget)

    def eventFilter(self, obj: QObject, event: QEvent):
        if event.type() in (QEvent.Enter, QEvent.FocusIn):
            hint = obj.property("hint_text")
            if hint:
                self.hint_label.setText(str(hint))
            else:
                self.hint_label.setText(self.section_hint)
        elif event.type() in (QEvent.Leave, QEvent.FocusOut):
            self.hint_label.setText(self.section_hint)
        return super().eventFilter(obj, event)

    def update_config_value(self, path: str, value: Any) -> bool:
        if not path or ".." in path:
            QMessageBox.warning(self, "错误", "无效的配置路径。")
            return False

        if not self._validate_config_value(path, value):
            QMessageBox.warning(self, "错误", f"配置值验证失败：{path} = {value}")
            return False

        keys = path.split(".")
        ref: Dict[str, Any] = self.config_data
        for key in keys[:-1]:
            ref = ref.setdefault(key, {})
        ref[keys[-1]] = value
        return True

    def _validate_config_value(self, path: str, value: Any) -> bool:
        current = self._resolve_config_reference(path)
        if current is None:
            return True
        if isinstance(current, bool):
            return isinstance(value, bool)
        if isinstance(current, int) and not isinstance(current, bool):
            return isinstance(value, int) and not isinstance(value, bool)
        if isinstance(current, float):
            return isinstance(value, (float, int))
        return True

    def _resolve_config_reference(self, path: str) -> Optional[Any]:
        keys = path.split(".")
        ref: Any = self.config_data
        for key in keys[:-1]:
            if not isinstance(ref, dict):
                return None
            ref = ref.get(key)
        if not isinstance(ref, dict):
            return None
        return ref.get(keys[-1])

    def save_master_config(self) -> None:
        try:
            with open(MASTER_CONFIG, "w", encoding="utf-8") as f:
                yaml.safe_dump(self.config_data, f, allow_unicode=True, sort_keys=False)
            QMessageBox.information(self, "成功", f"已保存 {MASTER_CONFIG}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {e}")

    # --------------------------- 性能监控 ---------------------------
    def start_monitoring(self):
        if rclpy is None:
            QMessageBox.warning(self, "提示", "当前环境未安装 rclpy，无法启用监控。")
            return
        if self.monitor_thread and self.monitor_thread.is_alive():
            QMessageBox.information(self, "提示", "性能监控已在运行。")
            return

        self.monitor_stop_event = threading.Event()
        self.monitor_thread = threading.Thread(target=self._monitor_spin, daemon=True)
        self.monitor_thread.start()
        QMessageBox.information(self, "提示", "性能监控已启动。")

    def stop_monitoring(self):
        if self.monitor_stop_event:
            self.monitor_stop_event.set()
            self.metrics_signal.emit("停止性能监控...")

    def _monitor_spin(self):
        try:
            self.monitor_subscriptions = []
            if not self.rclpy_initialized:
                rclpy.init()
                self.rclpy_initialized = True

            if not self.ros_node:
                self.ros_node = rclpy.create_node("slam_gui_monitor")

            def perf_callback(msg: Float64MultiArray):
                data = [round(v, 4) for v in msg.data]
                self.metrics_signal.emit(f"/fastlio2/performance_metrics: {data}")

            def coord_callback(msg: Float64MultiArray):
                data = [round(v, 4) for v in msg.data]
                self.coordinator_signal.emit(f"/coordinator/metrics: {data}")

            sub1 = self.ros_node.create_subscription(
                Float64MultiArray, "/fastlio2/performance_metrics", perf_callback, 10
            )
            sub2 = self.ros_node.create_subscription(
                Float64MultiArray, "/coordinator/metrics", coord_callback, 10
            )
            self.monitor_subscriptions = [sub1, sub2]

            while rclpy.ok() and not self.monitor_stop_event.is_set():
                rclpy.spin_once(self.ros_node, timeout_sec=self.ROS_SPIN_TIMEOUT)

        except rclpy.exceptions.ROSInterruptException:
            self.metrics_signal.emit("ROS被中断\n")
        except rclpy.exceptions.InvalidHandle:
            self.metrics_signal.emit("ROS句柄无效\n")
        except Exception as exc:  # pragma: no cover
            self.metrics_signal.emit(
                f"未知异常: {exc}\n{traceback.format_exc()}\n"
            )
        finally:
            for sub in self.monitor_subscriptions:
                try:
                    self.ros_node.destroy_subscription(sub)
                except Exception:
                    pass
            self.monitor_subscriptions = []

            if self.ros_node:
                try:
                    self.ros_node.destroy_node()
                except Exception:
                    pass
                self.ros_node = None

            self.monitor_thread = None
            self.monitor_stop_event = None

    def _append_metrics_text(self, text: str):
        self.metrics_edit.append(text)
        self._trim_text_edit(self.metrics_edit)

    def _append_coord_text(self, text: str):
        self.coordinator_edit.append(text)
        self._trim_text_edit(self.coordinator_edit)

    def _shutdown_ros(self) -> None:
        if not self.rclpy_initialized or rclpy is None:
            return
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
                self.ros_node = None
            self.monitor_subscriptions = []
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        finally:
            self.rclpy_initialized = False

    def closeEvent(self, event):
        if self._closing_via_timer:
            self._closing_via_timer = False
            self._shutdown_ros()
            event.accept()
            return

        if self.monitor_thread and self.monitor_thread.is_alive():
            reply = QMessageBox.question(
                self,
                "确认",
                "性能监控仍在运行，确定退出吗？",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply == QMessageBox.No:
                event.ignore()
                return
            self.stop_monitoring()
            if self.cleanup_timer_id is None:
                self.cleanup_timer_id = self.startTimer(self.THREAD_CHECK_INTERVAL_MS)
            event.ignore()
            return

        self._shutdown_ros()
        event.accept()

    def timerEvent(self, event):
        if (
            self.cleanup_timer_id is not None
            and event.timerId() == self.cleanup_timer_id
        ):
            if self.monitor_thread and self.monitor_thread.is_alive():
                return
            self.killTimer(self.cleanup_timer_id)
            self.cleanup_timer_id = None
            self._closing_via_timer = True
            super().close()
            return
        super().timerEvent(event)


# --------------------------------------------------------------------------- 启动
def main():
    app = QApplication(sys.argv)
    window = SlamManagerWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
