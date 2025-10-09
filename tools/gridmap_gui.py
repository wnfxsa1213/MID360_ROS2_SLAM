#!/usr/bin/env python3
"""
可视化的 PCD → 栅格地图转换工具（PyQt5）

功能概览：
- 快速选择待转换的 PCD（支持索引 saved_maps/.pcd_index.json）或手动浏览
- 选择 patches/poses/HBA 轨迹（可选）与 ROI 裁剪
- 常用参数可视化调节：分辨率、阈值、膨胀、门洞保护、地面主导、预处理等
- 一键执行并显示日志；成功后加载并预览生成的 PGM 地图

依赖：PyQt5、numpy、opencv-python
"""

import json
import os
import sys
from pathlib import Path
import subprocess

import numpy as np
import cv2

from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QTextEdit
)


REPO_ROOT = Path(__file__).resolve().parent.parent
TOOLS_DIR = Path(__file__).resolve().parent
SAVED_MAPS = REPO_ROOT / 'saved_maps'
INDEX_JSON = SAVED_MAPS / '.pcd_index.json'


def load_index() -> list:
    if not INDEX_JSON.exists():
        return []
    try:
        with open(INDEX_JSON, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data.get('files', [])
    except Exception:
        return []


class GridMapGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PCD → 栅格地图转换器')
        self.resize(1200, 800)

        self.process = None  # type: QProcess

        central = QWidget()
        self.setCentralWidget(central)
        layout = QGridLayout(central)

        # 输入与数据源
        src_group = QGroupBox('数据源')
        src_layout = QGridLayout(src_group)

        self.combo_pcd = QComboBox()
        self.btn_refresh = QPushButton('刷新索引')
        self.btn_browse_pcd = QPushButton('浏览PCD')
        self.edit_pcd = QLineEdit()
        self.edit_pcd.setPlaceholderText('或手动输入PCD路径')

        src_layout.addWidget(QLabel('索引PCD列表'), 0, 0)
        src_layout.addWidget(self.combo_pcd, 0, 1, 1, 2)
        src_layout.addWidget(self.btn_refresh, 0, 3)
        src_layout.addWidget(QLabel('PCD路径'), 1, 0)
        src_layout.addWidget(self.edit_pcd, 1, 1, 1, 2)
        src_layout.addWidget(self.btn_browse_pcd, 1, 3)

        # patches/poses/hba
        self.edit_patches = QLineEdit()
        self.edit_poses = QLineEdit()
        self.edit_hba = QLineEdit()
        self.btn_browse_patches = QPushButton('选择目录')
        self.btn_browse_poses = QPushButton('选择文件')
        self.btn_browse_hba = QPushButton('选择文件')
        self.chk_roi_from_hba = QCheckBox('使用HBA轨迹ROI')
        self.spin_roi_buf = QDoubleSpinBox(); self.spin_roi_buf.setDecimals(2); self.spin_roi_buf.setRange(0.0, 50.0); self.spin_roi_buf.setValue(2.0)

        src_layout.addWidget(QLabel('patches 目录'), 2, 0)
        src_layout.addWidget(self.edit_patches, 2, 1, 1, 2)
        src_layout.addWidget(self.btn_browse_patches, 2, 3)
        src_layout.addWidget(QLabel('poses.txt'), 3, 0)
        src_layout.addWidget(self.edit_poses, 3, 1, 1, 2)
        src_layout.addWidget(self.btn_browse_poses, 3, 3)
        src_layout.addWidget(QLabel('HBA 轨迹'), 4, 0)
        src_layout.addWidget(self.edit_hba, 4, 1, 1, 1)
        src_layout.addWidget(self.btn_browse_hba, 4, 2)
        src_layout.addWidget(self.chk_roi_from_hba, 4, 3)
        src_layout.addWidget(QLabel('ROI缓冲(m)'), 5, 2)
        src_layout.addWidget(self.spin_roi_buf, 5, 3)

        layout.addWidget(src_group, 0, 0, 1, 2)

        # 参数
        param_group = QGroupBox('转换参数')
        p = QGridLayout(param_group)

        self.spin_res = QDoubleSpinBox(); self.spin_res.setDecimals(3); self.spin_res.setRange(0.01, 0.5); self.spin_res.setValue(0.05)
        self.spin_robot_h = QDoubleSpinBox(); self.spin_robot_h.setDecimals(2); self.spin_robot_h.setRange(0.2, 2.5); self.spin_robot_h.setValue(0.8)
        self.spin_obs_th = QSpinBox(); self.spin_obs_th.setRange(1, 50); self.spin_obs_th.setValue(8)
        self.spin_inflate = QDoubleSpinBox(); self.spin_inflate.setDecimals(2); self.spin_inflate.setRange(0.0, 1.0); self.spin_inflate.setValue(0.15)
        self.spin_min_door = QDoubleSpinBox(); self.spin_min_door.setDecimals(2); self.spin_min_door.setRange(0.0, 5.0); self.spin_min_door.setValue(0.7)
        self.spin_ground_ratio = QDoubleSpinBox(); self.spin_ground_ratio.setDecimals(2); self.spin_ground_ratio.setRange(0.5, 10.0); self.spin_ground_ratio.setValue(2.0)
        self.spin_vert_thresh = QDoubleSpinBox(); self.spin_vert_thresh.setDecimals(2); self.spin_vert_thresh.setRange(0.0, 5.0); self.spin_vert_thresh.setValue(0.8)

        self.chk_disable_pano = QCheckBox('禁用全景分析')
        self.chk_enable_vert = QCheckBox('启用垂直结构约束')
        self.chk_no_require_vert = QCheckBox('不要求垂直跨度判障')

        # 方向与射线雕刻
        self.chk_auto_yaw = QCheckBox('自动主方向对齐(PCA)')
        self.spin_yaw = QDoubleSpinBox(); self.spin_yaw.setDecimals(2); self.spin_yaw.setRange(-180.0, 180.0); self.spin_yaw.setValue(0.0)
        self.chk_ray = QCheckBox('启用射线雕刻')
        self.combo_ray_mode = QComboBox(); self.combo_ray_mode.addItems(['returns','uniform'])
        self.spin_ray_step = QDoubleSpinBox(); self.spin_ray_step.setDecimals(1); self.spin_ray_step.setRange(0.1, 30.0); self.spin_ray_step.setValue(2.0)
        self.spin_ray_range = QDoubleSpinBox(); self.spin_ray_range.setDecimals(1); self.spin_ray_range.setRange(1.0, 200.0); self.spin_ray_range.setValue(30.0)
        self.spin_ray_stride = QSpinBox(); self.spin_ray_stride.setRange(1, 1000); self.spin_ray_stride.setValue(10)

        self.chk_prefilter = QCheckBox('启用预处理(体素+SOR)')
        self.spin_voxel = QDoubleSpinBox(); self.spin_voxel.setDecimals(2); self.spin_voxel.setRange(0.0, 1.0); self.spin_voxel.setValue(0.03)
        self.spin_sor_k = QSpinBox(); self.spin_sor_k.setRange(0, 200); self.spin_sor_k.setValue(20)
        self.spin_sor_std = QDoubleSpinBox(); self.spin_sor_std.setDecimals(2); self.spin_sor_std.setRange(0.1, 5.0); self.spin_sor_std.setValue(1.0)

        self.spin_open_iters = QSpinBox(); self.spin_open_iters.setRange(0, 10); self.spin_open_iters.setValue(1)
        self.spin_min_obs_area = QSpinBox(); self.spin_min_obs_area.setRange(0, 10000); self.spin_min_obs_area.setValue(20)

        row = 0
        p.addWidget(QLabel('分辨率(m/px)'), row, 0); p.addWidget(self.spin_res, row, 1)
        p.addWidget(QLabel('机器人高度(m)'), row, 2); p.addWidget(self.spin_robot_h, row, 3); row += 1
        p.addWidget(QLabel('障碍阈值'), row, 0); p.addWidget(self.spin_obs_th, row, 1)
        p.addWidget(QLabel('膨胀半径(m)'), row, 2); p.addWidget(self.spin_inflate, row, 3); row += 1
        p.addWidget(QLabel('最小门宽(m)'), row, 0); p.addWidget(self.spin_min_door, row, 1)
        p.addWidget(QLabel('地面主导比'), row, 2); p.addWidget(self.spin_ground_ratio, row, 3); row += 1
        p.addWidget(QLabel('垂直阈值'), row, 0); p.addWidget(self.spin_vert_thresh, row, 1)
        p.addWidget(self.chk_disable_pano, row, 2); p.addWidget(self.chk_enable_vert, row, 3); row += 1
        p.addWidget(self.chk_no_require_vert, row, 2); row += 1

        p.addWidget(self.chk_prefilter, row, 0)
        p.addWidget(QLabel('体素(m)'), row, 1); p.addWidget(self.spin_voxel, row, 2)
        p.addWidget(QLabel('SOR-K'), row, 3); p.addWidget(self.spin_sor_k, row, 4)
        p.addWidget(QLabel('SOR-std'), row, 5); p.addWidget(self.spin_sor_std, row, 6); row += 1

        p.addWidget(QLabel('障碍开运算'), row, 0); p.addWidget(self.spin_open_iters, row, 1)
        p.addWidget(QLabel('最小障碍面积(px)'), row, 2); p.addWidget(self.spin_min_obs_area, row, 3); row += 1

        # 方向/射线行
        p.addWidget(self.chk_auto_yaw, row, 0)
        p.addWidget(QLabel('Yaw(度,+逆时针)'), row, 1); p.addWidget(self.spin_yaw, row, 2); row += 1
        p.addWidget(self.chk_ray, row, 0)
        p.addWidget(QLabel('模式'), row, 1); p.addWidget(self.combo_ray_mode, row, 2)
        p.addWidget(QLabel('角步(度)'), row, 3); p.addWidget(self.spin_ray_step, row, 4)
        p.addWidget(QLabel('最大范围(m)'), row, 5); p.addWidget(self.spin_ray_range, row, 6); row += 1
        p.addWidget(QLabel('位姿步长'), row, 5); p.addWidget(self.spin_ray_stride, row, 6); row += 1

        layout.addWidget(param_group, 1, 0)

        # 预设 & 执行
        action_group = QGroupBox('执行')
        a = QGridLayout(action_group)
        self.combo_preset = QComboBox(); self.combo_preset.addItems(['Custom', 'Indoor', 'Outdoor'])
        self.btn_run = QPushButton('开始转换')
        self.edit_output_base = QLineEdit(); self.edit_output_base.setPlaceholderText('输出基名(可选)，不含后缀；默认使用输入名_gridmap')
        self.btn_choose_output = QPushButton('选择输出目录')
        self.edit_output_dir = QLineEdit(str(SAVED_MAPS))

        a.addWidget(QLabel('预设'), 0, 0); a.addWidget(self.combo_preset, 0, 1)
        a.addWidget(QLabel('输出目录'), 0, 2); a.addWidget(self.edit_output_dir, 0, 3); a.addWidget(self.btn_choose_output, 0, 4)
        a.addWidget(QLabel('输出基名'), 1, 0); a.addWidget(self.edit_output_base, 1, 1, 1, 4)
        a.addWidget(self.btn_run, 2, 0, 1, 5)

        layout.addWidget(action_group, 1, 1)

        # 日志 & 预览
        io_group = QGroupBox('日志 / 预览')
        io = QGridLayout(io_group)
        self.log = QTextEdit(); self.log.setReadOnly(True)
        self.preview = QLabel('生成的PGM将显示在此'); self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumHeight(360)

        # 质量指标区域
        metrics_box = QGroupBox('质量指标')
        mlay = QVBoxLayout(metrics_box)
        self.lbl_counts = QLabel('像素计数: -')
        self.lbl_percents = QLabel('占比: -')
        self.lbl_min_width = QLabel('最小门洞净宽: -')
        mlay.addWidget(self.lbl_counts)
        mlay.addWidget(self.lbl_percents)
        mlay.addWidget(self.lbl_min_width)

        io.addWidget(self.log, 0, 0)
        io.addWidget(self.preview, 0, 1)
        io.addWidget(metrics_box, 1, 0, 1, 2)
        layout.addWidget(io_group, 2, 0, 1, 2)

        # 连接信号
        self.btn_refresh.clicked.connect(self.refresh_index)
        self.btn_browse_pcd.clicked.connect(self.browse_pcd)
        self.btn_browse_patches.clicked.connect(self.browse_patches)
        self.btn_browse_poses.clicked.connect(self.browse_poses)
        self.btn_browse_hba.clicked.connect(self.browse_hba)
        self.btn_choose_output.clicked.connect(self.choose_output_dir)
        self.combo_preset.currentTextChanged.connect(self.on_preset)
        self.btn_run.clicked.connect(self.on_run)

        self.refresh_index()
        self.on_preset(self.combo_preset.currentText())

    def append_log(self, text: str):
        self.log.append(text)
        self.log.moveCursor(self.log.textCursor().End)

    def refresh_index(self):
        # 构建索引
        idx_tool = TOOLS_DIR / 'pcd_index.py'
        if idx_tool.exists() and SAVED_MAPS.exists():
            subprocess.run([sys.executable, str(idx_tool), '--root', str(SAVED_MAPS), '--ensure'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        files = load_index()
        self.combo_pcd.clear()
        if not files:
            self.combo_pcd.addItem('(索引为空)')
            return
        # 按修改时间降序
        files = sorted(files, key=lambda x: x['mtime'], reverse=True)
        for f in files[:200]:
            self.combo_pcd.addItem(Path(f['path']).name, f['path'])

    def browse_pcd(self):
        path, _ = QFileDialog.getOpenFileName(self, '选择PCD', str(SAVED_MAPS), 'PCD Files (*.pcd)')
        if path:
            self.edit_pcd.setText(path)

    def browse_patches(self):
        path = QFileDialog.getExistingDirectory(self, '选择patches目录', str(SAVED_MAPS))
        if path:
            self.edit_patches.setText(path)

    def browse_poses(self):
        path, _ = QFileDialog.getOpenFileName(self, '选择poses.txt', str(SAVED_MAPS), 'Text Files (*.txt);;All Files (*)')
        if path:
            self.edit_poses.setText(path)

    def browse_hba(self):
        path, _ = QFileDialog.getOpenFileName(self, '选择HBA轨迹', str(SAVED_MAPS), 'Text Files (*.txt);;All Files (*)')
        if path:
            self.edit_hba.setText(path)

    def choose_output_dir(self):
        path = QFileDialog.getExistingDirectory(self, '选择输出目录', str(SAVED_MAPS))
        if path:
            self.edit_output_dir.setText(path)

    def on_preset(self, name: str):
        if name == 'Indoor':
            self.spin_res.setValue(0.03)
            self.spin_obs_th.setValue(10)
            self.spin_inflate.setValue(0.10)
            self.spin_ground_ratio.setValue(2.5)
            self.spin_min_door.setValue(0.7)
        elif name == 'Outdoor':
            self.spin_res.setValue(0.05)
            self.spin_obs_th.setValue(8)
            self.spin_inflate.setValue(0.15)
            self.spin_ground_ratio.setValue(2.0)
            self.spin_min_door.setValue(0.0)

    def build_command(self):
        script = TOOLS_DIR / 'pcd_to_gridmap.py'
        args = [sys.executable, str(script)]

        # 输入PCD
        pcd = self.edit_pcd.text().strip()
        if not pcd:
            # 用索引当前选择
            data = self.combo_pcd.currentData()
            if data and isinstance(data, str) and data.endswith('.pcd'):
                pcd = data
        if not pcd:
            QMessageBox.warning(self, '提示', '请选择或输入 PCD 文件')
            return []
        args.append(pcd)

        # 输出
        out_dir = self.edit_output_dir.text().strip() or str(SAVED_MAPS)
        base = self.edit_output_base.text().strip()
        if base:
            out_base = str(Path(out_dir) / base)
            args += ['-o', out_base]

        # 常用参数
        args += ['--resolution', str(self.spin_res.value())]
        args += ['--robot-height', str(self.spin_robot_h.value())]
        args += ['--obstacle-thresh', str(self.spin_obs_th.value())]
        args += ['--inflate-radius', str(self.spin_inflate.value())]
        args += ['--ground-dominates-ratio', str(self.spin_ground_ratio.value())]
        args += ['--vertical-thresh', str(self.spin_vert_thresh.value())]
        if self.spin_min_door.value() > 0:
            args += ['--min-door-width', str(self.spin_min_door.value())]
        if self.chk_disable_pano.isChecked():
            args += ['--disable-panoramic']
        if self.chk_enable_vert.isChecked():
            args += ['--enable-vertical-structures']
        if self.chk_no_require_vert.isChecked():
            args += ['--no-require-vertical']

        # 方向/射线
        if self.chk_auto_yaw.isChecked():
            args += ['--auto-yaw-pca']
        if abs(self.spin_yaw.value()) > 0.0001:
            args += ['--yaw-deg', str(self.spin_yaw.value())]
        if self.chk_ray.isChecked():
            args += ['--ray-carve', '--ray-mode', self.combo_ray_mode.currentText(), '--ray-angle-step-deg', str(self.spin_ray_step.value()), '--ray-max-range', str(self.spin_ray_range.value()), '--ray-pose-stride', str(self.spin_ray_stride.value())]

        # 预处理
        if self.chk_prefilter.isChecked():
            args += ['--enable-pre-filter']
            if self.spin_voxel.value() > 0:
                args += ['--voxel-size', str(self.spin_voxel.value())]
            if self.spin_sor_k.value() > 0:
                args += ['--sor-mean-k', str(self.spin_sor_k.value())]
                args += ['--sor-std-ratio', str(self.spin_sor_std.value())]

        # 清理
        if self.spin_open_iters.value() > 0:
            args += ['--obstacle-open-iters', str(self.spin_open_iters.value())]
        if self.spin_min_obs_area.value() > 0:
            args += ['--min-obstacle-area', str(self.spin_min_obs_area.value())]

        # patches / poses / hba
        if self.edit_patches.text().strip():
            args += ['--patches-dir', self.edit_patches.text().strip()]
        if self.edit_poses.text().strip():
            args += ['--poses-file', self.edit_poses.text().strip()]
        if self.edit_hba.text().strip():
            args += ['--hba-poses', self.edit_hba.text().strip()]
        if self.chk_roi_from_hba.isChecked():
            args += ['--roi-from-hba', '--roi-buffer', str(self.spin_roi_buf.value())]

        return args

    def on_run(self):
        if self.process and self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, '执行中', '已有任务在运行，请稍候')
            return

        args = self.build_command()
        if not args:
            return

        self.log.clear()
        self.append_log('运行命令: ' + ' '.join(args))

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.on_proc_output)
        self.process.finished.connect(self.on_proc_finished)
        self.process.start(args[0], args[1:])

    def on_proc_output(self):
        if not self.process:
            return
        out = self.process.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        if out:
            self.append_log(out.rstrip())

    def on_proc_finished(self):
        self.append_log('任务结束')
        # 尝试从日志中解析 YAML 路径
        text = self.log.toPlainText()
        yaml_path = ''
        for line in text.splitlines():
            if '地图元数据已保存为:' in line:
                yaml_path = line.split(':', 1)[1].strip()
                break
        if not yaml_path:
            # 尝试用输出基名推断
            base = self.edit_output_base.text().strip()
            if base:
                cand = Path(self.edit_output_dir.text().strip() or str(SAVED_MAPS)) / (base + '.yaml')
                if cand.exists():
                    yaml_path = str(cand)
        if yaml_path:
            self.show_preview(Path(yaml_path))

    def show_preview(self, yaml_file: Path):
        try:
            import yaml as _y
            with open(yaml_file, 'r', encoding='utf-8') as f:
                meta = _y.safe_load(f)
            pgm = yaml_file.parent / meta['image']
            img = cv2.imread(str(pgm), cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.append_log(f'无法加载PGM: {pgm}')
                return
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            h, w, _ = rgb.shape
            qimg = QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg).scaled(self.preview.width(), self.preview.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.preview.setPixmap(pix)

            # 计算质量指标
            self.update_metrics(img, meta)

            # 若存在metrics json，显示雕刻前后对比
            metrics_json = yaml_file.with_name(yaml_file.stem.replace('_gridmap', '') + '_metrics.json')
            if not metrics_json.exists():
                # 兼容：按输出基名构造
                metrics_json = yaml_file.with_name(yaml_file.stem + '_metrics.json')
            if metrics_json.exists():
                try:
                    with open(metrics_json, 'r', encoding='utf-8') as jf:
                        mj = json.load(jf)
                    if 'pre' in mj and 'post' in mj:
                        total = mj.get('width', img.shape[1]) * mj.get('height', img.shape[0])
                        pre_u = mj['pre']['unknown'] / total * 100.0 if total else 0.0
                        post_u = mj['post']['unknown'] / total * 100.0 if total else 0.0
                        delta = post_u - pre_u
                        self.append_log(f"雕刻未知占比: 之前 {pre_u:.2f}% → 之后 {post_u:.2f}% (Δ {delta:+.2f}%)")
                except Exception as e:
                    self.append_log(f'读取指标JSON失败: {e}')
        except Exception as e:
            self.append_log(f'预览失败: {e}')

    @staticmethod
    def _skeletonize(binary_img: np.ndarray) -> np.ndarray:
        """形态学骨架提取（binary_img为0/1）"""
        img = (binary_img.copy() * 255).astype(np.uint8)
        skel = np.zeros_like(img)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while True:
            opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
            temp = cv2.subtract(img, opened)
            eroded = cv2.erode(img, element)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded
            if cv2.countNonZero(img) == 0:
                break
        return (skel > 0).astype(np.uint8)

    def update_metrics(self, image: np.ndarray, metadata: dict):
        try:
            res = float(metadata.get('resolution', 0.05))
            total = image.size
            obstacles = int(np.sum(image == 0))
            free = int(np.sum(image == 255))
            unknown = int(total - obstacles - free)
            p_obs = obstacles / total * 100.0
            p_free = free / total * 100.0
            p_unk = unknown / total * 100.0
            self.lbl_counts.setText(f'像素计数: 障碍 {obstacles}, 自由 {free}, 未知 {unknown}, 总 {total}')
            self.lbl_percents.setText(f'占比: 障碍 {p_obs:.1f}%, 自由 {p_free:.1f}%, 未知 {p_unk:.1f}%')

            # 估算最小门洞净宽：在自由空间骨架上取 2*距离变换 的最小值
            free_bin = (image == 255).astype(np.uint8)
            if np.any(free_bin):
                dist = cv2.distanceTransform(free_bin, cv2.DIST_L2, 3)
                skel = self._skeletonize(free_bin)
                widths_pix = (dist * 2.0)[skel == 1]
                widths_pix = widths_pix[widths_pix > 0.5]  # 过滤过小噪声
                if widths_pix.size > 0:
                    min_width_m = float(np.min(widths_pix) * res)
                    self.lbl_min_width.setText(f'最小门洞净宽: {min_width_m:.2f} m')
                else:
                    self.lbl_min_width.setText('最小门洞净宽: 无法估计(自由骨架过少)')
            else:
                self.lbl_min_width.setText('最小门洞净宽: 无自由空间')
        except Exception as e:
            self.append_log(f'质量指标计算失败: {e}')


def main():
    app = QApplication(sys.argv)
    gui = GridMapGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
