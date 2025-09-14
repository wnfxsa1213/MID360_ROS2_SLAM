#include "slam_gui/control_panel.h"
#include "slam_gui/ros2_interface.h"  // 包含SystemStatus定义
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>

ControlPanel::ControlPanel(QWidget* parent)
    : QWidget(parent)
    , main_layout_(nullptr)
    , control_tabs_(nullptr)
{
    setupUI();
    connectSignals();
    
    // 初始化定时器
    update_timer_ = new QTimer(this);
    recording_timer_ = new QTimer(this);
    
    connect(update_timer_, &QTimer::timeout, this, &ControlPanel::updateRecordingInfo);
    connect(recording_timer_, &QTimer::timeout, this, &ControlPanel::updateRecordingInfo);
    
    update_timer_->start(1000); // 1秒更新一次
    
    // 初始化状态
    control_state_.slam_running = false;
    control_state_.recording_data = false;
    control_state_.auto_save_enabled = false;
    
    updateButtonStates();
}

ControlPanel::~ControlPanel()
{
    if (update_timer_) update_timer_->stop();
    if (recording_timer_) recording_timer_->stop();
}

void ControlPanel::setupUI()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(5, 5, 5, 5);
    
    // 创建标签页
    control_tabs_ = new QTabWidget(this);
    main_layout_->addWidget(control_tabs_);
    
    // SLAM控制页面
    QWidget* slam_tab = new QWidget();
    QVBoxLayout* slam_layout = new QVBoxLayout(slam_tab);
    
    setupSlamControlGroup();
    setupComponentControlGroup();
    slam_layout->addWidget(slam_control_group_);
    slam_layout->addWidget(component_control_group_);
    slam_layout->addStretch();
    
    control_tabs_->addTab(slam_tab, "SLAM控制");
    
    // 数据管理页面
    QWidget* data_tab = new QWidget();
    QVBoxLayout* data_layout = new QVBoxLayout(data_tab);
    
    setupDataManagementGroup();
    data_layout->addWidget(data_management_group_);
    data_layout->addStretch();
    
    control_tabs_->addTab(data_tab, "数据管理");
    
    // 参数控制页面
    QWidget* param_tab = new QWidget();
    QVBoxLayout* param_layout = new QVBoxLayout(param_tab);
    
    setupParameterControlGroup();
    setupStatusDisplayGroup();
    param_layout->addWidget(parameter_group_);
    param_layout->addWidget(status_group_);
    param_layout->addStretch();
    
    control_tabs_->addTab(param_tab, "参数&状态");
    
    setLayout(main_layout_);
}

void ControlPanel::setupSlamControlGroup()
{
    slam_control_group_ = new QGroupBox("SLAM系统控制");
    QVBoxLayout* layout = new QVBoxLayout(slam_control_group_);
    
    // 主要控制按钮
    QHBoxLayout* main_buttons_layout = new QHBoxLayout();
    
    start_slam_btn_ = new QPushButton("启动SLAM");
    start_slam_btn_->setStyleSheet(getButtonStyleSheet(true));
    main_buttons_layout->addWidget(start_slam_btn_);
    
    stop_slam_btn_ = new QPushButton("停止SLAM");
    stop_slam_btn_->setEnabled(false);
    main_buttons_layout->addWidget(stop_slam_btn_);
    
    layout->addLayout(main_buttons_layout);
    
    QHBoxLayout* aux_buttons_layout = new QHBoxLayout();
    
    pause_slam_btn_ = new QPushButton("暂停");
    pause_slam_btn_->setEnabled(false);
    aux_buttons_layout->addWidget(pause_slam_btn_);
    
    reset_slam_btn_ = new QPushButton("重置");
    aux_buttons_layout->addWidget(reset_slam_btn_);
    
    layout->addLayout(aux_buttons_layout);
    
    // 状态显示
    slam_status_label_ = new QLabel("状态: 未连接");
    slam_status_label_->setStyleSheet("QLabel { color: #ff6b6b; font-weight: bold; }");
    layout->addWidget(slam_status_label_);
    
    slam_progress_ = new QProgressBar();
    slam_progress_->setVisible(false);
    layout->addWidget(slam_progress_);
    
    // 紧急停止按钮
    emergency_stop_btn_ = new QPushButton("🛑 紧急停止");
    emergency_stop_btn_->setStyleSheet("QPushButton { background-color: #dc3545; color: white; font-weight: bold; }");
    layout->addWidget(emergency_stop_btn_);
}

void ControlPanel::setupComponentControlGroup()
{
    component_control_group_ = new QGroupBox("组件控制");
    QGridLayout* layout = new QGridLayout(component_control_group_);
    
    // FastLIO2
    fastlio2_cb_ = new QCheckBox("FastLIO2");
    fastlio2_cb_->setChecked(true);
    layout->addWidget(fastlio2_cb_, 0, 0);
    
    fastlio2_status_ = new QLabel("●");
    fastlio2_status_->setStyleSheet(getStatusStyleSheet(false));
    layout->addWidget(fastlio2_status_, 0, 1);
    
    // PGO
    pgo_cb_ = new QCheckBox("PGO优化");
    pgo_cb_->setChecked(true);
    layout->addWidget(pgo_cb_, 1, 0);
    
    pgo_status_ = new QLabel("●");
    pgo_status_->setStyleSheet(getStatusStyleSheet(false));
    layout->addWidget(pgo_status_, 1, 1);
    
    // HBA
    hba_cb_ = new QCheckBox("HBA优化");
    hba_cb_->setChecked(true);
    layout->addWidget(hba_cb_, 2, 0);
    
    hba_status_ = new QLabel("●");
    hba_status_->setStyleSheet(getStatusStyleSheet(false));
    layout->addWidget(hba_status_, 2, 1);
    
    // Localizer
    localizer_cb_ = new QCheckBox("重定位");
    localizer_cb_->setChecked(true);
    layout->addWidget(localizer_cb_, 3, 0);
    
    localizer_status_ = new QLabel("●");
    localizer_status_->setStyleSheet(getStatusStyleSheet(false));
    layout->addWidget(localizer_status_, 3, 1);
}

void ControlPanel::setupDataManagementGroup()
{
    data_management_group_ = new QGroupBox("数据管理");
    QVBoxLayout* layout = new QVBoxLayout(data_management_group_);
    
    // 地图操作
    QHBoxLayout* map_layout = new QHBoxLayout();
    save_map_btn_ = new QPushButton("保存地图");
    load_map_btn_ = new QPushButton("加载地图");
    map_layout->addWidget(save_map_btn_);
    map_layout->addWidget(load_map_btn_);
    layout->addLayout(map_layout);
    
    export_trajectory_btn_ = new QPushButton("导出轨迹");
    layout->addWidget(export_trajectory_btn_);
    
    layout->addWidget(new QLabel(""));
    
    // 录制控制
    recording_cb_ = new QCheckBox("录制数据");
    layout->addWidget(recording_cb_);
    
    auto_save_cb_ = new QCheckBox("自动保存");
    layout->addWidget(auto_save_cb_);
    
    // 会话管理
    layout->addWidget(new QLabel("会话名称:"));
    session_name_edit_ = new QLineEdit("default_session");
    layout->addWidget(session_name_edit_);
    
    layout->addWidget(new QLabel("保存目录:"));
    QHBoxLayout* dir_layout = new QHBoxLayout();
    save_directory_edit_ = new QLineEdit("./saved_maps");
    browse_directory_btn_ = new QPushButton("浏览");
    dir_layout->addWidget(save_directory_edit_);
    dir_layout->addWidget(browse_directory_btn_);
    layout->addLayout(dir_layout);
    
    // 录制信息
    layout->addWidget(new QLabel("录制信息:"));
    recording_time_label_ = new QLabel("时长: 00:00:00");
    recorded_points_label_ = new QLabel("点数: 0");
    trajectory_length_label_ = new QLabel("轨迹长度: 0.0m");
    
    layout->addWidget(recording_time_label_);
    layout->addWidget(recorded_points_label_);
    layout->addWidget(trajectory_length_label_);
    
    // 快速操作
    layout->addWidget(new QLabel(""));
    quick_save_btn_ = new QPushButton("快速保存");
    quick_save_btn_->setStyleSheet(getButtonStyleSheet(true));
    layout->addWidget(quick_save_btn_);
}

void ControlPanel::setupParameterControlGroup()
{
    parameter_group_ = new QGroupBox("快速参数调整");
    QGridLayout* layout = new QGridLayout(parameter_group_);
    
    // 分辨率参数
    layout->addWidget(new QLabel("扫描分辨率:"), 0, 0);
    scan_resolution_spin_ = new QDoubleSpinBox();
    scan_resolution_spin_->setRange(0.01, 1.0);
    scan_resolution_spin_->setSingleStep(0.01);
    scan_resolution_spin_->setValue(0.15);
    scan_resolution_spin_->setSuffix(" m");
    layout->addWidget(scan_resolution_spin_, 0, 1);
    
    layout->addWidget(new QLabel("地图分辨率:"), 1, 0);
    map_resolution_spin_ = new QDoubleSpinBox();
    map_resolution_spin_->setRange(0.01, 1.0);
    map_resolution_spin_->setSingleStep(0.01);
    map_resolution_spin_->setValue(0.2);
    map_resolution_spin_->setSuffix(" m");
    layout->addWidget(map_resolution_spin_, 1, 1);
    
    layout->addWidget(new QLabel("最大范围:"), 2, 0);
    max_range_spin_ = new QSpinBox();
    max_range_spin_->setRange(10, 200);
    max_range_spin_->setValue(80);
    max_range_spin_->setSuffix(" m");
    layout->addWidget(max_range_spin_, 2, 1);
    
    // 功能开关
    loop_closure_cb_ = new QCheckBox("回环检测");
    loop_closure_cb_->setChecked(true);
    layout->addWidget(loop_closure_cb_, 3, 0, 1, 2);
    
    layout->addWidget(new QLabel("体素大小:"), 4, 0);
    voxel_size_spin_ = new QDoubleSpinBox();
    voxel_size_spin_->setRange(0.1, 2.0);
    voxel_size_spin_->setSingleStep(0.1);
    voxel_size_spin_->setValue(0.5);
    voxel_size_spin_->setSuffix(" m");
    layout->addWidget(voxel_size_spin_, 4, 1);
}

void ControlPanel::setupStatusDisplayGroup()
{
    status_group_ = new QGroupBox("系统状态");
    QGridLayout* layout = new QGridLayout(status_group_);
    
    // CPU使用率
    layout->addWidget(new QLabel("CPU:"), 0, 0);
    cpu_usage_label_ = new QLabel("0.0%");
    layout->addWidget(cpu_usage_label_, 0, 1);
    cpu_progress_ = new QProgressBar();
    cpu_progress_->setMaximumHeight(10);
    layout->addWidget(cpu_progress_, 0, 2);
    
    // 内存使用率
    layout->addWidget(new QLabel("内存:"), 1, 0);
    memory_usage_label_ = new QLabel("0.0%");
    layout->addWidget(memory_usage_label_, 1, 1);
    memory_progress_ = new QProgressBar();
    memory_progress_->setMaximumHeight(10);
    layout->addWidget(memory_progress_, 1, 2);
    
    // 频率
    layout->addWidget(new QLabel("频率:"), 2, 0);
    frequency_label_ = new QLabel("0.0 Hz");
    layout->addWidget(frequency_label_, 2, 1, 1, 2);
    
    // 点云数量
    layout->addWidget(new QLabel("点数:"), 3, 0);
    points_count_label_ = new QLabel("0");
    layout->addWidget(points_count_label_, 3, 1, 1, 2);
    
    // 视图重置按钮
    reset_view_btn_ = new QPushButton("重置视角");
    layout->addWidget(reset_view_btn_, 4, 0, 1, 3);
}

void ControlPanel::connectSignals()
{
    // SLAM控制信号
    connect(start_slam_btn_, &QPushButton::clicked, this, &ControlPanel::onSlamStartClicked);
    connect(stop_slam_btn_, &QPushButton::clicked, this, &ControlPanel::onSlamStopClicked);
    connect(pause_slam_btn_, &QPushButton::clicked, this, &ControlPanel::onSlamPauseClicked);
    connect(reset_slam_btn_, &QPushButton::clicked, this, &ControlPanel::onSlamResetClicked);
    connect(emergency_stop_btn_, &QPushButton::clicked, this, &ControlPanel::onSlamStopClicked);
    
    // 组件控制信号
    connect(fastlio2_cb_, &QCheckBox::toggled, this, &ControlPanel::onFastLIO2Toggled);
    connect(pgo_cb_, &QCheckBox::toggled, this, &ControlPanel::onPGOToggled);
    connect(hba_cb_, &QCheckBox::toggled, this, &ControlPanel::onHBAToggled);
    connect(localizer_cb_, &QCheckBox::toggled, this, &ControlPanel::onLocalizerToggled);
    
    // 数据管理信号
    connect(save_map_btn_, &QPushButton::clicked, this, &ControlPanel::onSaveMapClicked);
    connect(load_map_btn_, &QPushButton::clicked, this, &ControlPanel::onLoadMapClicked);
    connect(export_trajectory_btn_, &QPushButton::clicked, this, &ControlPanel::onExportTrajectoryClicked);
    connect(recording_cb_, &QCheckBox::toggled, this, &ControlPanel::onRecordingToggled);
    connect(auto_save_cb_, &QCheckBox::toggled, this, &ControlPanel::onAutoSaveToggled);
    
    // 会话管理信号
    connect(session_name_edit_, &QLineEdit::textChanged, this, &ControlPanel::onSessionNameChanged);
    connect(save_directory_edit_, &QLineEdit::textChanged, this, &ControlPanel::onSaveDirectoryChanged);
    connect(browse_directory_btn_, &QPushButton::clicked, [this]() {
        QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", save_directory_edit_->text());
        if (!dir.isEmpty()) {
            save_directory_edit_->setText(dir);
        }
    });
    
    // 快速操作信号
    connect(quick_save_btn_, &QPushButton::clicked, this, &ControlPanel::onSaveMapClicked);

    // 参数调整信号连接
    connect(scan_resolution_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        [this](double value) {
            emit parameterChanged("scan_resolution", value);
        });

    connect(map_resolution_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        [this](double value) {
            emit parameterChanged("map_resolution", value);
        });

    connect(max_range_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
        [this](int value) {
            emit parameterChanged("max_range", value);
        });

    connect(voxel_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        [this](double value) {
            emit parameterChanged("voxel_size", value);
        });

    connect(loop_closure_cb_, &QCheckBox::toggled,
        [this](bool enabled) {
            emit parameterChanged("loop_closure_enabled", enabled);
        });
}

SystemControlState ControlPanel::getControlState() const
{
    return control_state_;
}

void ControlPanel::setControlState(const SystemControlState& state)
{
    control_state_ = state;
    updateButtonStates();
    updateStatusIndicators();
}

void ControlPanel::updateSystemStatus()
{
    // 简化版本：暂时不更新实际的系统状态
    updateStatusIndicators();
}

void ControlPanel::updateButtonStates()
{
    start_slam_btn_->setEnabled(!control_state_.slam_running);
    stop_slam_btn_->setEnabled(control_state_.slam_running);
    pause_slam_btn_->setEnabled(control_state_.slam_running);
    
    // 更新状态标签
    if (control_state_.slam_running) {
        slam_status_label_->setText("状态: 运行中");
        slam_status_label_->setStyleSheet("QLabel { color: #51cf66; font-weight: bold; }");
    } else {
        slam_status_label_->setText("状态: 未运行");
        slam_status_label_->setStyleSheet("QLabel { color: #ff6b6b; font-weight: bold; }");
    }
    
    // 更新录制状态
    recording_cb_->setChecked(control_state_.recording_data);
    auto_save_cb_->setChecked(control_state_.auto_save_enabled);

    // 更新组件复选框状态
    fastlio2_cb_->setChecked(control_state_.fastlio2_enabled);
    pgo_cb_->setChecked(control_state_.pgo_enabled);
    hba_cb_->setChecked(control_state_.hba_enabled);
    localizer_cb_->setChecked(control_state_.localizer_enabled);
}

void ControlPanel::updateStatusIndicators()
{
    // 更新组件状态指示器
    fastlio2_status_->setStyleSheet(getStatusStyleSheet(last_system_status_.fastlio2_active, true));
    pgo_status_->setStyleSheet(getStatusStyleSheet(last_system_status_.pgo_active, true));
    hba_status_->setStyleSheet(getStatusStyleSheet(last_system_status_.hba_active, true));
    localizer_status_->setStyleSheet(getStatusStyleSheet(last_system_status_.localizer_active, true));
    
    // 更新性能指标（这里需要从系统监控获取数据）
    frequency_label_->setText(QString("%1 Hz").arg(last_system_status_.fastlio2_frequency, 0, 'f', 1));
}

void ControlPanel::updateRecordingInfo()
{
    if (control_state_.recording_data) {
        control_state_.recording_duration++;
        
        int hours = control_state_.recording_duration / 3600;
        int minutes = (control_state_.recording_duration % 3600) / 60;
        int seconds = control_state_.recording_duration % 60;
        
        recording_time_label_->setText(QString("时长: %1:%2:%3")
            .arg(hours, 2, 10, QChar('0'))
            .arg(minutes, 2, 10, QChar('0'))
            .arg(seconds, 2, 10, QChar('0')));
    }
    
    recorded_points_label_->setText(QString("点数: %1").arg(control_state_.recorded_points));
    trajectory_length_label_->setText(QString("轨迹长度: %1m").arg(control_state_.trajectory_length, 0, 'f', 1));
}

// 槽函数实现
void ControlPanel::onSlamStartClicked()
{
    control_state_.slam_running = true;
    updateButtonStates();
    emit slamStartRequested();
}

void ControlPanel::onSlamStopClicked()
{
    control_state_.slam_running = false;
    control_state_.recording_data = false;
    updateButtonStates();
    emit slamStopRequested();
}

void ControlPanel::onSlamPauseClicked()
{
    emit slamPauseRequested();
}

void ControlPanel::onSlamResetClicked()
{
    control_state_.recording_duration = 0;
    control_state_.recorded_points = 0;
    control_state_.trajectory_length = 0.0;
    updateRecordingInfo();
    emit slamResetRequested();
}

void ControlPanel::onFastLIO2Toggled(bool enabled)
{
    control_state_.fastlio2_enabled = enabled;
    emit componentToggled("fastlio2", enabled);
}

void ControlPanel::onPGOToggled(bool enabled)
{
    control_state_.pgo_enabled = enabled;
    emit componentToggled("pgo", enabled);
}

void ControlPanel::onHBAToggled(bool enabled)
{
    control_state_.hba_enabled = enabled;
    emit componentToggled("hba", enabled);
}

void ControlPanel::onLocalizerToggled(bool enabled)
{
    control_state_.localizer_enabled = enabled;
    emit componentToggled("localizer", enabled);
}

void ControlPanel::onSaveMapClicked()
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString default_filename = QString("%1/%2_%3.pcd")
        .arg(control_state_.save_directory)
        .arg(control_state_.current_session_name)
        .arg(timestamp);
    
    QString filename = QFileDialog::getSaveFileName(this,
        "保存地图", default_filename, "PCD Files (*.pcd);;All Files (*)");
    
    if (!filename.isEmpty()) {
        emit mapSaveRequested(filename, true, hba_cb_->isChecked());
    }
}

void ControlPanel::onLoadMapClicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
        "加载地图", control_state_.save_directory, "PCD Files (*.pcd);;All Files (*)");
    
    if (!filename.isEmpty()) {
        emit mapLoadRequested(filename);
    }
}

void ControlPanel::onExportTrajectoryClicked()
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString default_filename = QString("%1/%2_trajectory_%3.txt")
        .arg(control_state_.save_directory)
        .arg(control_state_.current_session_name)
        .arg(timestamp);
    
    QString filename = QFileDialog::getSaveFileName(this,
        "导出轨迹", default_filename, "Text Files (*.txt);;All Files (*)");
    
    if (!filename.isEmpty()) {
        emit trajectoryExportRequested(filename);
    }
}

void ControlPanel::onRecordingToggled(bool enabled)
{
    control_state_.recording_data = enabled;
    if (enabled) {
        recording_timer_->start(1000); // 每秒更新
    } else {
        recording_timer_->stop();
    }
    emit recordingToggled(enabled);
}

void ControlPanel::onAutoSaveToggled(bool enabled)
{
    control_state_.auto_save_enabled = enabled;
    emit autoSaveToggled(enabled);
}

void ControlPanel::onSessionNameChanged()
{
    control_state_.current_session_name = session_name_edit_->text();
    emit sessionChanged(control_state_.current_session_name);
}

void ControlPanel::onSaveDirectoryChanged()
{
    control_state_.save_directory = save_directory_edit_->text();
}

QString ControlPanel::getStatusStyleSheet(bool is_active, bool is_healthy) const
{
    if (is_active && is_healthy) {
        return "QLabel { color: #51cf66; font-size: 16px; }"; // 绿色
    } else if (is_active && !is_healthy) {
        return "QLabel { color: #ffd43b; font-size: 16px; }"; // 黄色
    } else {
        return "QLabel { color: #ff6b6b; font-size: 16px; }"; // 红色
    }
}

QString ControlPanel::getButtonStyleSheet(bool is_primary) const
{
    if (is_primary) {
        return "QPushButton { background-color: #339af0; color: white; font-weight: bold; padding: 8px; }";
    } else {
        return "QPushButton { padding: 6px; }";
    }
}

// MOC文件由CMake自动处理