#include "slam_gui/main_window.h"
#include <QApplication>
#include <QCloseEvent>
#include <QResizeEvent>
#include <QSplitter>
#include <QDockWidget>
#include <QHeaderView>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QProcessEnvironment>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , central_widget_(nullptr)
    , main_splitter_(nullptr)
    , right_splitter_(nullptr)
    , visualization_widget_(nullptr)
    , control_panel_(nullptr)
    , log_viewer_(nullptr)
    , monitor_tab_widget_(nullptr)
    , status_tree_(nullptr)
    , performance_table_(nullptr)
    , diagnostics_text_(nullptr)
    , menu_bar_(nullptr)
    , main_toolbar_(nullptr)
    , slam_toolbar_(nullptr)
    , status_bar_(nullptr)
    , is_fullscreen_(false)
    , slam_running_(false)
{
    // 设置窗口属性
    setWindowTitle("SLAM可视化控制中心 - Mid360 LiDAR系统");
    setMinimumSize(1200, 800);
    resize(1600, 1000);
    
    // 初始化设置
    settings_ = new QSettings("SLAM_GUI", "MainWindow", this);

    // 初始化核心组件
    ros2_interface_ = std::make_unique<ROS2Interface>(this);
    system_monitor_ = std::make_unique<SystemMonitor>(this);
    config_manager_ = std::make_unique<ConfigManager>(this);

    // 设置UI（必须在validateWorkspaceEnvironment之前，因为需要log_viewer_）
    setupUI();
    connectSignals();
    loadSettings();

    // 验证工作环境（现在log_viewer_已经初始化了）
    if (!validateWorkspaceEnvironment()) {
        showErrorMessage("环境错误", "当前目录不是有效的ROS2工作空间，请切换到ws_livox目录");
    }
    
    // 初始化定时器
    ui_update_timer_ = new QTimer(this);
    status_update_timer_ = new QTimer(this);
    
    connect(ui_update_timer_, &QTimer::timeout, this, &MainWindow::updateUI);
    connect(status_update_timer_, &QTimer::timeout, this, &MainWindow::updateStatusBar);
    
    ui_update_timer_->start(100); // 10Hz UI更新
    status_update_timer_->start(1000); // 1Hz状态更新
    
    // 初始化ROS2接口
    if (ros2_interface_->initialize("slam_gui_main")) {
        ros2_interface_->startDataCollection();
        showInfoMessage("系统启动", "SLAM GUI已成功启动，ROS2接口连接正常");
    } else {
        showErrorMessage("启动错误", "ROS2接口初始化失败，请检查ROS2环境");
    }

    // 启动系统监控
    system_monitor_->startMonitoring();

    // 设置进程管理
    setupProcessTimeouts();

    // 检测SLAM系统状态
    detectSlamStatus();
}

MainWindow::~MainWindow()
{
    saveSettings();
    
    if (ros2_interface_) {
        ros2_interface_->shutdown();
    }
}

void MainWindow::setupUI()
{
    setupMenuBar();
    setupToolBar();
    setupStatusBar();
    setupCentralWidget();
}

void MainWindow::setupMenuBar()
{
    menu_bar_ = menuBar();
    
    // 文件菜单
    file_menu_ = menu_bar_->addMenu("文件(&F)");
    
    action_new_ = file_menu_->addAction("新建(&N)");
    action_new_->setShortcut(QKeySequence::New);
    action_new_->setStatusTip("创建新的SLAM会话");
    
    action_open_ = file_menu_->addAction("打开(&O)");
    action_open_->setShortcut(QKeySequence::Open);
    action_open_->setStatusTip("打开已保存的地图文件");
    
    file_menu_->addSeparator();
    
    action_save_ = file_menu_->addAction("保存(&S)");
    action_save_->setShortcut(QKeySequence::Save);
    action_save_->setStatusTip("保存当前地图");
    
    action_save_as_ = file_menu_->addAction("另存为(&A)");
    action_save_as_->setShortcut(QKeySequence::SaveAs);
    action_save_as_->setStatusTip("将地图保存到指定位置");
    
    file_menu_->addSeparator();
    
    action_exit_ = file_menu_->addAction("退出(&X)");
    action_exit_->setShortcut(QKeySequence::Quit);
    action_exit_->setStatusTip("退出SLAM GUI");
    
    // 编辑菜单
    edit_menu_ = menu_bar_->addMenu("编辑(&E)");
    
    action_preferences_ = edit_menu_->addAction("首选项(&P)");
    action_preferences_->setStatusTip("打开系统配置设置");
    
    // 视图菜单
    view_menu_ = menu_bar_->addMenu("视图(&V)");
    
    action_fullscreen_ = view_menu_->addAction("全屏(&F)");
    action_fullscreen_->setShortcut(QKeySequence::FullScreen);
    action_fullscreen_->setCheckable(true);
    action_fullscreen_->setStatusTip("切换全屏模式");
    
    view_menu_->addSeparator();
    
    action_reset_layout_ = view_menu_->addAction("重置布局(&R)");
    action_reset_layout_->setStatusTip("恢复默认窗口布局");
    
    view_menu_->addSeparator();
    
    action_show_control_panel_ = view_menu_->addAction("显示控制面板(&C)");
    action_show_control_panel_->setCheckable(true);
    action_show_control_panel_->setChecked(true);
    
    action_show_monitor_ = view_menu_->addAction("显示监控面板(&M)");
    action_show_monitor_->setCheckable(true);
    action_show_monitor_->setChecked(true);
    
    action_show_log_ = view_menu_->addAction("显示日志面板(&L)");
    action_show_log_->setCheckable(true);
    action_show_log_->setChecked(true);
    
    // SLAM菜单
    slam_menu_ = menu_bar_->addMenu("SLAM(&S)");
    
    action_slam_start_ = slam_menu_->addAction("启动SLAM(&S)");
    action_slam_start_->setStatusTip("启动SLAM系统");
    
    action_slam_stop_ = slam_menu_->addAction("停止SLAM(&T)");
    action_slam_stop_->setStatusTip("停止SLAM系统");
    action_slam_stop_->setEnabled(false);
    
    action_slam_pause_ = slam_menu_->addAction("暂停SLAM(&P)");
    action_slam_pause_->setStatusTip("暂停SLAM处理");
    action_slam_pause_->setEnabled(false);
    
    action_slam_reset_ = slam_menu_->addAction("重置SLAM(&R)");
    action_slam_reset_->setStatusTip("重置SLAM系统");
    
    slam_menu_->addSeparator();
    
    action_slam_save_map_ = slam_menu_->addAction("保存地图(&M)");
    action_slam_save_map_->setStatusTip("保存当前建立的地图");
    
    action_slam_load_map_ = slam_menu_->addAction("加载地图(&L)");
    action_slam_load_map_->setStatusTip("加载已保存的地图用于定位");
    
    // 帮助菜单
    help_menu_ = menu_bar_->addMenu("帮助(&H)");
    
    action_user_guide_ = help_menu_->addAction("用户指南(&G)");
    action_user_guide_->setStatusTip("打开用户操作指南");
    
    help_menu_->addSeparator();
    
    action_about_ = help_menu_->addAction("关于(&A)");
    action_about_->setStatusTip("关于SLAM GUI");
}

void MainWindow::setupToolBar()
{
    // 主工具栏
    main_toolbar_ = addToolBar("主工具栏");
    main_toolbar_->setObjectName("MainToolBar");
    
    main_toolbar_->addAction(action_new_);
    main_toolbar_->addAction(action_open_);
    main_toolbar_->addAction(action_save_);
    main_toolbar_->addSeparator();
    main_toolbar_->addAction(action_preferences_);
    
    // SLAM控制工具栏
    slam_toolbar_ = addToolBar("SLAM控制");
    slam_toolbar_->setObjectName("SLAMToolBar");
    
    slam_toolbar_->addAction(action_slam_start_);
    slam_toolbar_->addAction(action_slam_stop_);
    slam_toolbar_->addAction(action_slam_pause_);
    slam_toolbar_->addAction(action_slam_reset_);
    slam_toolbar_->addSeparator();
    slam_toolbar_->addAction(action_slam_save_map_);
    slam_toolbar_->addAction(action_slam_load_map_);
}

void MainWindow::setupStatusBar()
{
    status_bar_ = statusBar();
    
    // SLAM状态标签
    status_slam_label_ = new QLabel("SLAM: 未连接");
    status_slam_label_->setMinimumWidth(120);
    status_bar_->addWidget(status_slam_label_);
    
    status_bar_->addWidget(new QLabel("|"));
    
    // 节点状态标签
    status_nodes_label_ = new QLabel("节点: 0/4");
    status_nodes_label_->setMinimumWidth(80);
    status_bar_->addWidget(status_nodes_label_);
    
    status_bar_->addWidget(new QLabel("|"));
    
    // 频率显示
    status_frequency_label_ = new QLabel("频率: 0.0Hz");
    status_frequency_label_->setMinimumWidth(100);
    status_bar_->addWidget(status_frequency_label_);
    
    // 进度条
    status_progress_bar_ = new QProgressBar();
    status_progress_bar_->setMaximumWidth(200);
    status_progress_bar_->setVisible(false);
    status_bar_->addPermanentWidget(status_progress_bar_);
    
    status_bar_->showMessage("就绪", 2000);
}

void MainWindow::setupCentralWidget()
{
    central_widget_ = new QWidget();
    setCentralWidget(central_widget_);
    
    // 创建主分割器 (水平分割)
    main_splitter_ = new QSplitter(Qt::Horizontal, central_widget_);
    
    // 创建可视化组件
    visualization_widget_ = new VisualizationWidget(this);
    main_splitter_->addWidget(visualization_widget_);
    
    // 创建右侧分割器 (垂直分割)
    right_splitter_ = new QSplitter(Qt::Vertical);
    main_splitter_->addWidget(right_splitter_);
    
    // 创建控制面板
    control_panel_ = new ControlPanel(this);
    right_splitter_->addWidget(control_panel_);
    
    // 创建监控标签页
    monitor_tab_widget_ = new QTabWidget();
    setupMonitorTabs();
    right_splitter_->addWidget(monitor_tab_widget_);
    
    // 创建日志查看器
    log_viewer_ = new LogViewer(this);
    right_splitter_->addWidget(log_viewer_);
    
    // 设置分割器比例
    main_splitter_->setSizes({1000, 400}); // 左侧可视化区域更大
    right_splitter_->setSizes({200, 300, 200}); // 控制面板、监控、日志
    
    // 设置中心布局
    QHBoxLayout* layout = new QHBoxLayout(central_widget_);
    layout->addWidget(main_splitter_);
    layout->setContentsMargins(2, 2, 2, 2);
}

void MainWindow::setupMonitorTabs()
{
    // 状态监控标签页
    status_monitor_widget_ = new QWidget();
    QVBoxLayout* status_layout = new QVBoxLayout(status_monitor_widget_);
    
    status_tree_ = new QTreeWidget();
    status_tree_->setHeaderLabels({"项目", "状态", "值"});
    status_tree_->header()->setStretchLastSection(true);
    status_layout->addWidget(status_tree_);
    
    monitor_tab_widget_->addTab(status_monitor_widget_, "系统状态");
    
    // 性能监控标签页
    performance_monitor_widget_ = new QWidget();
    QVBoxLayout* perf_layout = new QVBoxLayout(performance_monitor_widget_);
    
    performance_table_ = new QTableWidget(0, 3);
    performance_table_->setHorizontalHeaderLabels({"指标", "当前值", "平均值"});
    performance_table_->horizontalHeader()->setStretchLastSection(true);
    perf_layout->addWidget(performance_table_);
    
    monitor_tab_widget_->addTab(performance_monitor_widget_, "性能指标");
    
    // 诊断信息标签页
    diagnostics_monitor_widget_ = new QWidget();
    QVBoxLayout* diag_layout = new QVBoxLayout(diagnostics_monitor_widget_);
    
    diagnostics_text_ = new QTextEdit();
    diagnostics_text_->setReadOnly(true);
    diagnostics_text_->setFont(QFont("Consolas", 9));
    diag_layout->addWidget(diagnostics_text_);
    
    monitor_tab_widget_->addTab(diagnostics_monitor_widget_, "诊断信息");
}

void MainWindow::connectSignals()
{
    // ROS2接口信号连接
    connect(ros2_interface_.get(), &ROS2Interface::newSLAMData,
            this, &MainWindow::onNewSLAMData);
    connect(ros2_interface_.get(), &ROS2Interface::systemStatusChanged,
            this, &MainWindow::onSystemStatusChanged);
    connect(ros2_interface_.get(), &ROS2Interface::errorOccurred,
            this, &MainWindow::onErrorOccurred);
    connect(ros2_interface_.get(), &ROS2Interface::serviceCallCompleted,
            this, &MainWindow::onServiceCallCompleted);
    
    // 菜单动作连接
    connect(action_new_, &QAction::triggered, this, &MainWindow::onFileNew);
    connect(action_open_, &QAction::triggered, this, &MainWindow::onFileOpen);
    connect(action_save_, &QAction::triggered, this, &MainWindow::onFileSave);
    connect(action_save_as_, &QAction::triggered, this, &MainWindow::onFileSaveAs);
    connect(action_exit_, &QAction::triggered, this, &MainWindow::onFileExit);
    
    connect(action_preferences_, &QAction::triggered, this, &MainWindow::onEditPreferences);
    
    connect(action_fullscreen_, &QAction::triggered, this, &MainWindow::onViewFullscreen);
    connect(action_reset_layout_, &QAction::triggered, this, &MainWindow::onViewResetLayout);
    
    connect(action_slam_start_, &QAction::triggered, this, &MainWindow::onSlamStart);
    connect(action_slam_stop_, &QAction::triggered, this, &MainWindow::onSlamStop);
    connect(action_slam_pause_, &QAction::triggered, this, &MainWindow::onSlamPause);
    connect(action_slam_reset_, &QAction::triggered, this, &MainWindow::onSlamReset);
    connect(action_slam_save_map_, &QAction::triggered, this, &MainWindow::onSlamSaveMap);
    connect(action_slam_load_map_, &QAction::triggered, this, &MainWindow::onSlamLoadMap);
    
    connect(action_about_, &QAction::triggered, this, &MainWindow::onHelpAbout);
    connect(action_user_guide_, &QAction::triggered, this, &MainWindow::onHelpUserGuide);

    // 控制面板信号连接 - 这些连接实现了GUI控制与后端系统的集成
    if (control_panel_) {
        connect(control_panel_, &ControlPanel::slamStartRequested, this, &MainWindow::onSlamStart);
        connect(control_panel_, &ControlPanel::slamStopRequested, this, &MainWindow::onSlamStop);
        connect(control_panel_, &ControlPanel::slamPauseRequested, this, &MainWindow::onSlamPause);
        connect(control_panel_, &ControlPanel::slamResetRequested, this, &MainWindow::onSlamReset);

        connect(control_panel_, &ControlPanel::mapSaveRequested, this, &MainWindow::onControlPanelSaveMap);
        connect(control_panel_, &ControlPanel::mapLoadRequested, this, &MainWindow::onControlPanelLoadMap);
        connect(control_panel_, &ControlPanel::trajectoryExportRequested, this, &MainWindow::onControlPanelExportTrajectory);

        connect(control_panel_, &ControlPanel::componentToggled, this, &MainWindow::onComponentToggled);
        connect(control_panel_, &ControlPanel::recordingToggled, this, &MainWindow::onRecordingToggled);
        connect(control_panel_, &ControlPanel::autoSaveToggled, this, &MainWindow::onAutoSaveToggled);

        // 参数变化信号连接
        connect(control_panel_, &ControlPanel::parameterChanged, this,
                [this](const QString& parameter_name, const QVariant& value) {
                    QString message = QString("参数 %1 已更改为: %2").arg(parameter_name, value.toString());
                    log_viewer_->addLogMessage(LogViewer::Info, message);

                    // 将参数变化应用到ROS2节点
                    synchronizeParameterToNodes(parameter_name, value);
                });
    }

    // 系统监控信号连接
    if (system_monitor_) {
        connect(system_monitor_.get(), &SystemMonitor::metricsUpdated, this, &MainWindow::onSystemMetricsUpdated);
        connect(system_monitor_.get(), &SystemMonitor::processStatusChanged, this, &MainWindow::onProcessStatusChanged);
        connect(system_monitor_.get(), &SystemMonitor::systemAlertTriggered, this, &MainWindow::onSystemAlertTriggered);
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (slam_running_) {
        if (!askConfirmation("退出确认", "SLAM系统正在运行，确定要退出吗？")) {
            event->ignore();
            return;
        }
    }
    
    saveSettings();
    event->accept();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    // 可以在这里添加自定义调整逻辑
}

void MainWindow::loadSettings()
{
    restoreGeometry(settings_->value("geometry").toByteArray());
    restoreState(settings_->value("windowState").toByteArray());
    
    // 保存默认状态
    default_geometry_ = saveGeometry();
    default_state_ = saveState();
}

void MainWindow::saveSettings()
{
    settings_->setValue("geometry", saveGeometry());
    settings_->setValue("windowState", saveState());
}

void MainWindow::updateUI()
{
    // 定期更新UI组件
    if (visualization_widget_) {
        visualization_widget_->update();
    }
}

void MainWindow::updateStatusBar()
{
    auto status = ros2_interface_->getSystemStatus();
    
    // 更新SLAM状态
    if (status.fastlio2_active) {
        status_slam_label_->setText("SLAM: 运行中");
        status_slam_label_->setStyleSheet("color: green;");
    } else {
        status_slam_label_->setText("SLAM: 未运行");
        status_slam_label_->setStyleSheet("color: red;");
    }
    
    // 更新节点计数
    int active_nodes = 0;
    if (status.fastlio2_active) active_nodes++;
    if (status.pgo_active) active_nodes++;
    if (status.hba_active) active_nodes++;
    if (status.localizer_active) active_nodes++;
    
    status_nodes_label_->setText(QString("节点: %1/4").arg(active_nodes));
    
    // 更新频率显示
    status_frequency_label_->setText(QString("频率: %1Hz").arg(status.fastlio2_frequency, 0, 'f', 1));
}

// 实现剩余的槽函数...
void MainWindow::onNewSLAMData(const SLAMData& data)
{
    if (visualization_widget_) {
        visualization_widget_->updateSLAMData(data);
    }
}

void MainWindow::onSystemStatusChanged(const SystemStatus& status)
{
    // 更新状态树显示系统状态
    if (status_tree_) {
        status_tree_->clear();

        auto* fastlio2_item = new QTreeWidgetItem(status_tree_);
        fastlio2_item->setText(0, "FastLIO2");
        fastlio2_item->setText(1, status.fastlio2_active ? "运行中" : "已停止");
        fastlio2_item->setText(2, QString("%1 Hz").arg(status.fastlio2_frequency, 0, 'f', 1));
        fastlio2_item->setForeground(1, status.fastlio2_active ? Qt::green : Qt::red);

        auto* pgo_item = new QTreeWidgetItem(status_tree_);
        pgo_item->setText(0, "PGO");
        pgo_item->setText(1, status.pgo_active ? "运行中" : "已停止");
        pgo_item->setText(2, QString("%1 Hz").arg(status.pgo_frequency, 0, 'f', 1));
        pgo_item->setForeground(1, status.pgo_active ? Qt::green : Qt::red);

        auto* hba_item = new QTreeWidgetItem(status_tree_);
        hba_item->setText(0, "HBA");
        hba_item->setText(1, status.hba_active ? "运行中" : "已停止");
        hba_item->setForeground(1, status.hba_active ? Qt::green : Qt::red);

        auto* localizer_item = new QTreeWidgetItem(status_tree_);
        localizer_item->setText(0, "Localizer");
        localizer_item->setText(1, status.localizer_active ? "运行中" : "已停止");
        localizer_item->setForeground(1, status.localizer_active ? Qt::green : Qt::red);
    }
}

void MainWindow::onErrorOccurred(const QString& error_message)
{
    showErrorMessage("系统错误", error_message);
    log_viewer_->addLogMessage(LogViewer::Error, error_message);
}

void MainWindow::onServiceCallCompleted(const QString& service_name, bool success, const QString& message)
{
    QString log_message = QString("%1: %2").arg(service_name, message);
    if (success) {
        log_viewer_->addLogMessage(LogViewer::Info, log_message);
    } else {
        log_viewer_->addLogMessage(LogViewer::Warning, log_message);
    }
}

// 菜单动作实现
void MainWindow::onFileNew() { /* 实现新建功能 */ }
void MainWindow::onFileOpen() { /* 实现打开功能 */ }
void MainWindow::onFileSave() { /* 实现保存功能 */ }
void MainWindow::onFileSaveAs() { /* 实现另存为功能 */ }
void MainWindow::onFileExit() { close(); }

void MainWindow::onEditPreferences() { /* 实现首选项对话框 */ }

void MainWindow::onViewFullscreen() 
{
    if (is_fullscreen_) {
        showNormal();
    } else {
        showFullScreen();
    }
    is_fullscreen_ = !is_fullscreen_;
}

void MainWindow::onViewResetLayout() 
{
    restoreGeometry(default_geometry_);
    restoreState(default_state_);
}

void MainWindow::onViewShowControlPanel() { /* 实现面板显示控制 */ }
void MainWindow::onViewShowMonitor() { /* 实现面板显示控制 */ }
void MainWindow::onViewShowLog() { /* 实现面板显示控制 */ }

void MainWindow::onSlamStart()
{
    // 通过launch文件启动SLAM系统
    log_viewer_->addLogMessage(LogViewer::Info, "正在启动SLAM系统...");

    QProcess* launch_process = new QProcess(this);

    // 获取工作空间路径并设置环境
    QString workspace_path = getWorkspacePath();
    if (workspace_path.isEmpty()) {
        log_viewer_->addLogMessage(LogViewer::Error, "无法确定ROS2工作空间路径，SLAM启动失败");
        launch_process->deleteLater();
        return;
    }

    setupROS2Environment(launch_process, workspace_path);

    launch_process->setProgram("ros2");
    launch_process->setArguments({"launch", "fastlio2", "enhanced_visualization.launch.py"});

    connect(launch_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, launch_process](int exitCode, QProcess::ExitStatus exitStatus) {
            if (exitStatus == QProcess::NormalExit && exitCode == 0) {
                slam_running_ = true;
                action_slam_start_->setEnabled(false);
                action_slam_stop_->setEnabled(true);
                action_slam_pause_->setEnabled(true);
                log_viewer_->addLogMessage(LogViewer::Info, "SLAM系统启动成功");

                // 启动后延迟更新组件状态
                QTimer::singleShot(3000, this, [this]() {
                    updateComponentStatus();
                });
            } else {
                log_viewer_->addLogMessage(LogViewer::Error, "SLAM系统启动失败");
            }
            launch_process->deleteLater();
        });

    connect(launch_process, &QProcess::errorOccurred,
        [this, launch_process](QProcess::ProcessError error) {
            QString errorMsg = QString("SLAM启动进程错误: %1").arg(error);
            log_viewer_->addLogMessage(LogViewer::Error, errorMsg);
            launch_process->deleteLater();
        });

    launch_process->start();

    // 更新UI状态
    action_slam_start_->setEnabled(false);
    status_progress_bar_->setVisible(true);
    status_progress_bar_->setRange(0, 0); // 不确定进度
}

void MainWindow::onSlamStop()
{
    log_viewer_->addLogMessage(LogViewer::Info, "正在停止SLAM系统...");

    // 终止SLAM相关节点
    QProcess* stop_process = new QProcess(this);
    stop_process->setProgram("pkill");
    stop_process->setArguments({"-f", "fastlio2"});

    connect(stop_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, stop_process](int exitCode, QProcess::ExitStatus exitStatus) {
            Q_UNUSED(exitCode); // pkill的返回码不一定可靠
            Q_UNUSED(exitStatus);
            slam_running_ = false;
            action_slam_start_->setEnabled(true);
            action_slam_stop_->setEnabled(false);
            action_slam_pause_->setEnabled(false);
            status_progress_bar_->setVisible(false);
            log_viewer_->addLogMessage(LogViewer::Info, "SLAM系统已停止");

            // 停止后延迟更新组件状态
            QTimer::singleShot(1000, this, [this]() {
                updateComponentStatus();
            });

            stop_process->deleteLater();
        });

    connect(stop_process, &QProcess::errorOccurred,
        [this, stop_process](QProcess::ProcessError error) {
            QString errorMsg = QString("SLAM停止进程错误: %1").arg(error);
            log_viewer_->addLogMessage(LogViewer::Warning, errorMsg);
            stop_process->deleteLater();
        });

    stop_process->start();
}

void MainWindow::onSlamPause() { /* 实现暂停功能 */ }
void MainWindow::onSlamReset() { /* 实现重置功能 */ }

void MainWindow::onSlamSaveMap() 
{
    QString fileName = QFileDialog::getSaveFileName(this, 
        "保存地图", "", "PCD Files (*.pcd);;All Files (*)");
    
    if (!fileName.isEmpty()) {
        status_progress_bar_->setVisible(true);
        ros2_interface_->saveMap(fileName.toStdString());
    }
}

void MainWindow::onSlamLoadMap() { /* 实现地图加载功能 */ }

void MainWindow::onHelpAbout() 
{
    QMessageBox::about(this, "关于SLAM GUI", 
        "SLAM可视化控制中心 v1.0\n\n"
        "基于Qt5和ROS2的Mid360 LiDAR SLAM系统\n"
        "提供实时可视化、系统监控和操控功能\n\n"
        "开发: SLAM团队");
}

void MainWindow::onHelpUserGuide() { /* 实现用户指南 */ }

void MainWindow::showErrorMessage(const QString& title, const QString& message)
{
    QMessageBox::critical(this, title, message);
}

void MainWindow::showInfoMessage(const QString& title, const QString& message)
{
    QMessageBox::information(this, title, message);
}

bool MainWindow::askConfirmation(const QString& title, const QString& message)
{
    return QMessageBox::question(this, title, message,
        QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
}

// 控制面板槽函数实现
void MainWindow::onControlPanelSaveMap(const QString& file_path, bool save_patches, bool enable_hba_refine)
{
    status_progress_bar_->setVisible(true);
    status_progress_bar_->setRange(0, 0); // 不确定进度

    if (ros2_interface_->saveMap(file_path.toStdString(), save_patches, enable_hba_refine)) {
        log_viewer_->addLogMessage(LogViewer::Info, QString("地图保存请求已发送: %1").arg(file_path));
    } else {
        status_progress_bar_->setVisible(false);
        showErrorMessage("保存失败", "无法调用地图保存服务");
    }
}

void MainWindow::onControlPanelLoadMap(const QString& file_path)
{
    log_viewer_->addLogMessage(LogViewer::Info, QString("加载地图: %1").arg(file_path));
    // TODO: 实现地图加载ROS服务调用
}

void MainWindow::onControlPanelExportTrajectory(const QString& file_path)
{
    if (ros2_interface_->savePoses(file_path.toStdString())) {
        log_viewer_->addLogMessage(LogViewer::Info, QString("轨迹导出请求已发送: %1").arg(file_path));
    } else {
        showErrorMessage("导出失败", "无法调用轨迹保存服务");
    }
}

void MainWindow::onComponentToggled(const QString& component_name, bool enabled)
{
    QString message = QString("正在%1%2组件...").arg(enabled ? "启动" : "停止", component_name);
    log_viewer_->addLogMessage(LogViewer::Info, message);

    if (enabled) {
        // 启动组件
        startSLAMComponent(component_name);
    } else {
        // 停止组件
        stopSLAMComponent(component_name);
    }
}

void MainWindow::onRecordingToggled(bool enabled)
{
    if (enabled) {
        log_viewer_->addLogMessage(LogViewer::Info, "开始录制SLAM数据");
        // TODO: 开始数据录制
    } else {
        log_viewer_->addLogMessage(LogViewer::Info, "停止录制SLAM数据");
        // TODO: 停止数据录制
    }
}

void MainWindow::onAutoSaveToggled(bool enabled)
{
    QString message = QString("自动保存功能已%1").arg(enabled ? "启用" : "禁用");
    log_viewer_->addLogMessage(LogViewer::Info, message);
    // TODO: 实现自动保存逻辑
}

// 系统监控槽函数实现
void MainWindow::onSystemMetricsUpdated(const SystemMetrics& metrics)
{
    // 更新性能表格
    if (performance_table_) {
        performance_table_->clearContents();
        performance_table_->setRowCount(6);

        performance_table_->setItem(0, 0, new QTableWidgetItem("CPU使用率"));
        performance_table_->setItem(0, 1, new QTableWidgetItem(QString("%1%").arg(metrics.total_cpu_usage, 0, 'f', 1)));

        performance_table_->setItem(1, 0, new QTableWidgetItem("内存使用率"));
        performance_table_->setItem(1, 1, new QTableWidgetItem(QString("%1%").arg(metrics.total_memory_usage, 0, 'f', 1)));

        performance_table_->setItem(2, 0, new QTableWidgetItem("可用内存"));
        performance_table_->setItem(2, 1, new QTableWidgetItem(QString("%1 GB").arg(metrics.available_memory_gb, 0, 'f', 1)));

        performance_table_->setItem(3, 0, new QTableWidgetItem("磁盘使用率"));
        performance_table_->setItem(3, 1, new QTableWidgetItem(QString("%1%").arg(metrics.disk_usage, 0, 'f', 1)));

        performance_table_->setItem(4, 0, new QTableWidgetItem("网络发送"));
        performance_table_->setItem(4, 1, new QTableWidgetItem(QString("%1 KB/s").arg(metrics.network_tx_rate / 1024.0, 0, 'f', 1)));

        performance_table_->setItem(5, 0, new QTableWidgetItem("网络接收"));
        performance_table_->setItem(5, 1, new QTableWidgetItem(QString("%1 KB/s").arg(metrics.network_rx_rate / 1024.0, 0, 'f', 1)));
    }

    // 将最新的系统状态传递给控制面板
    if (control_panel_) {
        control_panel_->updateSystemStatus();
    }

    // 更新组件状态指示
    updateComponentStatus();
}

void MainWindow::onProcessStatusChanged(const QString& process_name, bool is_running)
{
    QString status_text = is_running ? "运行中" : "已停止";
    log_viewer_->addLogMessage(LogViewer::Info, QString("进程 %1: %2").arg(process_name, status_text));

    // 更新状态树
    if (status_tree_) {
        auto items = status_tree_->findItems(process_name, Qt::MatchExactly);
        if (!items.isEmpty()) {
            items[0]->setText(1, status_text);
            items[0]->setForeground(1, is_running ? Qt::green : Qt::red);
        } else {
            // 添加新的进程状态项
            auto* item = new QTreeWidgetItem(status_tree_);
            item->setText(0, process_name);
            item->setText(1, status_text);
            item->setForeground(1, is_running ? Qt::green : Qt::red);
        }
    }
}

void MainWindow::onSystemAlertTriggered(const QString& alert_message)
{
    log_viewer_->addLogMessage(LogViewer::Warning, QString("系统告警: %1").arg(alert_message));

    // 可以选择显示弹窗告警
    // showErrorMessage("系统告警", alert_message);
}

void MainWindow::detectSlamStatus()
{
    // 通过检查ROS2节点来判断SLAM是否在运行
    QProcess* check_process = new QProcess(this);
    check_process->setProgram("ros2");
    check_process->setArguments({"node", "list"});

    connect(check_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, check_process](int exitCode, QProcess::ExitStatus exitStatus) {
            if (exitStatus == QProcess::NormalExit && exitCode == 0) {
                QString output = check_process->readAllStandardOutput();
                bool slamRunning = output.contains("lio_node");

                slam_running_ = slamRunning;
                action_slam_start_->setEnabled(!slamRunning);
                action_slam_stop_->setEnabled(slamRunning);
                action_slam_pause_->setEnabled(slamRunning);

                if (slamRunning) {
                    log_viewer_->addLogMessage(LogViewer::Info, "检测到SLAM系统正在运行");
                    status_slam_label_->setText("SLAM: 运行中");
                    status_slam_label_->setStyleSheet("color: green;");
                } else {
                    log_viewer_->addLogMessage(LogViewer::Info, "SLAM系统未运行，仅显示原始雷达数据");
                    status_slam_label_->setText("SLAM: 未运行");
                    status_slam_label_->setStyleSheet("color: orange;");
                }
            } else {
                log_viewer_->addLogMessage(LogViewer::Warning, "无法检测SLAM系统状态");
            }
            check_process->deleteLater();
        });

    check_process->start();
}

void MainWindow::startSLAMComponent(const QString& component_name)
{
    // 检查组件是否已经在运行
    if (ros2_interface_->isComponentRunning(component_name.toStdString())) {
        log_viewer_->addLogMessage(LogViewer::Warning, QString("%1组件已经在运行").arg(component_name));
        return;
    }

    QProcess* start_process = new QProcess(this);

    // 获取绝对工作空间路径
    QString workspace_path = getWorkspacePath();
    if (workspace_path.isEmpty()) {
        log_viewer_->addLogMessage(LogViewer::Error, "无法确定ROS2工作空间路径");
        start_process->deleteLater();
        return;
    }

    // 设置ROS2环境
    setupROS2Environment(start_process, workspace_path);

    QString config_path;
    if (component_name == "fastlio2") {
        config_path = workspace_path + "/src/fastlio2/config/lio.yaml";
        start_process->setProgram("ros2");
        start_process->setArguments({"run", "fastlio2", "lio_node", "--ros-args", "-p",
                                   QString("config_path:=%1").arg(config_path)});
    }
    else if (component_name == "pgo") {
        config_path = workspace_path + "/src/pgo/config/pgo.yaml";
        start_process->setProgram("ros2");
        start_process->setArguments({"run", "pgo", "pgo_node", "--ros-args", "-p",
                                   QString("config_path:=%1").arg(config_path)});
    }
    else if (component_name == "hba") {
        config_path = workspace_path + "/src/hba/config/hba.yaml";
        start_process->setProgram("ros2");
        start_process->setArguments({"run", "hba", "hba_node", "--ros-args", "-p",
                                   QString("config_path:=%1").arg(config_path)});
    }
    else if (component_name == "localizer") {
        config_path = workspace_path + "/src/localizer/config/localizer.yaml";
        start_process->setProgram("ros2");
        start_process->setArguments({"run", "localizer", "localizer_node", "--ros-args", "-p",
                                   QString("config_path:=%1").arg(config_path)});
    }
    else {
        log_viewer_->addLogMessage(LogViewer::Error, QString("未知组件: %1").arg(component_name));
        start_process->deleteLater();
        return;
    }

    // 验证配置文件存在
    if (!QFile::exists(config_path)) {
        log_viewer_->addLogMessage(LogViewer::Error, QString("配置文件不存在: %1").arg(config_path));
        start_process->deleteLater();
        return;
    }

    connect(start_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, start_process, component_name](int exitCode, QProcess::ExitStatus exitStatus) {
            Q_UNUSED(exitCode);
            Q_UNUSED(exitStatus);
            log_viewer_->addLogMessage(LogViewer::Info, QString("%1组件进程已结束").arg(component_name));
            start_process->deleteLater();
        });

    connect(start_process, &QProcess::errorOccurred,
        [this, start_process, component_name](QProcess::ProcessError error) {
            QString errorMsg = QString("%1组件启动失败: %2").arg(component_name).arg(error);
            log_viewer_->addLogMessage(LogViewer::Error, errorMsg);
            start_process->deleteLater();
        });

    start_process->start();
    log_viewer_->addLogMessage(LogViewer::Info, QString("%1组件启动命令已执行").arg(component_name));

    // 延迟更新组件状态，等待进程启动
    QTimer::singleShot(2000, this, [this]() {
        updateComponentStatus();
    });
}

void MainWindow::stopSLAMComponent(const QString& component_name)
{
    // 检查组件是否正在运行
    if (!ros2_interface_->isComponentRunning(component_name.toStdString())) {
        log_viewer_->addLogMessage(LogViewer::Warning, QString("%1组件未在运行").arg(component_name));
        return;
    }

    QProcess* stop_process = new QProcess(this);

    QString node_name;
    if (component_name == "fastlio2") {
        node_name = "lio_node";
    } else if (component_name == "pgo") {
        node_name = "pgo_node";
    } else if (component_name == "hba") {
        node_name = "hba_node";
    } else if (component_name == "localizer") {
        node_name = "localizer_node";
    } else {
        log_viewer_->addLogMessage(LogViewer::Error, QString("未知组件: %1").arg(component_name));
        stop_process->deleteLater();
        return;
    }

    stop_process->setProgram("pkill");
    stop_process->setArguments({"-f", node_name});

    connect(stop_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, stop_process, component_name](int exitCode, QProcess::ExitStatus exitStatus) {
            Q_UNUSED(exitCode);
            Q_UNUSED(exitStatus);
            log_viewer_->addLogMessage(LogViewer::Info, QString("%1组件已停止").arg(component_name));
            stop_process->deleteLater();
        });

    connect(stop_process, &QProcess::errorOccurred,
        [this, stop_process, component_name](QProcess::ProcessError error) {
            QString errorMsg = QString("%1组件停止失败: %2").arg(component_name).arg(error);
            log_viewer_->addLogMessage(LogViewer::Warning, errorMsg);
            stop_process->deleteLater();
        });

    stop_process->start();
    log_viewer_->addLogMessage(LogViewer::Info, QString("正在停止%1组件...").arg(component_name));

    // 延迟更新组件状态，等待进程停止
    QTimer::singleShot(1000, this, [this]() {
        updateComponentStatus();
    });
}

void MainWindow::updateComponentStatus()
{
    if (!ros2_interface_ || !control_panel_) {
        return;
    }

    // 检查各组件的运行状态，并更新控制面板的状态指示
    bool fastlio2_running = ros2_interface_->isComponentRunning("fastlio2");
    bool pgo_running = ros2_interface_->isComponentRunning("pgo");
    bool hba_running = ros2_interface_->isComponentRunning("hba");
    bool localizer_running = ros2_interface_->isComponentRunning("localizer");

    // 获取当前控制状态
    auto current_state = control_panel_->getControlState();

    // 只有在状态实际改变时才更新，避免无限循环的信号
    bool needs_update = false;
    if (current_state.fastlio2_enabled != fastlio2_running) {
        current_state.fastlio2_enabled = fastlio2_running;
        needs_update = true;
    }
    if (current_state.pgo_enabled != pgo_running) {
        current_state.pgo_enabled = pgo_running;
        needs_update = true;
    }
    if (current_state.hba_enabled != hba_running) {
        current_state.hba_enabled = hba_running;
        needs_update = true;
    }
    if (current_state.localizer_enabled != localizer_running) {
        current_state.localizer_enabled = localizer_running;
        needs_update = true;
    }

    if (needs_update) {
        // 临时断开信号连接，避免循环触发
        disconnect(control_panel_, &ControlPanel::componentToggled, this, &MainWindow::onComponentToggled);

        // 更新控制面板状态
        control_panel_->setControlState(current_state);

        // 重新连接信号
        connect(control_panel_, &ControlPanel::componentToggled, this, &MainWindow::onComponentToggled);

        // 记录状态变化
        QString status_msg = QString("组件状态更新: FastLIO2[%1] PGO[%2] HBA[%3] Localizer[%4]")
            .arg(fastlio2_running ? "运行" : "停止")
            .arg(pgo_running ? "运行" : "停止")
            .arg(hba_running ? "运行" : "停止")
            .arg(localizer_running ? "运行" : "停止");
        log_viewer_->addLogMessage(LogViewer::Info, status_msg);
    }
}

bool MainWindow::validateWorkspaceEnvironment()
{
    QString current_dir = QDir::currentPath();
    
    // 检查是否在ROS2工作空间中
    QDir workspace_dir(current_dir);
    
    // 检查必要的目录结构
    if (!workspace_dir.exists("src")) {
        log_viewer_->addLogMessage(LogViewer::Warning, "未找到src目录，可能不在ROS2工作空间中");
        return false;
    }
    
    if (!workspace_dir.exists("install")) {
        log_viewer_->addLogMessage(LogViewer::Warning, "未找到install目录，请先编译项目");
        return false;
    }
    
    // 检查核心SLAM组件
    QStringList required_components = {"src/fastlio2", "src/pgo", "src/hba", "src/localizer", "src/interface"};
    for (const QString& component : required_components) {
        if (!workspace_dir.exists(component)) {
            log_viewer_->addLogMessage(LogViewer::Error, QString("缺少组件: %1").arg(component));
            return false;
        }
    }
    
    // 检查配置文件
    QStringList config_files = {
        "src/fastlio2/config/lio.yaml",
        "src/pgo/config/pgo.yaml",
        "src/hba/config/hba.yaml",
        "src/localizer/config/localizer.yaml"
    };
    
    for (const QString& config_file : config_files) {
        if (!workspace_dir.exists(config_file)) {
            log_viewer_->addLogMessage(LogViewer::Warning, QString("配置文件缺失: %1").arg(config_file));
        }
    }
    
    log_viewer_->addLogMessage(LogViewer::Info, QString("工作环境验证通过: %1").arg(current_dir));
    return true;
}

QString MainWindow::getWorkspacePath()
{
    // 首先尝试从环境变量获取
    QString ros_workspace = qgetenv("COLCON_PREFIX_PATH");
    if (!ros_workspace.isEmpty()) {
        // COLCON_PREFIX_PATH格式: /path/to/workspace/install
        QDir workspace_dir(ros_workspace);
        if (workspace_dir.cdUp()) { // 回到workspace根目录
            return workspace_dir.absolutePath();
        }
    }

    // 从当前目录向上查找工作空间
    QDir current_dir = QDir::current();
    do {
        if (current_dir.exists("src") && current_dir.exists("install") &&
            current_dir.exists("src/fastlio2")) {
            return current_dir.absolutePath();
        }
    } while (current_dir.cdUp());

    // 使用当前目录作为最后的后备选项
    return QDir::currentPath();
}

void MainWindow::setupROS2Environment(QProcess* process, const QString& workspace_path)
{
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

    // 设置ROS2基础环境
    QString ros_distro = env.value("ROS_DISTRO", "humble");
    env.insert("ROS_DISTRO", ros_distro);
    env.insert("ROS_VERSION", "2");

    // 设置工作空间环境
    QString install_path = workspace_path + "/install";
    QString setup_script = install_path + "/setup.bash";

    if (QFile::exists(setup_script)) {
        // 添加工作空间的setup到环境中
        QString current_colcon_path = env.value("COLCON_PREFIX_PATH");
        if (!current_colcon_path.contains(install_path)) {
            if (!current_colcon_path.isEmpty()) {
                env.insert("COLCON_PREFIX_PATH", install_path + ":" + current_colcon_path);
            } else {
                env.insert("COLCON_PREFIX_PATH", install_path);
            }
        }

        // 设置AMENT_PREFIX_PATH
        QString current_ament_path = env.value("AMENT_PREFIX_PATH");
        if (!current_ament_path.contains(install_path)) {
            if (!current_ament_path.isEmpty()) {
                env.insert("AMENT_PREFIX_PATH", install_path + ":" + current_ament_path);
            } else {
                env.insert("AMENT_PREFIX_PATH", install_path);
            }
        }

        // 设置Python路径
        QString python_path = install_path + "/lib/python3/dist-packages";
        QString current_python_path = env.value("PYTHONPATH");
        if (!current_python_path.contains(python_path)) {
            if (!current_python_path.isEmpty()) {
                env.insert("PYTHONPATH", python_path + ":" + current_python_path);
            } else {
                env.insert("PYTHONPATH", python_path);
            }
        }
    }

    // 设置工作目录
    process->setWorkingDirectory(workspace_path);
    process->setProcessEnvironment(env);

    log_viewer_->addLogMessage(LogViewer::Debug,
        QString("设置ROS2环境 - 工作空间: %1").arg(workspace_path));
}

void MainWindow::setupProcessTimeouts()
{
    // 设置进程超时处理，避免GUI卡死
    process_timeout_timer_ = new QTimer(this);
    process_timeout_timer_->setSingleShot(true);
    process_timeout_timer_->setInterval(30000); // 30秒超时

    connect(process_timeout_timer_, &QTimer::timeout, this, &MainWindow::onProcessTimeout);

    // 进程监控定时器
    process_monitor_timer_ = new QTimer(this);
    process_monitor_timer_->setInterval(5000); // 每5秒检查一次
    connect(process_monitor_timer_, &QTimer::timeout, this, &MainWindow::monitorActiveProcesses);
    process_monitor_timer_->start();
}

void MainWindow::onProcessTimeout()
{
    log_viewer_->addLogMessage(LogViewer::Warning, "进程启动超时，可能需要手动检查");
    status_progress_bar_->setVisible(false);

    // 恢复UI状态
    action_slam_start_->setEnabled(true);
    action_slam_stop_->setEnabled(false);
}

void MainWindow::monitorActiveProcesses()
{
    // 检查关键进程是否仍在运行
    QStringList critical_processes = {"lio_node", "pgo_node", "hba_node", "localizer_node"};

    for (const QString& process_name : critical_processes) {
        // 使用pgrep检查进程
        QProcess* check_process = new QProcess(this);
        check_process->setProgram("pgrep");
        check_process->setArguments({"-x", process_name});

        connect(check_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this, check_process, process_name](int exitCode, QProcess::ExitStatus exitStatus) {
                Q_UNUSED(exitStatus);
                bool is_running = (exitCode == 0);

                // 更新进程状态记录
                if (active_processes_.contains(process_name) != is_running) {
                    active_processes_[process_name] = is_running;

                    QString status_text = is_running ? "运行中" : "已停止";
                    log_viewer_->addLogMessage(LogViewer::Debug,
                        QString("进程状态变化: %1 -> %2").arg(process_name, status_text));

                    // 如果关键进程意外停止，发出警告
                    if (!is_running && slam_running_) {
                        log_viewer_->addLogMessage(LogViewer::Warning,
                            QString("关键进程 %1 意外停止").arg(process_name));
                    }
                }

                check_process->deleteLater();
            });

        check_process->start();
    }
}

void MainWindow::synchronizeParameterToNodes(const QString& parameter_name, const QVariant& value)
{
    if (!ros2_interface_) {
        log_viewer_->addLogMessage(LogViewer::Warning, "ROS2接口未初始化，无法同步参数");
        return;
    }

    // 根据参数类型确定需要更新的节点
    QMap<QString, QVariant> fastlio2_params;
    QMap<QString, QVariant> pgo_params;
    QMap<QString, QVariant> hba_params;
    QMap<QString, QVariant> localizer_params;

    // 参数映射：GUI参数名 -> ROS2节点参数名
    if (parameter_name == "scan_resolution") {
        fastlio2_params["scan_resolution"] = value;
    } else if (parameter_name == "map_resolution") {
        fastlio2_params["map_resolution"] = value;
    } else if (parameter_name == "max_range") {
        fastlio2_params["max_lidar_range"] = value;
    } else if (parameter_name == "voxel_size") {
        fastlio2_params["voxel_size"] = value;
        hba_params["voxel_size"] = value;  // HBA也需要体素大小参数
    } else if (parameter_name == "loop_closure_enabled") {
        pgo_params["enable_loop_closure"] = value;
    } else {
        log_viewer_->addLogMessage(LogViewer::Warning,
            QString("未知参数: %1，跳过同步").arg(parameter_name));
        return;
    }

    // 同步参数到各个节点
    bool sync_success = true;

    if (!fastlio2_params.isEmpty()) {
        if (!ros2_interface_->updateFastLIO2Parameters(fastlio2_params)) {
            log_viewer_->addLogMessage(LogViewer::Warning, "FastLIO2参数同步失败");
            sync_success = false;
        }
    }

    if (!pgo_params.isEmpty()) {
        if (!ros2_interface_->updatePGOParameters(pgo_params)) {
            log_viewer_->addLogMessage(LogViewer::Warning, "PGO参数同步失败");
            sync_success = false;
        }
    }

    if (!hba_params.isEmpty()) {
        if (!ros2_interface_->updateHBAParameters(hba_params)) {
            log_viewer_->addLogMessage(LogViewer::Warning, "HBA参数同步失败");
            sync_success = false;
        }
    }

    if (!localizer_params.isEmpty()) {
        if (!ros2_interface_->updateLocalizerParameters(localizer_params)) {
            log_viewer_->addLogMessage(LogViewer::Warning, "Localizer参数同步失败");
            sync_success = false;
        }
    }

    if (sync_success) {
        log_viewer_->addLogMessage(LogViewer::Info,
            QString("参数 %1 已成功同步到ROS2节点").arg(parameter_name));
    }
}

// 服务接口槽函数实现
void MainWindow::onMapRefinementRequested(const QString& input_path, const QString& output_path, double refinement_level)
{
    if (!ros2_interface_) {
        log_viewer_->addLogMessage(LogViewer::Error, "ROS2接口未初始化，无法执行地图精细化");
        return;
    }

    log_viewer_->addLogMessage(LogViewer::Info,
        QString("开始地图精细化处理 - 输入: %1, 输出: %2, 精度级别: %3")
        .arg(input_path, output_path).arg(refinement_level));

    status_progress_bar_->setVisible(true);
    status_progress_bar_->setRange(0, 0); // 不确定进度

    // 异步调用地图精细化服务
    QTimer::singleShot(100, this, [this, input_path, output_path, refinement_level]() {
        bool success = ros2_interface_->refineMap(
            input_path.toStdString(),
            output_path.toStdString(),
            refinement_level
        );

        status_progress_bar_->setVisible(false);

        if (success) {
            log_viewer_->addLogMessage(LogViewer::Info, "地图精细化请求已提交");
        } else {
            log_viewer_->addLogMessage(LogViewer::Error, "地图精细化请求失败");
        }
    });
}

void MainWindow::onRelocalizationRequested(const QString& map_path, double x, double y, double z, double yaw)
{
    if (!ros2_interface_) {
        log_viewer_->addLogMessage(LogViewer::Error, "ROS2接口未初始化，无法执行重定位");
        return;
    }

    log_viewer_->addLogMessage(LogViewer::Info,
        QString("开始重定位 - 地图: %1, 位置: [%2, %3, %4], 偏航: %5")
        .arg(map_path).arg(x).arg(y).arg(z).arg(yaw));

    status_progress_bar_->setVisible(true);
    status_progress_bar_->setRange(0, 0); // 不确定进度

    // 异步调用重定位服务
    QTimer::singleShot(100, this, [this, map_path, x, y, z, yaw]() {
        bool success = ros2_interface_->relocalize(
            map_path.toStdString(),
            x, y, z, yaw
        );

        status_progress_bar_->setVisible(false);

        if (success) {
            log_viewer_->addLogMessage(LogViewer::Info, "重定位请求已提交");
        } else {
            log_viewer_->addLogMessage(LogViewer::Error, "重定位请求失败");
        }
    });
}

void MainWindow::onLocalizationValidationRequested()
{
    if (!ros2_interface_) {
        log_viewer_->addLogMessage(LogViewer::Warning, "ROS2接口未初始化，无法验证定位状态");
        return;
    }

    bool is_valid = ros2_interface_->isLocalizationValid();

    QString status_text = is_valid ? "有效" : "无效";
    QString message = QString("定位状态验证结果: %1").arg(status_text);

    LogViewer::Level level = is_valid ? LogViewer::Info : LogViewer::Warning;
    log_viewer_->addLogMessage(level, message);

    // 更新状态栏显示
    if (is_valid) {
        status_bar_->showMessage("定位状态: 有效", 3000);
    } else {
        status_bar_->showMessage("定位状态: 无效", 3000);
    }
}

// MOC文件由CMake自动处理