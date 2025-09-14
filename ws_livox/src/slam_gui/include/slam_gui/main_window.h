#ifndef SLAM_GUI_MAIN_WINDOW_H
#define SLAM_GUI_MAIN_WINDOW_H

#include <memory>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSplitter>
#include <QTabWidget>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTreeWidget>
#include <QTableWidget>
#include <QProgressBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>
#include <QProcess>

#include "ros2_interface.h"
#include "system_monitor.h"
#include "config_manager.h"
#include "visualization_widget.h"
#include "control_panel.h"
#include "log_viewer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void onNewSLAMData(const SLAMData& data);
    void onSystemStatusChanged(const SystemStatus& status);
    void onErrorOccurred(const QString& error_message);
    void onServiceCallCompleted(const QString& service_name, bool success, const QString& message);
    
    // 菜单动作
    void onFileNew();
    void onFileOpen();
    void onFileSave();
    void onFileSaveAs();
    void onFileExit();
    
    void onEditPreferences();
    
    void onViewFullscreen();
    void onViewResetLayout();
    void onViewShowControlPanel();
    void onViewShowMonitor();
    void onViewShowLog();
    
    void onSlamStart();
    void onSlamStop();
    void onSlamPause();
    void onSlamReset();
    void onSlamSaveMap();
    void onSlamLoadMap();
    
    void onHelpAbout();
    void onHelpUserGuide();

    // 控制面板槽函数
    void onControlPanelSaveMap(const QString& file_path, bool save_patches, bool enable_hba_refine);
    void onControlPanelLoadMap(const QString& file_path);
    void onControlPanelExportTrajectory(const QString& file_path);
    void onComponentToggled(const QString& component_name, bool enabled);
    void onRecordingToggled(bool enabled);
    void onAutoSaveToggled(bool enabled);

    // 服务接口槽函数
    void onMapRefinementRequested(const QString& input_path, const QString& output_path, double refinement_level);
    void onRelocalizationRequested(const QString& map_path, double x, double y, double z, double yaw);
    void onLocalizationValidationRequested();

    // 系统监控槽函数
    void onSystemMetricsUpdated(const SystemMetrics& metrics);
    void onProcessStatusChanged(const QString& process_name, bool is_running);
    void onSystemAlertTriggered(const QString& alert_message);

    // 定时更新
    void updateUI();
    void updateStatusBar();

private:
    void setupUI();
    void setupMenuBar();
    void setupToolBar();
    void setupStatusBar();
    void setupCentralWidget();
    void setupMonitorTabs();
    void setupDockWidgets();
    
    void connectSignals();
    void loadSettings();
    void saveSettings();
    
    void showErrorMessage(const QString& title, const QString& message);
    void showInfoMessage(const QString& title, const QString& message);
    bool askConfirmation(const QString& title, const QString& message);

    void detectSlamStatus();
    void startSLAMComponent(const QString& component_name);
    void stopSLAMComponent(const QString& component_name);
    void updateComponentStatus();
    bool validateWorkspaceEnvironment();
    void setupProcessTimeouts();

    // 环境配置方法
    QString getWorkspacePath();
    void setupROS2Environment(QProcess* process, const QString& workspace_path);

    // 进程管理方法
    void onProcessTimeout();
    void monitorActiveProcesses();

    // 参数同步方法
    void synchronizeParameterToNodes(const QString& parameter_name, const QVariant& value);

private:
    // 核心组件
    std::unique_ptr<ROS2Interface> ros2_interface_;
    std::unique_ptr<SystemMonitor> system_monitor_;
    std::unique_ptr<ConfigManager> config_manager_;
    
    // UI组件
    QWidget* central_widget_;
    QSplitter* main_splitter_;
    QSplitter* right_splitter_;
    
    // 主要面板
    VisualizationWidget* visualization_widget_;
    ControlPanel* control_panel_;
    LogViewer* log_viewer_;
    
    // 监控面板
    QTabWidget* monitor_tab_widget_;
    QWidget* status_monitor_widget_;
    QWidget* performance_monitor_widget_;
    QWidget* diagnostics_monitor_widget_;
    
    // 状态显示组件
    QTreeWidget* status_tree_;
    QTableWidget* performance_table_;
    QTextEdit* diagnostics_text_;
    
    // 菜单和工具栏
    QMenuBar* menu_bar_;
    QToolBar* main_toolbar_;
    QToolBar* slam_toolbar_;
    
    // 菜单项
    QMenu* file_menu_;
    QMenu* edit_menu_;
    QMenu* view_menu_;
    QMenu* slam_menu_;
    QMenu* help_menu_;
    
    // 动作
    QAction* action_new_;
    QAction* action_open_;
    QAction* action_save_;
    QAction* action_save_as_;
    QAction* action_exit_;
    
    QAction* action_preferences_;
    
    QAction* action_fullscreen_;
    QAction* action_reset_layout_;
    QAction* action_show_control_panel_;
    QAction* action_show_monitor_;
    QAction* action_show_log_;
    
    QAction* action_slam_start_;
    QAction* action_slam_stop_;
    QAction* action_slam_pause_;
    QAction* action_slam_reset_;
    QAction* action_slam_save_map_;
    QAction* action_slam_load_map_;
    
    QAction* action_about_;
    QAction* action_user_guide_;
    
    // 状态栏组件
    QStatusBar* status_bar_;
    QLabel* status_slam_label_;
    QLabel* status_nodes_label_;
    QLabel* status_frequency_label_;
    QProgressBar* status_progress_bar_;
    
    // 定时器
    QTimer* ui_update_timer_;
    QTimer* status_update_timer_;
    QTimer* process_timeout_timer_;
    QTimer* process_monitor_timer_;
    
    // 设置
    QSettings* settings_;
    
    // 状态变量
    bool is_fullscreen_;
    bool slam_running_;
    QString current_map_file_;
    QMap<QString, bool> active_processes_;
    
    // 布局状态
    QByteArray default_geometry_;
    QByteArray default_state_;
};

#endif // SLAM_GUI_MAIN_WINDOW_H