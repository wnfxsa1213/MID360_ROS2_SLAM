#ifndef SLAM_GUI_CONTROL_PANEL_H
#define SLAM_GUI_CONTROL_PANEL_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QProgressBar>
#include <QTextEdit>
#include <QTabWidget>
#include <QSlider>
#include <QFrame>
#include <QTimer>
#include <QDateTime>
#include <memory>

// 前向声明
struct SystemStatus;

struct SystemControlState
{
    bool fastlio2_enabled = false;
    bool pgo_enabled = false; 
    bool hba_enabled = false;
    bool localizer_enabled = false;
    
    bool slam_running = false;
    bool recording_data = false;
    bool auto_save_enabled = false;
    
    QString current_session_name = "default_session";
    QString save_directory = "./saved_maps";
    
    int recording_duration = 0; // seconds
    size_t recorded_points = 0;
    double trajectory_length = 0.0;
};

class ControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget* parent = nullptr);
    ~ControlPanel();

    SystemControlState getControlState() const;
    void setControlState(const SystemControlState& state);
    
    // void updateSystemStatus(const SystemStatus& status);  // 暂时注释
    void updateSystemStatus();  // 简化版本

signals:
    void slamStartRequested();
    void slamStopRequested();
    void slamPauseRequested();
    void slamResetRequested();
    
    void componentToggled(const QString& component_name, bool enabled);
    void parameterChanged(const QString& parameter_name, const QVariant& value);
    
    void mapSaveRequested(const QString& file_path, bool save_patches, bool enable_hba_refine);
    void mapLoadRequested(const QString& file_path);
    void trajectoryExportRequested(const QString& file_path);
    
    void recordingToggled(bool enabled);
    void autoSaveToggled(bool enabled);
    
    void sessionChanged(const QString& session_name);

private slots:
    void onSlamStartClicked();
    void onSlamStopClicked();
    void onSlamPauseClicked();
    void onSlamResetClicked();
    
    void onFastLIO2Toggled(bool enabled);
    void onPGOToggled(bool enabled);
    void onHBAToggled(bool enabled);
    void onLocalizerToggled(bool enabled);
    
    void onSaveMapClicked();
    void onLoadMapClicked();
    void onExportTrajectoryClicked();
    
    void onRecordingToggled(bool enabled);
    void onAutoSaveToggled(bool enabled);
    
    void onSessionNameChanged();
    void onSaveDirectoryChanged();
    
    void updateRecordingInfo();

private:
    void setupUI();
    void setupSlamControlGroup();
    void setupComponentControlGroup();
    void setupDataManagementGroup();
    void setupParameterControlGroup();
    void setupStatusDisplayGroup();
    
    void connectSignals();
    void updateButtonStates();
    void updateStatusIndicators();

private:
    // 布局
    QVBoxLayout* main_layout_;
    QTabWidget* control_tabs_;
    
    // SLAM控制组
    QGroupBox* slam_control_group_;
    QPushButton* start_slam_btn_;
    QPushButton* stop_slam_btn_;
    QPushButton* pause_slam_btn_;
    QPushButton* reset_slam_btn_;
    QLabel* slam_status_label_;
    QProgressBar* slam_progress_;
    
    // 组件控制组
    QGroupBox* component_control_group_;
    QCheckBox* fastlio2_cb_;
    QCheckBox* pgo_cb_;
    QCheckBox* hba_cb_;
    QCheckBox* localizer_cb_;
    
    // 组件状态指示器
    QLabel* fastlio2_status_;
    QLabel* pgo_status_;
    QLabel* hba_status_;
    QLabel* localizer_status_;
    
    // 数据管理组
    QGroupBox* data_management_group_;
    QPushButton* save_map_btn_;
    QPushButton* load_map_btn_;
    QPushButton* export_trajectory_btn_;
    QCheckBox* recording_cb_;
    QCheckBox* auto_save_cb_;
    
    // 会话管理
    QLineEdit* session_name_edit_;
    QLineEdit* save_directory_edit_;
    QPushButton* browse_directory_btn_;
    
    // 录制信息显示
    QLabel* recording_time_label_;
    QLabel* recorded_points_label_;
    QLabel* trajectory_length_label_;
    
    // 参数快速调整
    QGroupBox* parameter_group_;
    QDoubleSpinBox* scan_resolution_spin_;
    QDoubleSpinBox* map_resolution_spin_;
    QSpinBox* max_range_spin_;
    QCheckBox* loop_closure_cb_;
    QDoubleSpinBox* voxel_size_spin_;
    
    // 系统状态显示
    QGroupBox* status_group_;
    QLabel* cpu_usage_label_;
    QLabel* memory_usage_label_;
    QLabel* frequency_label_;
    QLabel* points_count_label_;
    QProgressBar* cpu_progress_;
    QProgressBar* memory_progress_;
    
    // 快速操作按钮
    QPushButton* emergency_stop_btn_;
    QPushButton* quick_save_btn_;
    QPushButton* reset_view_btn_;
    
    // 状态和数据
    SystemControlState control_state_;
    // SystemStatus last_system_status_;  // 暂时注释，用简单类型代替
    struct {
        bool fastlio2_active = false;
        bool pgo_active = false;
        bool hba_active = false;
        bool localizer_active = false;
        double fastlio2_frequency = 0.0;
    } last_system_status_;
    
    QTimer* update_timer_;
    QTimer* recording_timer_;
    
    // 样式定义
    QString getStatusStyleSheet(bool is_active, bool is_healthy = true) const;
    QString getButtonStyleSheet(bool is_primary = false) const;
};

#endif // SLAM_GUI_CONTROL_PANEL_H