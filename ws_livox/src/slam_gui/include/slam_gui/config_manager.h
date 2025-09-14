#ifndef SLAM_GUI_CONFIG_MANAGER_H
#define SLAM_GUI_CONFIG_MANAGER_H

#include <QObject>
#include <QString>
#include <QVariant>
#include <memory>
#include <map>
#include <mutex>
#include <functional>
#include <yaml-cpp/yaml.h>

struct SLAMConfig
{
    // 全局参数
    double scan_resolution = 0.15;
    double map_resolution = 0.2;
    double rough_resolution = 0.15;
    
    // 网络配置
    QString host_ip = "192.168.1.50";
    QString lidar_ip = "192.168.1.3";
    
    // FastLIO2参数
    struct {
        double max_range = 80.0;
        double min_range = 0.5;
        int filter_num = 2;
        double cube_len = 200.0;
        double det_range = 100.0;
        double move_thresh = 10.0;
        
        // IMU参数
        double noise_acc = 0.01;
        double noise_gyro = 0.01;
        double bias_acc = 0.001;
        double bias_gyro = 0.0005;
        int init_samples = 150;
        
        // 算法参数
        int near_search_num = 10;
        int ieskf_max_iter = 10;
        bool gravity_align = true;
        bool estimate_extrinsic = false;
        double lidar_cov_inv = 5000.0;
        
        // 外参标定
        std::vector<double> rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        std::vector<double> translation = {-0.011, -0.02329, 0.04412};
    } fastlio2;
    
    // PGO参数
    struct {
        bool enabled = true;
        double delta_translation = 0.5;
        double delta_rotation = 10.0;
        bool loop_closure_enabled = true;
        double search_radius = 1.0;
        double time_threshold = 60.0;
        double score_threshold = 0.15;
        int submap_half_range = 5;
        double min_detect_duration = 5.0;
        bool enable_pose_feedback = true;
        double feedback_frequency_hz = 2.0;
    } pgo;
    
    // HBA参数
    struct {
        bool enabled = true;
        int window_size = 20;
        int stride = 10;
        int max_iterations = 10;
        int hba_iterations = 5;
        double convergence_threshold = 0.01;
        double plane_threshold = 0.01;
        double voxel_size = 0.5;
        int min_points = 10;
        int max_layer = 3;
    } hba;
    
    // Localizer参数
    struct {
        bool enabled = true;
        int rough_max_iteration = 5;
        double rough_score_threshold = 0.2;
        int refine_max_iteration = 10;
        double refine_score_threshold = 0.1;
        bool relocalization_enabled = true;
        double search_radius = 2.0;
        double confidence_threshold = 0.8;
        double update_hz = 1.0;
    } localizer;
    
    // 可视化配置
    struct {
        bool rviz_enabled = true;
        bool auto_start = true;
        bool show_trajectory = true;
        bool show_point_cloud = true;
        bool show_keyframes = true;
        bool show_loop_closures = true;
        int point_size = 2;
        double trajectory_width = 0.1;
    } visualization;
    
    // 监控配置
    struct {
        bool enabled = true;
        bool publish_diagnostics = true;
        bool log_performance = true;
        int max_processing_time = 200;
        int min_point_cloud_size = 1000;
        int max_memory_usage = 28672;
        int max_cpu_usage = 80;
    } monitoring;
    
    // 高级选项
    struct {
        bool enable_parallel = true;
        int num_threads = 12;
        bool enable_optimization = true;
        int max_cache_size = 16384;
        bool enable_profiling = false;
        QString log_level = "INFO";
        bool save_intermediate_results = false;
    } advanced;
};

class ConfigManager : public QObject
{
    Q_OBJECT

public:
    explicit ConfigManager(QObject* parent = nullptr);
    ~ConfigManager();

    bool loadConfig(const QString& config_file_path);
    bool saveConfig(const QString& config_file_path = "");
    bool reloadConfig();
    
    SLAMConfig getCurrentConfig() const;
    void setConfig(const SLAMConfig& config);
    
    // 参数访问接口
    QVariant getParameter(const QString& key) const;
    bool setParameter(const QString& key, const QVariant& value);
    
    // 参数组访问
    QVariant getParameterGroup(const QString& group_name) const;
    bool setParameterGroup(const QString& group_name, const QVariant& group_data);
    
    // 配置验证
    bool validateConfig(const SLAMConfig& config, QString& error_message) const;
    
    // 默认配置
    SLAMConfig getDefaultConfig() const;
    void resetToDefault();
    
    // 配置文件操作
    QString getCurrentConfigFile() const { return current_config_file_; }
    bool hasUnsavedChanges() const { return has_unsaved_changes_; }

signals:
    void configChanged(const SLAMConfig& new_config);
    void parameterChanged(const QString& key, const QVariant& value);
    void configLoadError(const QString& error_message);
    void configSaveError(const QString& error_message);

private:
    void initializeDefaultConfig();
    void markAsChanged();
    
    // YAML转换辅助函数
    SLAMConfig yamlToConfig(const YAML::Node& yaml_node) const;
    YAML::Node configToYaml(const SLAMConfig& config) const;
    
    // 参数路径解析
    QVariant getNestedParameter(const QString& path) const;
    bool setNestedParameter(const QString& path, const QVariant& value);

private:
    mutable std::mutex config_mutex_;
    SLAMConfig current_config_;
    QString current_config_file_;
    bool has_unsaved_changes_;
    
    // 参数映射表 (用于路径式访问)
    std::map<QString, std::function<QVariant()>> parameter_getters_;
    std::map<QString, std::function<bool(const QVariant&)>> parameter_setters_;
};

#endif // SLAM_GUI_CONFIG_MANAGER_H