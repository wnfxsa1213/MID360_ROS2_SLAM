#include "slam_gui/config_manager.h"
#include <QFile>
#include <QDir>
#include <QStandardPaths>
#include <fstream>
#include <stdexcept>

ConfigManager::ConfigManager(QObject* parent)
    : QObject(parent)
    , has_unsaved_changes_(false)
{
    initializeDefaultConfig();
}

ConfigManager::~ConfigManager()
{
    // 自动保存更改（如果需要）
    if (has_unsaved_changes_ && !current_config_file_.isEmpty()) {
        saveConfig();
    }
}

void ConfigManager::initializeDefaultConfig()
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    current_config_ = getDefaultConfig();
}

bool ConfigManager::loadConfig(const QString& config_file_path)
{
    try {
        if (!QFile::exists(config_file_path)) {
            emit configLoadError("配置文件不存在: " + config_file_path);
            return false;
        }
        
        YAML::Node yaml_config = YAML::LoadFile(config_file_path.toStdString());
        
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config_ = yamlToConfig(yaml_config);
        current_config_file_ = config_file_path;
        has_unsaved_changes_ = false;
        
        emit configChanged(current_config_);
        return true;
        
    } catch (const std::exception& e) {
        QString error_msg = QString("加载配置文件失败: %1").arg(e.what());
        emit configLoadError(error_msg);
        return false;
    }
}

bool ConfigManager::saveConfig(const QString& config_file_path)
{
    QString file_path = config_file_path.isEmpty() ? current_config_file_ : config_file_path;
    
    if (file_path.isEmpty()) {
        emit configSaveError("未指定保存路径");
        return false;
    }
    
    try {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        YAML::Node yaml_config = configToYaml(current_config_);
        
        std::ofstream file(file_path.toStdString());
        if (!file.is_open()) {
            emit configSaveError("无法创建配置文件: " + file_path);
            return false;
        }
        
        file << yaml_config;
        file.close();
        
        current_config_file_ = file_path;
        has_unsaved_changes_ = false;
        
        return true;
        
    } catch (const std::exception& e) {
        QString error_msg = QString("保存配置文件失败: %1").arg(e.what());
        emit configSaveError(error_msg);
        return false;
    }
}

bool ConfigManager::reloadConfig()
{
    if (current_config_file_.isEmpty()) {
        return false;
    }
    
    return loadConfig(current_config_file_);
}

SLAMConfig ConfigManager::getCurrentConfig() const
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    return current_config_;
}

void ConfigManager::setConfig(const SLAMConfig& config)
{
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config_ = config;
    }
    
    markAsChanged();
    emit configChanged(config);
}

QVariant ConfigManager::getParameter(const QString& key) const
{
    return getNestedParameter(key);
}

bool ConfigManager::setParameter(const QString& key, const QVariant& value)
{
    if (setNestedParameter(key, value)) {
        markAsChanged();
        emit parameterChanged(key, value);
        return true;
    }
    return false;
}

SLAMConfig ConfigManager::getDefaultConfig() const
{
    SLAMConfig config;
    // 默认配置已在结构体中定义
    return config;
}

void ConfigManager::resetToDefault()
{
    setConfig(getDefaultConfig());
}

void ConfigManager::markAsChanged()
{
    has_unsaved_changes_ = true;
}

SLAMConfig ConfigManager::yamlToConfig(const YAML::Node& yaml_node) const
{
    SLAMConfig config = getDefaultConfig(); // 从默认配置开始
    
    try {
        // 全局参数
        if (yaml_node["global_params"]) {
            auto global = yaml_node["global_params"];
            if (global["scan_resolution"]) config.scan_resolution = global["scan_resolution"].as<double>();
            if (global["map_resolution"]) config.map_resolution = global["map_resolution"].as<double>();
            if (global["rough_resolution"]) config.rough_resolution = global["rough_resolution"].as<double>();
        }
        
        // 系统配置
        if (yaml_node["system"]) {
            auto system = yaml_node["system"];
            if (system["host_ip"]) config.host_ip = QString::fromStdString(system["host_ip"].as<std::string>());
            if (system["lidar_ip"]) config.lidar_ip = QString::fromStdString(system["lidar_ip"].as<std::string>());
        }
        
        // FastLIO2配置
        if (yaml_node["fastlio2"]) {
            auto fastlio2 = yaml_node["fastlio2"];
            
            if (fastlio2["lidar_params"]) {
                auto lidar = fastlio2["lidar_params"];
                if (lidar["max_range"]) config.fastlio2.max_range = lidar["max_range"].as<double>();
                if (lidar["min_range"]) config.fastlio2.min_range = lidar["min_range"].as<double>();
                if (lidar["filter_num"]) config.fastlio2.filter_num = lidar["filter_num"].as<int>();
            }
            
            if (fastlio2["mapping_params"]) {
                auto mapping = fastlio2["mapping_params"];
                if (mapping["cube_len"]) config.fastlio2.cube_len = mapping["cube_len"].as<double>();
                if (mapping["det_range"]) config.fastlio2.det_range = mapping["det_range"].as<double>();
                if (mapping["move_thresh"]) config.fastlio2.move_thresh = mapping["move_thresh"].as<double>();
            }
            
            if (fastlio2["imu_params"]) {
                auto imu = fastlio2["imu_params"];
                if (imu["noise_acc"]) config.fastlio2.noise_acc = imu["noise_acc"].as<double>();
                if (imu["noise_gyro"]) config.fastlio2.noise_gyro = imu["noise_gyro"].as<double>();
                if (imu["bias_acc"]) config.fastlio2.bias_acc = imu["bias_acc"].as<double>();
                if (imu["bias_gyro"]) config.fastlio2.bias_gyro = imu["bias_gyro"].as<double>();
                if (imu["init_samples"]) config.fastlio2.init_samples = imu["init_samples"].as<int>();
            }
            
            if (fastlio2["algorithm_params"]) {
                auto algo = fastlio2["algorithm_params"];
                if (algo["near_search_num"]) config.fastlio2.near_search_num = algo["near_search_num"].as<int>();
                if (algo["ieskf_max_iter"]) config.fastlio2.ieskf_max_iter = algo["ieskf_max_iter"].as<int>();
                if (algo["gravity_align"]) config.fastlio2.gravity_align = algo["gravity_align"].as<bool>();
                if (algo["estimate_extrinsic"]) config.fastlio2.estimate_extrinsic = algo["estimate_extrinsic"].as<bool>();
                if (algo["lidar_cov_inv"]) config.fastlio2.lidar_cov_inv = algo["lidar_cov_inv"].as<double>();
            }
        }
        
        // PGO配置
        if (yaml_node["pgo"]) {
            auto pgo = yaml_node["pgo"];
            if (pgo["enabled"]) config.pgo.enabled = pgo["enabled"].as<bool>();
            
            if (pgo["keyframe"]) {
                auto keyframe = pgo["keyframe"];
                if (keyframe["delta_translation"]) config.pgo.delta_translation = keyframe["delta_translation"].as<double>();
                if (keyframe["delta_rotation"]) config.pgo.delta_rotation = keyframe["delta_rotation"].as<double>();
            }
            
            if (pgo["loop_closure"]) {
                auto loop = pgo["loop_closure"];
                if (loop["enabled"]) config.pgo.loop_closure_enabled = loop["enabled"].as<bool>();
                if (loop["search_radius"]) config.pgo.search_radius = loop["search_radius"].as<double>();
                if (loop["time_threshold"]) config.pgo.time_threshold = loop["time_threshold"].as<double>();
                if (loop["score_threshold"]) config.pgo.score_threshold = loop["score_threshold"].as<double>();
            }
        }
        
        // 可以继续添加其他配置项的解析...
        
    } catch (const std::exception& e) {
        throw std::runtime_error("YAML解析错误: " + std::string(e.what()));
    }
    
    return config;
}

YAML::Node ConfigManager::configToYaml(const SLAMConfig& config) const
{
    YAML::Node yaml;
    
    // 全局参数
    yaml["global_params"]["scan_resolution"] = config.scan_resolution;
    yaml["global_params"]["map_resolution"] = config.map_resolution;
    yaml["global_params"]["rough_resolution"] = config.rough_resolution;
    
    // 系统配置
    yaml["system"]["host_ip"] = config.host_ip.toStdString();
    yaml["system"]["lidar_ip"] = config.lidar_ip.toStdString();
    
    // FastLIO2配置
    yaml["fastlio2"]["lidar_params"]["max_range"] = config.fastlio2.max_range;
    yaml["fastlio2"]["lidar_params"]["min_range"] = config.fastlio2.min_range;
    yaml["fastlio2"]["lidar_params"]["filter_num"] = config.fastlio2.filter_num;
    
    yaml["fastlio2"]["mapping_params"]["cube_len"] = config.fastlio2.cube_len;
    yaml["fastlio2"]["mapping_params"]["det_range"] = config.fastlio2.det_range;
    yaml["fastlio2"]["mapping_params"]["move_thresh"] = config.fastlio2.move_thresh;
    
    yaml["fastlio2"]["imu_params"]["noise_acc"] = config.fastlio2.noise_acc;
    yaml["fastlio2"]["imu_params"]["noise_gyro"] = config.fastlio2.noise_gyro;
    yaml["fastlio2"]["imu_params"]["bias_acc"] = config.fastlio2.bias_acc;
    yaml["fastlio2"]["imu_params"]["bias_gyro"] = config.fastlio2.bias_gyro;
    yaml["fastlio2"]["imu_params"]["init_samples"] = config.fastlio2.init_samples;
    
    yaml["fastlio2"]["algorithm_params"]["near_search_num"] = config.fastlio2.near_search_num;
    yaml["fastlio2"]["algorithm_params"]["ieskf_max_iter"] = config.fastlio2.ieskf_max_iter;
    yaml["fastlio2"]["algorithm_params"]["gravity_align"] = config.fastlio2.gravity_align;
    yaml["fastlio2"]["algorithm_params"]["estimate_extrinsic"] = config.fastlio2.estimate_extrinsic;
    yaml["fastlio2"]["algorithm_params"]["lidar_cov_inv"] = config.fastlio2.lidar_cov_inv;
    
    // PGO配置
    yaml["pgo"]["enabled"] = config.pgo.enabled;
    yaml["pgo"]["keyframe"]["delta_translation"] = config.pgo.delta_translation;
    yaml["pgo"]["keyframe"]["delta_rotation"] = config.pgo.delta_rotation;
    yaml["pgo"]["loop_closure"]["enabled"] = config.pgo.loop_closure_enabled;
    yaml["pgo"]["loop_closure"]["search_radius"] = config.pgo.search_radius;
    yaml["pgo"]["loop_closure"]["time_threshold"] = config.pgo.time_threshold;
    yaml["pgo"]["loop_closure"]["score_threshold"] = config.pgo.score_threshold;
    
    // 可以继续添加其他配置项...
    
    return yaml;
}

bool ConfigManager::validateConfig(const SLAMConfig& config, QString& error_message) const
{
    // 验证基本范围
    if (config.scan_resolution <= 0.0 || config.scan_resolution > 1.0) {
        error_message = "扫描分辨率必须在0到1之间";
        return false;
    }
    
    if (config.map_resolution <= 0.0 || config.map_resolution > 1.0) {
        error_message = "地图分辨率必须在0到1之间"; 
        return false;
    }
    
    if (config.fastlio2.max_range <= config.fastlio2.min_range) {
        error_message = "最大检测距离必须大于最小检测距离";
        return false;
    }
    
    if (config.fastlio2.cube_len <= 0.0) {
        error_message = "立方体长度必须大于0";
        return false;
    }
    
    // 验证网络地址格式
    QStringList ip_parts = config.host_ip.split('.');
    if (ip_parts.size() != 4) {
        error_message = "主机IP格式不正确";
        return false;
    }
    
    ip_parts = config.lidar_ip.split('.');
    if (ip_parts.size() != 4) {
        error_message = "雷达IP格式不正确";
        return false;
    }
    
    return true;
}

QVariant ConfigManager::getNestedParameter(const QString& path) const
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    QStringList parts = path.split('.', Qt::SkipEmptyParts);
    if (parts.isEmpty()) return QVariant();
    
    // 简化版本：只支持一些常用参数
    if (parts[0] == "scan_resolution") return current_config_.scan_resolution;
    if (parts[0] == "map_resolution") return current_config_.map_resolution;
    if (parts[0] == "host_ip") return current_config_.host_ip;
    if (parts[0] == "lidar_ip") return current_config_.lidar_ip;
    
    if (parts.size() >= 2 && parts[0] == "fastlio2") {
        if (parts[1] == "max_range") return current_config_.fastlio2.max_range;
        if (parts[1] == "min_range") return current_config_.fastlio2.min_range;
        if (parts[1] == "cube_len") return current_config_.fastlio2.cube_len;
    }
    
    if (parts.size() >= 2 && parts[0] == "pgo") {
        if (parts[1] == "enabled") return current_config_.pgo.enabled;
        if (parts[1] == "delta_translation") return current_config_.pgo.delta_translation;
    }
    
    return QVariant();
}

bool ConfigManager::setNestedParameter(const QString& path, const QVariant& value)
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    QStringList parts = path.split('.', Qt::SkipEmptyParts);
    if (parts.isEmpty()) return false;
    
    // 简化版本：只支持一些常用参数的设置
    if (parts[0] == "scan_resolution" && value.canConvert<double>()) {
        current_config_.scan_resolution = value.toDouble();
        return true;
    }
    if (parts[0] == "map_resolution" && value.canConvert<double>()) {
        current_config_.map_resolution = value.toDouble();
        return true;
    }
    if (parts[0] == "host_ip" && value.canConvert<QString>()) {
        current_config_.host_ip = value.toString();
        return true;
    }
    if (parts[0] == "lidar_ip" && value.canConvert<QString>()) {
        current_config_.lidar_ip = value.toString();
        return true;
    }
    
    if (parts.size() >= 2 && parts[0] == "fastlio2") {
        if (parts[1] == "max_range" && value.canConvert<double>()) {
            current_config_.fastlio2.max_range = value.toDouble();
            return true;
        }
        if (parts[1] == "min_range" && value.canConvert<double>()) {
            current_config_.fastlio2.min_range = value.toDouble();
            return true;
        }
    }
    
    if (parts.size() >= 2 && parts[0] == "pgo") {
        if (parts[1] == "enabled" && value.canConvert<bool>()) {
            current_config_.pgo.enabled = value.toBool();
            return true;
        }
    }
    
    return false;
}

QVariant ConfigManager::getParameterGroup(const QString& group_name) const
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    if (group_name == "global_params") {
        QVariantMap map;
        map["scan_resolution"] = current_config_.scan_resolution;
        map["map_resolution"] = current_config_.map_resolution;
        map["rough_resolution"] = current_config_.rough_resolution;
        return map;
    }

    if (group_name == "system") {
        QVariantMap map;
        map["host_ip"] = current_config_.host_ip;
        map["lidar_ip"] = current_config_.lidar_ip;
        return map;
    }

    if (group_name == "fastlio2") {
        QVariantMap map;
        map["max_range"] = current_config_.fastlio2.max_range;
        map["min_range"] = current_config_.fastlio2.min_range;
        map["filter_num"] = current_config_.fastlio2.filter_num;
        map["cube_len"] = current_config_.fastlio2.cube_len;
        map["det_range"] = current_config_.fastlio2.det_range;
        map["move_thresh"] = current_config_.fastlio2.move_thresh;
        return map;
    }

    if (group_name == "pgo") {
        QVariantMap map;
        map["enabled"] = current_config_.pgo.enabled;
        map["delta_translation"] = current_config_.pgo.delta_translation;
        map["delta_rotation"] = current_config_.pgo.delta_rotation;
        map["loop_closure_enabled"] = current_config_.pgo.loop_closure_enabled;
        map["search_radius"] = current_config_.pgo.search_radius;
        return map;
    }

    return QVariant();
}

bool ConfigManager::setParameterGroup(const QString& group_name, const QVariant& group_data)
{
    if (!group_data.canConvert<QVariantMap>()) {
        return false;
    }

    QVariantMap map = group_data.toMap();
    std::lock_guard<std::mutex> lock(config_mutex_);

    if (group_name == "global_params") {
        if (map.contains("scan_resolution")) current_config_.scan_resolution = map["scan_resolution"].toDouble();
        if (map.contains("map_resolution")) current_config_.map_resolution = map["map_resolution"].toDouble();
        if (map.contains("rough_resolution")) current_config_.rough_resolution = map["rough_resolution"].toDouble();
        markAsChanged();
        return true;
    }

    if (group_name == "system") {
        if (map.contains("host_ip")) current_config_.host_ip = map["host_ip"].toString();
        if (map.contains("lidar_ip")) current_config_.lidar_ip = map["lidar_ip"].toString();
        markAsChanged();
        return true;
    }

    if (group_name == "fastlio2") {
        if (map.contains("max_range")) current_config_.fastlio2.max_range = map["max_range"].toDouble();
        if (map.contains("min_range")) current_config_.fastlio2.min_range = map["min_range"].toDouble();
        if (map.contains("filter_num")) current_config_.fastlio2.filter_num = map["filter_num"].toInt();
        if (map.contains("cube_len")) current_config_.fastlio2.cube_len = map["cube_len"].toDouble();
        if (map.contains("det_range")) current_config_.fastlio2.det_range = map["det_range"].toDouble();
        if (map.contains("move_thresh")) current_config_.fastlio2.move_thresh = map["move_thresh"].toDouble();
        markAsChanged();
        return true;
    }

    if (group_name == "pgo") {
        if (map.contains("enabled")) current_config_.pgo.enabled = map["enabled"].toBool();
        if (map.contains("delta_translation")) current_config_.pgo.delta_translation = map["delta_translation"].toDouble();
        if (map.contains("delta_rotation")) current_config_.pgo.delta_rotation = map["delta_rotation"].toDouble();
        if (map.contains("loop_closure_enabled")) current_config_.pgo.loop_closure_enabled = map["loop_closure_enabled"].toBool();
        if (map.contains("search_radius")) current_config_.pgo.search_radius = map["search_radius"].toDouble();
        markAsChanged();
        return true;
    }

    return false;
}

// MOC文件由CMake自动处理