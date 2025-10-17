#include "point_cloud_filter_bridge.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <filesystem>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace point_cloud_filter {

PointCloudFilterBridge::PointCloudFilterBridge(const rclcpp::NodeOptions& options)
    : Node("point_cloud_filter_bridge", options),
      total_frames_processed_(0),
      start_time_(std::chrono::high_resolution_clock::now()) {
    
    RCLCPP_INFO(this->get_logger(), "正在初始化点云过滤桥接节点...");

    // 初始化参数
    initializeParameters();

    // 初始化动态过滤器
    if (!initializeDynamicFilter()) {
        RCLCPP_ERROR(this->get_logger(), "动态过滤器初始化失败！");
        return;
    }

    // 初始化ROS接口
    initializeRosInterface();

    RCLCPP_INFO(this->get_logger(), 
        "点云过滤桥接节点初始化完成！\n"
        "  输入话题: %s\n"
        "  输出话题: %s\n"
        "  调试模式: %s\n"
        "  最大处理频率: %.1f Hz", 
        input_topic_.c_str(), 
        output_topic_.c_str(),
        debug_enabled_ ? "启用" : "禁用",
        max_processing_hz_);
}

void PointCloudFilterBridge::initializeParameters() {
    filter_config_defaults_ = localizers::DynamicFilterConfig{};
    BridgeParamDefaults defaults;

    // 配置文件解析
    this->declare_parameter<std::string>("filter_config_file", "");
    filter_config_file_ = this->get_parameter("filter_config_file").as_string();
    if (!filter_config_file_.empty()) {
        const auto resolved = resolveConfigPath(filter_config_file_);
        if (resolved.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "未找到过滤器配置文件: %s，使用内置默认值",
                        filter_config_file_.c_str());
        } else {
            filter_config_file_ = resolved;
            loadFilterDefaultsFromFile(filter_config_file_, defaults);
        }
    }

    // 声明并获取基础参数（支持外部覆盖）
    this->declare_parameter<std::string>("input_topic", defaults.input_topic);
    this->declare_parameter<std::string>("output_topic", defaults.output_topic);
    this->declare_parameter<std::string>("debug_topic", defaults.debug_topic);
    this->declare_parameter<std::string>("stats_topic", defaults.stats_topic);
    
    this->declare_parameter<bool>("debug_enabled", defaults.debug_enabled);
    this->declare_parameter<bool>("publish_stats", defaults.publish_stats);
    this->declare_parameter<double>("max_processing_hz", defaults.max_processing_hz);
    this->declare_parameter<double>("dynamic_filter.filter_ratio_warn", defaults.filter_ratio_warn);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    debug_topic_ = this->get_parameter("debug_topic").as_string();
    stats_topic_ = this->get_parameter("stats_topic").as_string();

    debug_enabled_ = this->get_parameter("debug_enabled").as_bool();
    publish_stats_ = this->get_parameter("publish_stats").as_bool();
    max_processing_hz_ = this->get_parameter("max_processing_hz").as_double();
    filter_ratio_warn_threshold_ = this->get_parameter("dynamic_filter.filter_ratio_warn").as_double();

    // 计算最小处理间隔
    if (max_processing_hz_ > 0.0) {
        min_process_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / max_processing_hz_));
    } else {
        min_process_interval_ = std::chrono::steady_clock::duration::zero();
    }

    last_process_time_ = std::chrono::steady_clock::now();
}

void PointCloudFilterBridge::initializeRosInterface() {
    // 创建订阅者
    raw_lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, 10, 
        std::bind(&PointCloudFilterBridge::lidarCallback, this, std::placeholders::_1));

    // 创建发布者
    filtered_lidar_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        output_topic_, 10);

    if (publish_stats_) {
        filter_stats_pub_ = this->create_publisher<interface::msg::FilterStats>(
            stats_topic_, 10);
    }

    if (debug_enabled_) {
        debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            debug_topic_, 10);
    }
}

bool PointCloudFilterBridge::initializeDynamicFilter() {
    try {
        if (!dynamic_filter_enabled_) {
            RCLCPP_WARN(this->get_logger(),
                        "动态过滤器已禁用，桥接节点将直接透传点云（配置文件: %s）",
                        filter_config_file_.empty() ? "默认内置" : filter_config_file_.c_str());
            return true;
        }

        // 加载过滤器配置
        filter_config_ = loadFilterConfig();

        // 验证配置
        if (!filter_config_.isValid()) {
            RCLCPP_ERROR(this->get_logger(), "动态过滤器配置无效！");
            return false;
        }

        // 创建过滤器实例
        dynamic_filter_ = std::make_unique<localizers::DynamicObjectFilter>(filter_config_);

        // 同步调试模式：如果节点开启了debug_enabled，则同时打开动态过滤器的详细调试
        try {
            bool df_debug = false;
            if (this->has_parameter("dynamic_filter.debug_enabled")) {
                df_debug = this->get_parameter("dynamic_filter.debug_enabled").as_bool();
            }
            // 若未单独指定 dynamic_filter.debug_enabled，则复用节点的 debug_enabled
            dynamic_filter_->setDebugMode(df_debug || debug_enabled_);
        } catch (...) {
            dynamic_filter_->setDebugMode(debug_enabled_);
        }

        RCLCPP_INFO(this->get_logger(), 
            "动态过滤器初始化完成:\n"
            "  运动阈值: %.3f m\n"
            "  历史帧数: %d\n"
            "  稳定性阈值: %.2f\n"
            "  搜索半径: %.3f m",
            filter_config_.motion_threshold,
            filter_config_.history_size,
            filter_config_.stability_threshold,
            filter_config_.search_radius);
        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(), "动态过滤器调试模式: 启用");
        }

        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "初始化动态过滤器时出现异常: %s", e.what());
        return false;
    }
}

localizers::DynamicFilterConfig PointCloudFilterBridge::loadFilterConfig() {
    localizers::DynamicFilterConfig config = filter_config_defaults_;

    // 声明并获取动态过滤器参数
    this->declare_parameter<bool>("dynamic_filter.enable", dynamic_filter_enabled_);
    this->declare_parameter<float>("dynamic_filter.motion_threshold", config.motion_threshold);
    this->declare_parameter<int>("dynamic_filter.history_size", config.history_size);
    this->declare_parameter<float>("dynamic_filter.stability_threshold", config.stability_threshold);
    this->declare_parameter<double>("dynamic_filter.max_time_diff", config.max_time_diff);
    this->declare_parameter<float>("dynamic_filter.search_radius", config.search_radius);
    this->declare_parameter<int>("dynamic_filter.min_neighbors", config.min_neighbors);
    this->declare_parameter<float>("dynamic_filter.normal_consistency_thresh", config.normal_consistency_thresh);
    this->declare_parameter<float>("dynamic_filter.density_ratio_thresh", config.density_ratio_thresh);
    this->declare_parameter<int>("dynamic_filter.downsample_ratio", config.downsample_ratio);
    this->declare_parameter<int>("dynamic_filter.max_points_per_frame", config.max_points_per_frame);
    this->declare_parameter<float>("dynamic_filter.voxel_size_base", config.voxel_size_base);

    dynamic_filter_enabled_ = this->get_parameter("dynamic_filter.enable").as_bool();
    config.motion_threshold = static_cast<float>(this->get_parameter("dynamic_filter.motion_threshold").as_double());
    config.history_size = this->get_parameter("dynamic_filter.history_size").as_int();
    config.stability_threshold = static_cast<float>(this->get_parameter("dynamic_filter.stability_threshold").as_double());
    config.max_time_diff = this->get_parameter("dynamic_filter.max_time_diff").as_double();
    config.search_radius = static_cast<float>(this->get_parameter("dynamic_filter.search_radius").as_double());
    config.min_neighbors = this->get_parameter("dynamic_filter.min_neighbors").as_int();
    config.normal_consistency_thresh = static_cast<float>(this->get_parameter("dynamic_filter.normal_consistency_thresh").as_double());
    config.density_ratio_thresh = static_cast<float>(this->get_parameter("dynamic_filter.density_ratio_thresh").as_double());
    config.downsample_ratio = this->get_parameter("dynamic_filter.downsample_ratio").as_int();
    config.max_points_per_frame = this->get_parameter("dynamic_filter.max_points_per_frame").as_int();
    config.voxel_size_base = static_cast<float>(this->get_parameter("dynamic_filter.voxel_size_base").as_double());

    return config;
}

void PointCloudFilterBridge::lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "接收到lidar数据，点数: %u", msg->point_num);

    // 检查处理频率限制
    if (!checkProcessingRate()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "处理频率限制，跳过此帧");
        return;
    }

    // 验证消息有效性
    if (!validateCustomMsg(msg)) {
        RCLCPP_WARN(this->get_logger(), "接收到无效的CustomMsg消息");
        return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "消息验证通过，开始处理");

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // 步骤1: 转换CustomMsg为PCL点云
        CloudType::Ptr input_cloud = customMsgToPCL(msg);
        if (!input_cloud || input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "转换后的点云为空");
            return;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PCL转换成功，点数: %zu", input_cloud->size());

        // 构建原始点云元数据索引
        last_input_cloud_ = input_cloud;
        last_input_metadata_.assign(msg->points.begin(), msg->points.end());
        if (last_input_cloud_ && !last_input_cloud_->empty()) {
            input_kdtree_.setInputCloud(last_input_cloud_);
            input_kdtree_ready_ = !last_input_metadata_.empty();
        } else {
            input_kdtree_ready_ = false;
        }

        // 步骤2: 应用动态过滤器（可选）
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        CloudType::Ptr filtered_cloud;
        if (dynamic_filter_enabled_ && dynamic_filter_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "开始动态过滤处理...");
            filtered_cloud = dynamic_filter_->filterDynamicObjects(input_cloud, timestamp);

            if (!filtered_cloud) {
                RCLCPP_ERROR(this->get_logger(), "动态过滤处理失败，返回空指针");
                return;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "动态过滤完成，输出点数: %zu", filtered_cloud->size());
        } else {
            filtered_cloud = input_cloud;
        }

        // 步骤3: 转换回CustomMsg格式
        auto filtered_msg = pclToCustomMsg(filtered_cloud, msg);
        if (!filtered_msg) {
            RCLCPP_ERROR(this->get_logger(), "PCL到CustomMsg转换失败");
            return;
        }

        // 步骤4: 发布过滤后的数据
        filtered_lidar_pub_->publish(*filtered_msg);

        // 计算处理时间
        auto end_time = std::chrono::high_resolution_clock::now();
        double processing_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // 更新统计信息
        total_frames_processed_++;
        logPerformanceStats(processing_time_ms, input_cloud->size(), filtered_cloud->size());

        // 发布统计信息
        if (publish_stats_) {
            localizers::FilterStats stats;
            if (dynamic_filter_enabled_ && dynamic_filter_) {
                stats = dynamic_filter_->getLastFilterStats();
            } else {
                stats.total_points = input_cloud->size();
                stats.static_points = filtered_cloud->size();
                stats.dynamic_points = stats.total_points - stats.static_points;
                stats.processing_time_ms = processing_time_ms;
                stats.history_frames = 1;
                stats.updateFilterRatio();
            }
            publishFilterStats(stats, msg->header);
        }

        // 发布调试点云
        if (debug_enabled_ && dynamic_filter_enabled_) {
            publishDebugCloud(filtered_cloud, msg->header);
        }

    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理点云时出现异常: %s", e.what());
    }
}

CloudType::Ptr PointCloudFilterBridge::customMsgToPCL(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& custom_msg) {
    
    CloudType::Ptr cloud(new CloudType());
    cloud->reserve(custom_msg->point_num);

    // 转换每个点
    for (const auto& point : custom_msg->points) {
        PointType pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = static_cast<float>(point.reflectivity);
        
        cloud->push_back(pcl_point);
    }

    // 设置点云属性
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;  // livox数据可能包含无效点

    return cloud;
}

livox_ros_driver2::msg::CustomMsg::SharedPtr PointCloudFilterBridge::pclToCustomMsg(
    const CloudType::Ptr& cloud,
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& original_msg) {

    auto custom_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();

    // 继承原始消息头与设备信息
    custom_msg->header = original_msg->header;
    custom_msg->timebase = original_msg->timebase;
    custom_msg->lidar_id = original_msg->lidar_id;
    custom_msg->rsvd = original_msg->rsvd;

    // 直接基于过滤后的点云生成 CustomMsg，同时回填原始元数据
    const size_t n = cloud ? cloud->size() : 0;
    custom_msg->points.reserve(n);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    for (size_t i = 0; i < n; ++i) {
        const auto& p = cloud->points[i];
        livox_ros_driver2::msg::CustomPoint out;
        out.x = p.x;
        out.y = p.y;
        out.z = p.z;
        
        uint8_t reflectivity = static_cast<uint8_t>(std::clamp(p.intensity, 0.0f, 255.0f));
        uint8_t tag = 0;
        uint8_t line = 0;
        uint32_t offset_time = 0;

        if (input_kdtree_ready_ && last_input_cloud_ && !last_input_cloud_->empty()) {
            if (input_kdtree_.nearestKSearch(p, 1, nn_indices, nn_dists) > 0) {
                const auto& src_point = last_input_metadata_.at(static_cast<size_t>(nn_indices[0]));
                reflectivity = src_point.reflectivity;
                tag = src_point.tag;
                line = src_point.line;
                offset_time = src_point.offset_time;
            } else if (!last_input_metadata_.empty()) {
                const auto& fallback = last_input_metadata_.at(std::min(i, last_input_metadata_.size() - 1));
                reflectivity = fallback.reflectivity;
                tag = fallback.tag;
                line = fallback.line;
                offset_time = fallback.offset_time;
            }
        } else if (!last_input_metadata_.empty()) {
            const auto& fallback = last_input_metadata_.at(std::min(i, last_input_metadata_.size() - 1));
            reflectivity = fallback.reflectivity;
            tag = fallback.tag;
            line = fallback.line;
            offset_time = fallback.offset_time;
        }

        out.reflectivity = reflectivity;
        out.tag = tag;
        out.line = line;
        out.offset_time = offset_time;
        custom_msg->points.push_back(out);
    }

    custom_msg->point_num = static_cast<uint32_t>(custom_msg->points.size());
    return custom_msg;
}

bool PointCloudFilterBridge::checkProcessingRate() {
    auto current_time = std::chrono::steady_clock::now();
    
    if (current_time - last_process_time_ < min_process_interval_) {
        return false;
    }
    
    last_process_time_ = current_time;
    return true;
}

void PointCloudFilterBridge::publishFilterStats(
    const localizers::FilterStats& stats,
    const std_msgs::msg::Header& header) {

    auto stats_msg = std::make_shared<interface::msg::FilterStats>();

    // 设置时间戳
    stats_msg->stamp = header.stamp;
    stats_msg->total_points = stats.total_points;
    stats_msg->dynamic_points = stats.dynamic_points;
    stats_msg->static_points = stats.static_points;
    stats_msg->filter_ratio = stats.filter_ratio;
    stats_msg->processing_time_ms = stats.processing_time_ms;
    stats_msg->history_frames = stats.history_frames;
    stats_msg->memory_usage_mb = stats.memory_usage_mb;

    filter_stats_pub_->publish(*stats_msg);
}

void PointCloudFilterBridge::publishDebugCloud(
    const CloudType::Ptr& cloud,
    const std_msgs::msg::Header& header) {
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    
    debug_cloud_pub_->publish(cloud_msg);
}

bool PointCloudFilterBridge::validateCustomMsg(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    
    if (!msg) {
        return false;
    }
    
    if (msg->point_num == 0) {
        return false;
    }
    
    if (msg->points.size() != msg->point_num) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "点数不匹配: point_num=%u, points.size()=%zu (继续处理)",
            msg->point_num, msg->points.size());
    }
    
    return true;
}

void PointCloudFilterBridge::logPerformanceStats(
    double processing_time_ms, 
    size_t input_points, 
    size_t output_points) {
    
    double filter_ratio = input_points > 0 ?
        static_cast<double>(input_points - output_points) / static_cast<double>(input_points) : 0.0;

    if (filter_ratio > filter_ratio_warn_threshold_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "过滤比例 %.1f%% 超过告警阈值 %.1f%%，请检查动态滤波参数",
            filter_ratio * 100.0,
            filter_ratio_warn_threshold_ * 100.0);
    }

    // 每100帧输出一次统计信息
    if (total_frames_processed_ % 100 == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(
            current_time - start_time_).count();
        
        double avg_fps = total_frames_processed_ / elapsed_seconds;
        
        RCLCPP_INFO(this->get_logger(),
            "性能统计 [第%zu帧]:\n"
            "  处理时间: %.2f ms\n"
            "  输入点数: %zu\n"
            "  输出点数: %zu\n"
            "  过滤比例: %.1f%%\n"
            "  平均帧率: %.1f FPS",
            total_frames_processed_,
            processing_time_ms,
            input_points,
            output_points,
            filter_ratio * 100.0,
            avg_fps);
    }
}

std::string PointCloudFilterBridge::resolveConfigPath(const std::string& path) const {
    namespace fs = std::filesystem;
    if (path.empty()) {
        return "";
    }

    fs::path candidate(path);
    if (candidate.is_absolute()) {
        return fs::exists(candidate) ? candidate.lexically_normal().string() : "";
    }

    std::vector<fs::path> search_roots;
    try {
        search_roots.emplace_back(ament_index_cpp::get_package_share_directory("point_cloud_filter"));
    } catch (const std::exception&) {
    }
    try {
        search_roots.emplace_back(ament_index_cpp::get_package_share_directory("localizer"));
    } catch (const std::exception&) {
    }
    search_roots.emplace_back(fs::current_path());

    for (const auto& root : search_roots) {
        fs::path resolved = root / candidate;
        if (fs::exists(resolved)) {
            return resolved.lexically_normal().string();
        }
    }
    return "";
}

void PointCloudFilterBridge::loadFilterDefaultsFromFile(const std::string& resolved_path,
                                                        BridgeParamDefaults& defaults) {
    try {
        YAML::Node root = YAML::LoadFile(resolved_path);

        if (root["bridge"]) {
            auto bridge = root["bridge"];
            if (bridge["input_topic"]) {
                defaults.input_topic = bridge["input_topic"].as<std::string>(defaults.input_topic);
            }
            if (bridge["output_topic"]) {
                defaults.output_topic = bridge["output_topic"].as<std::string>(defaults.output_topic);
            }
            if (bridge["debug_topic"]) {
                defaults.debug_topic = bridge["debug_topic"].as<std::string>(defaults.debug_topic);
            }
            if (bridge["stats_topic"]) {
                defaults.stats_topic = bridge["stats_topic"].as<std::string>(defaults.stats_topic);
            }
            if (bridge["debug_enabled"]) {
                defaults.debug_enabled = bridge["debug_enabled"].as<bool>(defaults.debug_enabled);
            }
            if (bridge["publish_stats"]) {
                defaults.publish_stats = bridge["publish_stats"].as<bool>(defaults.publish_stats);
            }
            if (bridge["max_processing_hz"]) {
                defaults.max_processing_hz = bridge["max_processing_hz"].as<double>(defaults.max_processing_hz);
            }
        }

        if (root["dynamic_filter"]) {
            auto df = root["dynamic_filter"];
            if (df["enable"]) {
                dynamic_filter_enabled_ = df["enable"].as<bool>(dynamic_filter_enabled_);
            }
            if (df["motion_threshold"]) {
                filter_config_defaults_.motion_threshold = static_cast<float>(
                    df["motion_threshold"].as<double>(filter_config_defaults_.motion_threshold));
            }
            if (df["history_size"]) {
                filter_config_defaults_.history_size = df["history_size"].as<int>(filter_config_defaults_.history_size);
            }
            if (df["stability_threshold"]) {
                filter_config_defaults_.stability_threshold = static_cast<float>(
                    df["stability_threshold"].as<double>(filter_config_defaults_.stability_threshold));
            }
            if (df["max_time_diff"]) {
                filter_config_defaults_.max_time_diff = df["max_time_diff"].as<double>(filter_config_defaults_.max_time_diff);
            }
            if (df["search_radius"]) {
                filter_config_defaults_.search_radius = static_cast<float>(
                    df["search_radius"].as<double>(filter_config_defaults_.search_radius));
            }
            if (df["min_neighbors"]) {
                filter_config_defaults_.min_neighbors = df["min_neighbors"].as<int>(filter_config_defaults_.min_neighbors);
            }
            if (df["normal_consistency_thresh"]) {
                filter_config_defaults_.normal_consistency_thresh = static_cast<float>(
                    df["normal_consistency_thresh"].as<double>(filter_config_defaults_.normal_consistency_thresh));
            }
            if (df["density_ratio_thresh"]) {
                filter_config_defaults_.density_ratio_thresh = static_cast<float>(
                    df["density_ratio_thresh"].as<double>(filter_config_defaults_.density_ratio_thresh));
            }
            if (df["downsample_ratio"]) {
                filter_config_defaults_.downsample_ratio = df["downsample_ratio"].as<int>(filter_config_defaults_.downsample_ratio);
            }
            if (df["max_points_per_frame"]) {
                filter_config_defaults_.max_points_per_frame = df["max_points_per_frame"].as<int>(filter_config_defaults_.max_points_per_frame);
            }
            if (df["voxel_size_base"]) {
                filter_config_defaults_.voxel_size_base = static_cast<float>(
                    df["voxel_size_base"].as<double>(filter_config_defaults_.voxel_size_base));
            }
            if (df["filter_ratio_warn"]) {
                defaults.filter_ratio_warn = df["filter_ratio_warn"].as<double>(defaults.filter_ratio_warn);
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "已从配置文件加载动态过滤器默认参数: %s",
                    resolved_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "加载过滤器配置文件失败(%s): %s",
                    resolved_path.c_str(),
                    e.what());
    }
}

} // namespace point_cloud_filter
