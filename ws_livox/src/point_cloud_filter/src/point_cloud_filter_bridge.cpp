#include "point_cloud_filter_bridge.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

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
    // 声明和获取参数
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    this->declare_parameter<std::string>("debug_topic", "/point_cloud_filter/debug_cloud");
    this->declare_parameter<std::string>("stats_topic", "/point_cloud_filter/filter_stats");
    
    this->declare_parameter<bool>("debug_enabled", false);
    this->declare_parameter<bool>("publish_stats", true);
    this->declare_parameter<double>("max_processing_hz", 10.0);

    // 获取参数值
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    debug_topic_ = this->get_parameter("debug_topic").as_string();
    stats_topic_ = this->get_parameter("stats_topic").as_string();
    
    debug_enabled_ = this->get_parameter("debug_enabled").as_bool();
    publish_stats_ = this->get_parameter("publish_stats").as_bool();
    max_processing_hz_ = this->get_parameter("max_processing_hz").as_double();

    // 计算最小处理间隔
    if (max_processing_hz_ > 0) {
        min_process_interval_ = std::chrono::milliseconds(
            static_cast<int>(1000.0 / max_processing_hz_));
    } else {
        min_process_interval_ = std::chrono::milliseconds(0);
    }

    last_process_time_ = std::chrono::high_resolution_clock::now();
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
    localizers::DynamicFilterConfig config;

    // 声明并获取动态过滤器参数
    this->declare_parameter<float>("dynamic_filter.motion_threshold", 0.05f);
    this->declare_parameter<int>("dynamic_filter.history_size", 3);
    this->declare_parameter<float>("dynamic_filter.stability_threshold", 0.8f);
    this->declare_parameter<double>("dynamic_filter.max_time_diff", 1.0);
    this->declare_parameter<float>("dynamic_filter.search_radius", 0.2f);
    this->declare_parameter<int>("dynamic_filter.min_neighbors", 8);
    this->declare_parameter<float>("dynamic_filter.normal_consistency_thresh", 0.8f);
    this->declare_parameter<float>("dynamic_filter.density_ratio_thresh", 0.5f);
    this->declare_parameter<int>("dynamic_filter.downsample_ratio", 2);
    this->declare_parameter<int>("dynamic_filter.max_points_per_frame", 50000);
    this->declare_parameter<float>("dynamic_filter.voxel_size_base", 0.01f);

    config.motion_threshold = this->get_parameter("dynamic_filter.motion_threshold").as_double();
    config.history_size = this->get_parameter("dynamic_filter.history_size").as_int();
    config.stability_threshold = this->get_parameter("dynamic_filter.stability_threshold").as_double();
    config.max_time_diff = this->get_parameter("dynamic_filter.max_time_diff").as_double();
    config.search_radius = this->get_parameter("dynamic_filter.search_radius").as_double();
    config.min_neighbors = this->get_parameter("dynamic_filter.min_neighbors").as_int();
    config.normal_consistency_thresh = this->get_parameter("dynamic_filter.normal_consistency_thresh").as_double();
    config.density_ratio_thresh = this->get_parameter("dynamic_filter.density_ratio_thresh").as_double();
    config.downsample_ratio = this->get_parameter("dynamic_filter.downsample_ratio").as_int();
    config.max_points_per_frame = this->get_parameter("dynamic_filter.max_points_per_frame").as_int();
    config.voxel_size_base = this->get_parameter("dynamic_filter.voxel_size_base").as_double();

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

        // 步骤2: 应用动态过滤器
        double timestamp = getCurrentTimestamp();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "开始动态过滤处理...");
        CloudType::Ptr filtered_cloud = dynamic_filter_->filterDynamicObjects(input_cloud, timestamp);

        if (!filtered_cloud) {
            RCLCPP_ERROR(this->get_logger(), "动态过滤处理失败，返回空指针");
            return;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "动态过滤完成，输出点数: %zu", filtered_cloud->size());

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
            auto stats = dynamic_filter_->getLastFilterStats();
            publishFilterStats(stats, msg->header);
        }

        // 发布调试点云
        if (debug_enabled_) {
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

    // 直接基于过滤后的点云生成 CustomMsg，避免 O(N^2) 匹配造成卡顿
    const size_t n = cloud ? cloud->size() : 0;
    custom_msg->points.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& p = cloud->points[i];
        livox_ros_driver2::msg::CustomPoint out;
        out.x = p.x;
        out.y = p.y;
        out.z = p.z;
        // intensity 在 customMsgToPCL 中由 reflectivity 赋值，这里复用
        out.reflectivity = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, p.intensity)));
        // 无法可靠恢复 tag/line/offset_time，使用保守默认值
        out.tag = 0;
        out.line = 0;
        out.offset_time = 0;
        custom_msg->points.push_back(out);
    }

    custom_msg->point_num = static_cast<uint32_t>(custom_msg->points.size());
    return custom_msg;
}

bool PointCloudFilterBridge::checkProcessingRate() {
    auto current_time = std::chrono::high_resolution_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_process_time_) < min_process_interval_) {
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

double PointCloudFilterBridge::getCurrentTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

void PointCloudFilterBridge::logPerformanceStats(
    double processing_time_ms, 
    size_t input_points, 
    size_t output_points) {
    
    // 每100帧输出一次统计信息
    if (total_frames_processed_ % 100 == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(
            current_time - start_time_).count();
        
        double avg_fps = total_frames_processed_ / elapsed_seconds;
        double filter_ratio = input_points > 0 ? 
            static_cast<double>(input_points - output_points) / input_points : 0.0;
        
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

} // namespace point_cloud_filter
