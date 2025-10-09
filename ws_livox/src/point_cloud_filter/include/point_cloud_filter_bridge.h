#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <interface/msg/filter_stats.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <memory>
#include <string>
#include <chrono>
#include <vector>

// 引入localizer的头文件
#include "localizers/dynamic_object_filter.h"
#include "localizers/commons.h"

namespace point_cloud_filter {

/**
 * @brief 点云过滤桥接节点类
 * 
 * 该类实现从livox原始数据到过滤后数据的桥接：
 * 1. 订阅 /livox/lidar (CustomMsg格式)
 * 2. 转换为PCL点云格式
 * 3. 应用DynamicObjectFilter过滤动态对象
 * 4. 转换回CustomMsg格式
 * 5. 发布到 /livox/lidar_filtered
 */
class PointCloudFilterBridge : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    explicit PointCloudFilterBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~PointCloudFilterBridge() = default;

private:
    // ================ ROS2 接口 ================
    
    /// 原始livox数据订阅者
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr raw_lidar_sub_;
    
    /// 过滤后数据发布者
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr filtered_lidar_pub_;
    
    /// 过滤统计信息发布者
    rclcpp::Publisher<interface::msg::FilterStats>::SharedPtr filter_stats_pub_;
    
    /// 调试用点云发布者（发布PCL格式用于可视化）
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;

    // ================ 参数和配置 ================
    
    /// 输入topic名称
    std::string input_topic_;
    
    /// 输出topic名称
    std::string output_topic_;
    
    /// 调试点云topic名称
    std::string debug_topic_;
    
    /// 统计信息topic名称
    std::string stats_topic_;
    
    /// 是否启用调试模式
    bool debug_enabled_;
    
    /// 是否发布统计信息
    bool publish_stats_;
    
    /// 处理频率限制（Hz）
    double max_processing_hz_;

    // ================ 过滤器组件 ================
    
    /// 动态对象过滤器
    std::unique_ptr<localizers::DynamicObjectFilter> dynamic_filter_;
    
    /// 过滤器配置
    localizers::DynamicFilterConfig filter_config_;

    // ================ 性能统计 ================
    
    /// 上次处理时间
    std::chrono::steady_clock::time_point last_process_time_;
    
    /// 处理时间间隔限制
    std::chrono::steady_clock::duration min_process_interval_;
    
    /// 总处理帧数
    size_t total_frames_processed_;
    
    /// 启动时间
    std::chrono::high_resolution_clock::time_point start_time_;

    /// 最近一次输入点云（用于元数据回填）
    CloudType::Ptr last_input_cloud_;

    /// 最近一次输入点云的原始 Livox 点信息
    std::vector<livox_ros_driver2::msg::CustomPoint> last_input_metadata_;

    /// 输入点云 KD-Tree，用于最近邻映射
    pcl::KdTreeFLANN<PointType> input_kdtree_;

    /// KD-Tree 是否准备就绪
    bool input_kdtree_ready_{false};

    // ================ 核心回调函数 ================

    /**
     * @brief 处理原始livox数据的回调函数
     * @param msg 原始CustomMsg消息
     */
    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    // ================ 消息转换函数 ================

    /**
     * @brief 将CustomMsg转换为PCL点云
     * @param custom_msg livox CustomMsg
     * @return PCL点云指针
     */
    CloudType::Ptr customMsgToPCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr& custom_msg);

    /**
     * @brief 将PCL点云转换为CustomMsg
     * @param cloud PCL点云
     * @param original_msg 原始消息（用于保持header等信息）
     * @return CustomMsg消息指针
     */
    livox_ros_driver2::msg::CustomMsg::SharedPtr pclToCustomMsg(
        const CloudType::Ptr& cloud,
        const livox_ros_driver2::msg::CustomMsg::SharedPtr& original_msg);

    // ================ 配置和初始化 ================

    /**
     * @brief 初始化ROS2参数
     */
    void initializeParameters();

    /**
     * @brief 初始化ROS2发布者和订阅者
     */
    void initializeRosInterface();

    /**
     * @brief 初始化动态过滤器
     * @return 是否初始化成功
     */
    bool initializeDynamicFilter();

    /**
     * @brief 从ROS参数加载过滤器配置
     * @return 加载的过滤器配置
     */
    localizers::DynamicFilterConfig loadFilterConfig();

    // ================ 工具函数 ================

    /**
     * @brief 检查处理频率限制
     * @return 是否可以处理当前消息
     */
    bool checkProcessingRate();

    /**
     * @brief 发布过滤统计信息
     * @param stats 过滤器统计信息
     * @param header 消息头
     */
    void publishFilterStats(const localizers::FilterStats& stats,
                           const std_msgs::msg::Header& header);

    /**
     * @brief 发布调试点云（用于可视化）
     * @param cloud PCL点云
     * @param header 消息头
     */
    void publishDebugCloud(const CloudType::Ptr& cloud,
                          const std_msgs::msg::Header& header);

    /**
     * @brief 验证CustomMsg消息有效性
     * @param msg CustomMsg消息
     * @return 是否有效
     */
    bool validateCustomMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

    /**
     * @brief 记录性能统计信息
     * @param processing_time_ms 处理时间（毫秒）
     * @param input_points 输入点数
     * @param output_points 输出点数
     */
    void logPerformanceStats(double processing_time_ms, 
                            size_t input_points, 
                            size_t output_points);
};

} // namespace point_cloud_filter
