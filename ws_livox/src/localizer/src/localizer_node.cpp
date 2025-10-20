#include <queue>
#include <mutex>
#include <filesystem>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/dynamic_object_filter.h"
#include "interface/error_codes.hpp"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include "interface/msg/filter_stats.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "fastlio2/body_cloud";
    std::string odom_topic = "fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;

    // 动态过滤器配置
    bool enable_dynamic_filter = true;
    bool publish_filter_stats = true;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("localizer/relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("localizer/relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/map_cloud", 10);

        // 初始化动态过滤器和相关发布器
        if (m_config.enable_dynamic_filter) {
            m_dynamic_filter = std::make_shared<localizers::DynamicObjectFilter>(m_filter_config);

            // 初始化过滤后点云发布器
            m_filtered_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/filtered_cloud", 10);

            // 初始化动态点云发布器（用于调试可视化）
            m_dynamic_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/dynamic_points", 10);

            RCLCPP_INFO(this->get_logger(), "Dynamic Object Filter and publishers initialized");
        }

        // 初始化统计信息发布器
        if (m_config.publish_filter_stats) {
            m_filter_stats_pub = this->create_publisher<interface::msg::FilterStats>("localizer/filter_stats", 10);
            RCLCPP_INFO(this->get_logger(), "Filter statistics publisher initialized");
        }

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        this->declare_parameter<bool>("use_external_filter", true);
        m_use_external_filter = this->get_parameter("use_external_filter").as_bool();

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        // 加载动态过滤器配置
        localizers::DynamicFilterConfig filter_defaults;
        bool enable_default = true;
        bool publish_stats_default = true;
        if (config["dynamic_filter"]) {
            auto filter_config = config["dynamic_filter"];
            if (filter_config["config_file"]) {
                m_filter_config_path = filter_config["config_file"].as<std::string>("");
                auto resolved = resolveConfigPath(m_filter_config_path);
                if (resolved.empty()) {
                    RCLCPP_WARN(this->get_logger(),
                                "未找到动态过滤器配置文件: %s，使用内置默认值",
                                m_filter_config_path.c_str());
                    m_filter_config_path.clear();
                } else {
                    m_filter_config_path = resolved;
                    loadFilterConfigFromFile(m_filter_config_path, filter_defaults, enable_default, publish_stats_default);
                }
            }

            m_filter_config = filter_defaults;
            m_config.enable_dynamic_filter = filter_config["enable"].as<bool>(enable_default);
            m_config.publish_filter_stats = filter_config["publish_stats"].as<bool>(publish_stats_default);

            // 时间一致性参数
            m_filter_config.motion_threshold = filter_config["motion_threshold"].as<float>(m_filter_config.motion_threshold);
            m_filter_config.history_size = filter_config["history_size"].as<int>(m_filter_config.history_size);
            m_filter_config.stability_threshold = filter_config["stability_threshold"].as<float>(m_filter_config.stability_threshold);
            m_filter_config.max_time_diff = filter_config["max_time_diff"].as<double>(m_filter_config.max_time_diff);

            // 几何特征参数
            m_filter_config.search_radius = filter_config["search_radius"].as<float>(m_filter_config.search_radius);
            m_filter_config.min_neighbors = filter_config["min_neighbors"].as<int>(m_filter_config.min_neighbors);
            m_filter_config.normal_consistency_thresh = filter_config["normal_consistency_thresh"].as<float>(m_filter_config.normal_consistency_thresh);
            m_filter_config.density_ratio_thresh = filter_config["density_ratio_thresh"].as<float>(m_filter_config.density_ratio_thresh);

            // 性能参数
            m_filter_config.downsample_ratio = filter_config["downsample_ratio"].as<int>(m_filter_config.downsample_ratio);
            m_filter_config.max_points_per_frame = filter_config["max_points_per_frame"].as<int>(m_filter_config.max_points_per_frame);
            m_filter_config.voxel_size_base = filter_config["voxel_size_base"].as<float>(m_filter_config.voxel_size_base);
        } else {
            // 使用默认配置
            m_config.enable_dynamic_filter = enable_default;
            m_config.publish_filter_stats = publish_stats_default;
            m_filter_config = filter_defaults;
        }

        if (m_config.enable_dynamic_filter) {
            auto issues = localizers::DynamicObjectFilter::sanitizeConfig(m_filter_config);
            for (const auto& issue : issues) {
                RCLCPP_WARN(this->get_logger(), "[FilterConfig] %s", issue.c_str());
            }
            if (!m_filter_config.isValid()) {
                RCLCPP_ERROR(this->get_logger(), "%s",
                             interface::errors::format(interface::errors::Code::INVALID_CONFIGURATION,
                                                       "dynamic filter configuration invalid after sanitization").c_str());
                m_config.enable_dynamic_filter = false;
            }
        }

        if (m_use_external_filter) {
            if (m_config.enable_dynamic_filter) {
                RCLCPP_INFO(this->get_logger(),
                            "检测到 use_external_filter=true，禁用本地动态过滤器，改用外部预过滤点云");
            }
            m_config.enable_dynamic_filter = false;
        }

        RCLCPP_INFO(this->get_logger(), "Dynamic filter enabled: %s",
                   m_config.enable_dynamic_filter ? "YES" : "NO");
        if (m_config.enable_dynamic_filter) {
            RCLCPP_INFO(this->get_logger(), "Filter stats publishing: %s",
                       m_config.publish_filter_stats ? "YES" : "NO");
        }
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex>(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        std::lock_guard<std::mutex>(m_state.message_mutex);

        // 将ROS消息转换为PCL点云
        CloudType::Ptr raw_cloud = std::make_shared<CloudType>();
        pcl::fromROSMsg(*cloud_msg, *raw_cloud);

        // 如果启用动态过滤器，应用过滤处理
        if (m_config.enable_dynamic_filter && m_dynamic_filter) {
            try {
                // 计算时间戳（秒）
                double timestamp = cloud_msg->header.stamp.sec + cloud_msg->header.stamp.nanosec * 1e-9;

                // 应用动态对象过滤
                m_state.last_cloud = m_dynamic_filter->filterDynamicObjects(raw_cloud, timestamp);

                // 发布过滤后的点云
                if (m_filtered_cloud_pub && m_filtered_cloud_pub->get_subscription_count() > 0) {
                    publishFilteredCloud(m_state.last_cloud, cloud_msg->header);
                }

                // 发布动态点云（用于调试可视化）
                if (m_dynamic_cloud_pub && m_dynamic_cloud_pub->get_subscription_count() > 0) {
                    CloudType::Ptr dynamic_cloud = m_dynamic_filter->getLastDynamicPoints();
                    if (dynamic_cloud && !dynamic_cloud->empty()) {
                        publishDynamicCloud(dynamic_cloud, cloud_msg->header);
                    }
                }

                // 发布统计信息
                if (m_config.publish_filter_stats && m_filter_stats_pub) {
                    publishFilterStats(timestamp);
                }

                RCLCPP_DEBUG(this->get_logger(), "Dynamic filtering applied. Original: %zu, Filtered: %zu points",
                           raw_cloud->size(), m_state.last_cloud->size());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Dynamic filtering failed: %s. Using raw cloud.", e.what());
                m_state.last_cloud = raw_cloud;
            }
        } else {
            // 不使用动态过滤器，直接使用原始点云
            m_state.last_cloud = raw_cloud;
        }

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        if (!pcd_path.empty()) {
            bool load_flag = m_localizer->loadMap(pcd_path);
            if (!load_flag)
            {
                response->success = false;
                response->message = "load map failed";
                return;
            }
        }
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        if (!m_localizer->align(m_state.initial_guess)) {
            response->success = false;
            response->message = "ICP align failed";
            return;
        }

        geometry_msgs::msg::Pose pose_msg;
        Eigen::Matrix4f result = m_state.initial_guess;
        Eigen::Quaternionf q(result.block<3,3>(0,0));
        pose_msg.position.x = result(0,3);
        pose_msg.position.y = result(1,3);
        pose_msg.position.z = result(2,3);
        pose_msg.orientation.w = q.w();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();

        response->success = true;
        response->message = "relocalize success";
        response->refined_pose = pose_msg;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex>(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

    /**
     * @brief 发布动态过滤器统计信息
     * @param timestamp 当前时间戳
     */
    void publishFilterStats(double timestamp)
    {
        if (!m_dynamic_filter || !m_filter_stats_pub) {
            return;
        }

        try {
            // 获取过滤器统计信息
            auto stats = m_dynamic_filter->getLastFilterStats();

            // 创建ROS消息
            interface::msg::FilterStats stats_msg;
            stats_msg.total_points = stats.total_points;
            stats_msg.dynamic_points = stats.dynamic_points;
            stats_msg.static_points = stats.static_points;
            stats_msg.filter_ratio = stats.filter_ratio;
            stats_msg.processing_time_ms = stats.processing_time_ms;
            stats_msg.history_frames = stats.history_frames;
            stats_msg.memory_usage_mb = stats.memory_usage_mb;

            // 设置时间戳
            stats_msg.stamp = rclcpp::Clock().now();

            // 发布统计信息
            m_filter_stats_pub->publish(stats_msg);

            RCLCPP_DEBUG(this->get_logger(),
                        "Filter stats: Total=%zu, Dynamic=%zu, Static=%zu, Ratio=%.3f, Time=%.2fms",
                        stats.total_points, stats.dynamic_points, stats.static_points,
                        stats.filter_ratio, stats.processing_time_ms);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to publish filter stats: %s", e.what());
        }
    }

    /**
     * @brief 发布过滤后的点云
     * @param filtered_cloud 过滤后的点云
     * @param header 原始消息头
     */
    void publishFilteredCloud(const CloudType::Ptr& filtered_cloud, const std_msgs::msg::Header& header)
    {
        if (!filtered_cloud || filtered_cloud->empty()) {
            return;
        }

        try {
            sensor_msgs::msg::PointCloud2 filtered_msg;
            pcl::toROSMsg(*filtered_cloud, filtered_msg);
            filtered_msg.header = header;
            filtered_msg.header.frame_id = "lidar";
            m_filtered_cloud_pub->publish(filtered_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published filtered cloud with %zu points", filtered_cloud->size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to publish filtered cloud: %s", e.what());
        }
    }

    /**
     * @brief 发布动态点云（用于调试可视化）
     * @param dynamic_cloud 动态点云
     * @param header 原始消息头
     */
    void publishDynamicCloud(const CloudType::Ptr& dynamic_cloud, const std_msgs::msg::Header& header)
    {
        if (!dynamic_cloud || dynamic_cloud->empty()) {
            return;
        }

        try {
            sensor_msgs::msg::PointCloud2 dynamic_msg;
            pcl::toROSMsg(*dynamic_cloud, dynamic_msg);
            dynamic_msg.header = header;
            dynamic_msg.header.frame_id = "lidar";
            m_dynamic_cloud_pub->publish(dynamic_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published dynamic cloud with %zu points", dynamic_cloud->size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to publish dynamic cloud: %s", e.what());
        }
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;

    // 动态过滤器相关
    localizers::DynamicFilterConfig m_filter_config;
    std::shared_ptr<localizers::DynamicObjectFilter> m_dynamic_filter;
    bool m_use_external_filter{true};
    std::string m_filter_config_path;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<interface::msg::FilterStats>::SharedPtr m_filter_stats_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_dynamic_cloud_pub;

    std::string resolveConfigPath(const std::string& path) const
    {
        namespace fs = std::filesystem;
        if (path.empty())
        {
            return "";
        }

        fs::path candidate(path);
        if (candidate.is_absolute())
        {
            return fs::exists(candidate) ? candidate.lexically_normal().string() : "";
        }

        std::vector<fs::path> search_roots;
        try
        {
            search_roots.emplace_back(ament_index_cpp::get_package_share_directory("localizer"));
        }
        catch (const std::exception &)
        {
        }
        try
        {
            search_roots.emplace_back(ament_index_cpp::get_package_share_directory("point_cloud_filter"));
        }
        catch (const std::exception &)
        {
        }
        search_roots.emplace_back(fs::current_path());

        for (const auto &root : search_roots)
        {
            fs::path resolved = root / candidate;
            if (fs::exists(resolved))
            {
                return resolved.lexically_normal().string();
            }
        }
        return "";
    }

    void loadFilterConfigFromFile(const std::string& path,
                                  localizers::DynamicFilterConfig& defaults,
                                  bool& enable_default,
                                  bool& publish_stats_default)
    {
        try
        {
            YAML::Node root = YAML::LoadFile(path);
            if (!root["dynamic_filter"])
            {
                RCLCPP_WARN(this->get_logger(),
                            "动态过滤器配置文件缺少 dynamic_filter 节点: %s",
                            path.c_str());
                return;
            }

            auto df = root["dynamic_filter"];
            if (df["enable"])
            {
                enable_default = df["enable"].as<bool>(enable_default);
            }
            if (df["publish_stats"])
            {
                publish_stats_default = df["publish_stats"].as<bool>(publish_stats_default);
            }
            if (df["motion_threshold"])
            {
                defaults.motion_threshold = static_cast<float>(df["motion_threshold"].as<double>(defaults.motion_threshold));
            }
            if (df["history_size"])
            {
                defaults.history_size = df["history_size"].as<int>(defaults.history_size);
            }
            if (df["stability_threshold"])
            {
                defaults.stability_threshold = static_cast<float>(df["stability_threshold"].as<double>(defaults.stability_threshold));
            }
            if (df["max_time_diff"])
            {
                defaults.max_time_diff = df["max_time_diff"].as<double>(defaults.max_time_diff);
            }
            if (df["search_radius"])
            {
                defaults.search_radius = static_cast<float>(df["search_radius"].as<double>(defaults.search_radius));
            }
            if (df["min_neighbors"])
            {
                defaults.min_neighbors = df["min_neighbors"].as<int>(defaults.min_neighbors);
            }
            if (df["normal_consistency_thresh"])
            {
                defaults.normal_consistency_thresh = static_cast<float>(df["normal_consistency_thresh"].as<double>(defaults.normal_consistency_thresh));
            }
            if (df["density_ratio_thresh"])
            {
                defaults.density_ratio_thresh = static_cast<float>(df["density_ratio_thresh"].as<double>(defaults.density_ratio_thresh));
            }
            if (df["downsample_ratio"])
            {
                defaults.downsample_ratio = df["downsample_ratio"].as<int>(defaults.downsample_ratio);
            }
            if (df["max_points_per_frame"])
            {
                defaults.max_points_per_frame = df["max_points_per_frame"].as<int>(defaults.max_points_per_frame);
            }
            if (df["voxel_size_base"])
            {
                defaults.voxel_size_base = static_cast<float>(df["voxel_size_base"].as<double>(defaults.voxel_size_base));
            }

            RCLCPP_INFO(this->get_logger(),
                        "已从动态过滤器配置文件加载默认参数: %s",
                        path.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(),
                        "加载动态过滤器配置文件失败(%s): %s",
                        path.c_str(),
                        e.what());
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
