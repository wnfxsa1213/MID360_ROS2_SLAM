#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cctype>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "interface/srv/save_maps.hpp"
#include "interface/srv/save_poses.hpp"
#include "interface/srv/update_pose.hpp"
#include "interface/srv/sync_state.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <yaml-cpp/yaml.h>
// 协同优化服务（触发保存前的全局优化）
#include "interface/srv/trigger_optimization.hpp"

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "base_link";     // 修复：与配置文件保持一致
    std::string world_frame = "odom";        // 修复：与配置文件保持一致
    bool print_time_cost = false;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::mutex path_mutex;  // 添加路径数据保护锁
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        // 优化订阅队列大小：IMU 1000Hz需要更大队列，激光雷达10Hz需要适中队列
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, 100, // IMU队列增加到100，支持高频数据
            std::bind(&LIONode::imuCB, this, std::placeholders::_1));

        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_config.lidar_topic, 20, // 激光雷达队列增加到20，提供缓冲
            std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("fastlio2/body_cloud", 10000);
        m_world_cloud_pub_internal = this->create_publisher<sensor_msgs::msg::PointCloud2>("fastlio2/world_cloud_internal", 10000);
        m_world_cloud_pub_legacy = this->create_publisher<sensor_msgs::msg::PointCloud2>("fastlio2/world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("fastlio2/lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("fastlio2/lio_odom", 10000);
        m_imu_pose_pub = this->create_publisher<visualization_msgs::msg::Marker>("fastlio2/imu_pose_marker", 100);
        m_performance_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("fastlio2/performance_metrics", 100);
        m_diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("fastlio2/diagnostics", 100);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 创建地图保存服务
        m_save_maps_service = this->create_service<interface::srv::SaveMaps>(
            "fastlio2/save_maps",
            std::bind(&LIONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
        
        m_save_poses_service = this->create_service<interface::srv::SavePoses>(
            "fastlio2/save_poses",
            std::bind(&LIONode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));

        // 创建协同工作服务
        m_update_pose_service = this->create_service<interface::srv::UpdatePose>(
            "fastlio2/update_pose",
            std::bind(&LIONode::updatePoseCB, this, std::placeholders::_1, std::placeholders::_2));

        m_sync_state_service = this->create_service<interface::srv::SyncState>(
            "fastlio2/sync_state",
            std::bind(&LIONode::syncStateCB, this, std::placeholders::_1, std::placeholders::_2));

        // 协同优化触发客户端
        m_trigger_opt_client = this->create_client<interface::srv::TriggerOptimization>("/coordinator/trigger_optimization");

        // 协同状态发布者
        m_coop_status_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "fastlio2/cooperation_status", 10);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        
        // 内存优化：初始化自适应点云内存池
        m_body_cloud_pool.reset(new CloudType);
        m_world_cloud_pool.reset(new CloudType);

        // 初始分配基于MID360典型点云大小（约20K-30K点）
        size_t initial_pool_size = 35000;
        m_body_cloud_pool->points.reserve(initial_pool_size);
        m_world_cloud_pool->points.reserve(initial_pool_size);

        // 初始化内存池统计
        m_max_observed_cloud_size = 0;
        m_pool_resize_counter = 0;
        
        m_timer = this->create_wall_timer(10ms, std::bind(&LIONode::timerCB, this));

        // 初始化协同状态时间戳，别让不同时间源坑我们
        auto coop_now = this->get_clock()->now();
        m_coop_state.last_pgo_update = coop_now;
        m_coop_state.last_hba_update = coop_now;
        m_coop_state.last_localizer_update = coop_now;

        last_world_cloud_publish_time_ = this->now();
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

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        // 加载缓冲区管理参数
        if (config["max_imu_buffer_size"]) {
            m_builder_config.max_imu_buffer_size = config["max_imu_buffer_size"].as<int>();
        }
        if (config["max_lidar_buffer_size"]) {
            m_builder_config.max_lidar_buffer_size = config["max_lidar_buffer_size"].as<int>();
        }
        if (config["enable_buffer_monitoring"]) {
            m_builder_config.enable_buffer_monitoring = config["enable_buffer_monitoring"].as<bool>();
        }

        RCLCPP_INFO(this->get_logger(), "缓冲区配置 - IMU最大: %d, 激光最大: %d, 监控: %s",
                   m_builder_config.max_imu_buffer_size, m_builder_config.max_lidar_buffer_size,
                   m_builder_config.enable_buffer_monitoring ? "启用" : "禁用");
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            // 智能异常处理：检查时序错误的严重程度
            double time_diff = m_state_data.last_imu_time - timestamp;
            if (time_diff > 0.1) // 时序错误超过100ms，认为是严重错误
            {
                RCLCPP_WARN(this->get_logger(), "严重IMU时序错误 (%.3fs)，清空缓冲区", time_diff);
                std::deque<IMUData>().swap(m_state_data.imu_buffer);
            }
            else
            {
                // 轻微时序错误，只移除冲突的数据点
                RCLCPP_WARN(this->get_logger(), "轻微IMU时序错误 (%.3fs)，移除冲突数据", time_diff);
                while (!m_state_data.imu_buffer.empty() &&
                       m_state_data.imu_buffer.back().time >= timestamp)
                {
                    m_state_data.imu_buffer.pop_back();
                }
            }
        }

        // 检查IMU缓冲区大小，防止无限增长
        while (m_state_data.imu_buffer.size() >= static_cast<size_t>(m_builder_config.max_imu_buffer_size))
        {
            m_state_data.imu_buffer.pop_front();
            if (m_builder_config.enable_buffer_monitoring)
            {
                static int overflow_counter = 0;
                if (++overflow_counter % 100 == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "IMU缓冲区溢出 #%d，丢弃旧数据", overflow_counter);
                }
            }
        }

        // Livox driver publishes linear_acceleration in g; convert to m/s^2 using 9.81
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 9.81,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        static int lidar_counter = 0;
        lidar_counter++;
        if (lidar_counter % 100 == 0) // 每100次回调打印一次
        {
            RCLCPP_INFO(this->get_logger(), "收到激光雷达数据 #%d, 点数量: %u", lidar_counter, msg->point_num);
        }
        
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        
        if (!cloud || cloud->empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "转换后的点云为空或无效!");
            return;
        }
        
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            // 智能异常处理：检查时序错误的严重程度
            double time_diff = m_state_data.last_lidar_time - timestamp;
            if (time_diff > 0.2) // 激光雷达时序错误超过200ms，认为是严重错误
            {
                RCLCPP_WARN(this->get_logger(), "严重激光雷达时序错误 (%.3fs)，清空缓冲区", time_diff);
                std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
                m_state_data.lidar_pushed = false; // 重置推送状态
            }
            else
            {
                // 轻微时序错误，只移除冲突的数据点
                RCLCPP_WARN(this->get_logger(), "轻微激光雷达时序错误 (%.3fs)，移除冲突数据", time_diff);
                while (!m_state_data.lidar_buffer.empty() &&
                       m_state_data.lidar_buffer.back().first >= timestamp)
                {
                    m_state_data.lidar_buffer.pop_back();
                }
                m_state_data.lidar_pushed = false; // 重置推送状态以重新同步
            }
        }

        // 检查激光雷达缓冲区大小，防止无限增长
        while (m_state_data.lidar_buffer.size() >= static_cast<size_t>(m_builder_config.max_lidar_buffer_size))
        {
            m_state_data.lidar_buffer.pop_front();
            if (m_builder_config.enable_buffer_monitoring)
            {
                static int overflow_counter = 0;
                if (++overflow_counter % 10 == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "激光雷达缓冲区溢出 #%d，丢弃旧数据", overflow_counter);
                }
            }
        }

        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        // 使用双重锁机制确保线程安全
        // 注意：总是按照固定顺序获取锁以避免死锁
        std::lock_guard<std::mutex> imu_lock(m_state_data.imu_mutex);
        std::lock_guard<std::mutex> lidar_lock(m_state_data.lidar_mutex);

        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            // 检查点云是否有效
            if (!m_package.cloud || m_package.cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "同步包中检测到无效点云，跳过此帧");
                m_state_data.lidar_buffer.pop_front();
                return false;
            }

            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }

        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }

        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (!pub || pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        // 使用锁保护路径数据的并发访问
        {
            std::lock_guard<std::mutex> lock(m_state_data.path_mutex);
            m_state_data.path.poses.push_back(pose);
            if (m_state_data.path.poses.size() > kMaxPathSize) {
                auto erase_begin = m_state_data.path.poses.begin();
                auto erase_end = erase_begin + (m_state_data.path.poses.size() - kMaxPathSize);
                m_state_data.path.poses.erase(erase_begin, erase_end);
            }
            m_state_data.path.header.stamp = Utils::getTime(time);
            path_pub->publish(m_state_data.path);
        }
    }

    void publishIMUPose(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, std::string frame_id, const double &time)
    {
        if (marker_pub->get_subscription_count() <= 0)
            return;
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = Utils::getTime(time);
        marker.ns = "imu_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 位置
        marker.pose.position.x = m_kf->x().t_wi.x();
        marker.pose.position.y = m_kf->x().t_wi.y();
        marker.pose.position.z = m_kf->x().t_wi.z();
        
        // 姿态
        Eigen::Quaterniond q(m_kf->x().r_wi);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        // 箭头大小和颜色
        marker.scale.x = 0.5;  // 箭头长度
        marker.scale.y = 0.05; // 箭头宽度
        marker.scale.z = 0.05; // 箭头高度
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_nanoseconds(100000000); // 100ms
        
        marker_pub->publish(marker);
    }

    void publishPerformanceMetrics(rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr perf_pub, 
                                 double processing_time_ms, const double &time)
    {
        if (perf_pub->get_subscription_count() <= 0)
            return;
            
        std_msgs::msg::Float64MultiArray metrics;
        metrics.data.resize(4);
        
        // 处理时间 (ms)
        metrics.data[0] = processing_time_ms;
        
        // 点云大小
        metrics.data[1] = m_package.cloud ? m_package.cloud->size() : 0;
        
        // IMU缓冲区大小
        metrics.data[2] = m_state_data.imu_buffer.size();
        
        // 激光雷达缓冲区大小
        metrics.data[3] = m_state_data.lidar_buffer.size();
        
        perf_pub->publish(metrics);
    }

    void publishDiagnostics(rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub,
                          double processing_time_ms, const double &time)
    {
        if (diag_pub->get_subscription_count() <= 0)
            return;
            
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = Utils::getTime(time);
        
        // 系统整体状态
        diagnostic_msgs::msg::DiagnosticStatus system_status;
        system_status.name = "FAST-LIO2 System";
        system_status.hardware_id = "Mid360_SLAM";
        
        // 根据处理时间评估系统状态
        if (processing_time_ms < 20.0) {
            system_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            system_status.message = "系统运行正常";
        } else if (processing_time_ms < 50.0) {
            system_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            system_status.message = "处理延迟较高";
        } else {
            system_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            system_status.message = "处理延迟过高，系统性能不佳";
        }
        
        // 添加详细信息
        diagnostic_msgs::msg::KeyValue kv_processing_time;
        kv_processing_time.key = "processing_time_ms";
        kv_processing_time.value = std::to_string(processing_time_ms);
        system_status.values.push_back(kv_processing_time);
        
        diagnostic_msgs::msg::KeyValue kv_point_count;
        kv_point_count.key = "point_cloud_size";
        kv_point_count.value = std::to_string(m_package.cloud ? m_package.cloud->size() : 0);
        system_status.values.push_back(kv_point_count);
        
        diagnostic_msgs::msg::KeyValue kv_imu_buffer;
        kv_imu_buffer.key = "imu_buffer_size";
        kv_imu_buffer.value = std::to_string(m_state_data.imu_buffer.size());
        system_status.values.push_back(kv_imu_buffer);
        
        diagnostic_msgs::msg::KeyValue kv_lidar_buffer;
        kv_lidar_buffer.key = "lidar_buffer_size";
        kv_lidar_buffer.value = std::to_string(m_state_data.lidar_buffer.size());
        system_status.values.push_back(kv_lidar_buffer);
        
        // 点云质量评估
        diagnostic_msgs::msg::DiagnosticStatus cloud_quality;
        cloud_quality.name = "Point Cloud Quality";
        cloud_quality.hardware_id = "Mid360_Lidar";
        
        if (m_package.cloud && !m_package.cloud->empty()) {
            size_t point_count = m_package.cloud->size();
            
            // 评估点云密度和质量
            if (point_count > 10000) {
                cloud_quality.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                cloud_quality.message = "点云质量良好";
            } else if (point_count > 5000) {
                cloud_quality.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                cloud_quality.message = "点云密度偏低";
            } else {
                cloud_quality.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                cloud_quality.message = "点云密度过低，影响建图质量";
            }
            
            diagnostic_msgs::msg::KeyValue kv_quality;
            kv_quality.key = "point_density_rating";
            if (point_count > 15000) kv_quality.value = "Excellent";
            else if (point_count > 10000) kv_quality.value = "Good";
            else if (point_count > 5000) kv_quality.value = "Fair";
            else kv_quality.value = "Poor";
            cloud_quality.values.push_back(kv_quality);
        } else {
            cloud_quality.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            cloud_quality.message = "无点云数据";
        }
        
        diag_array.status.push_back(system_status);
        diag_array.status.push_back(cloud_quality);
        
        diag_pub->publish(diag_array);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request,
                    std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        try {
            // 确保保存目录存在
            boost::filesystem::path save_path(request->file_path);
            boost::filesystem::path save_dir = save_path.parent_path();
            if (!save_dir.empty() && !boost::filesystem::exists(save_dir)) {
                boost::filesystem::create_directories(save_dir);
            }

            // 从ikd-tree获取全局累积的稠密地图点云
            if (m_builder && m_builder->lidar_processor() && m_builder->status() == BuilderStatus::MAPPING) {
                RCLCPP_INFO(this->get_logger(), "正在保存全局累积地图到: %s", request->file_path.c_str());

                // 使用getGlobalMap()获取ikd-tree中的所有累积点云
                CloudType::Ptr global_map = m_builder->lidar_processor()->getGlobalMap();

                if (global_map && !global_map->empty()) {
                    // 根据文件扩展名选择保存格式
                    std::string file_path = request->file_path;
                    if (file_path.substr(file_path.find_last_of(".") + 1) == "ply") {
                        pcl::io::savePLYFileBinary(file_path, *global_map);
                    } else {
                        // 默认保存为PCD格式
                        if (file_path.substr(file_path.find_last_of(".") + 1) != "pcd") {
                            file_path += ".pcd";
                        }
                        pcl::io::savePCDFileBinary(file_path, *global_map);
                    }

                    response->success = true;
                    response->message = "全局累积地图保存成功，点云数量: " + std::to_string(global_map->size());
                    RCLCPP_INFO(this->get_logger(), "全局地图保存完成，累积点数: %zu", global_map->size());
                } else {
                    response->success = false;
                    response->message = "无法获取全局地图数据或地图为空";
                }
            } else {
                response->success = false;
                response->message = "SLAM系统未处于建图状态";
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "保存失败: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "地图保存失败: %s", e.what());
        }
    }

    void savePosesCB(const std::shared_ptr<interface::srv::SavePoses::Request> request,
                     std::shared_ptr<interface::srv::SavePoses::Response> response)
    {
        try {
            // 确保保存目录存在
            boost::filesystem::path save_path(request->file_path);
            boost::filesystem::path save_dir = save_path.parent_path();
            if (!save_dir.empty() && !boost::filesystem::exists(save_dir)) {
                boost::filesystem::create_directories(save_dir);
            }

            RCLCPP_INFO(this->get_logger(), "正在保存轨迹数据到: %s", request->file_path.c_str());
            
            std::ofstream trajectory_file(request->file_path);
            if (!trajectory_file.is_open()) {
                response->success = false;
                response->message = "无法打开文件进行写入: " + request->file_path;
                return;
            }

            // 写入轨迹头信息
            trajectory_file << "# 时间戳 x y z qx qy qz qw\n";
            trajectory_file << "# MID360 SLAM轨迹数据\n";
            trajectory_file << "# 坐标系: " << m_node_config.world_frame << "\n";
            
            // 保存轨迹点
            size_t pose_count = 0;
            for (const auto& pose : m_state_data.path.poses) {
                double timestamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;
                trajectory_file << std::fixed << std::setprecision(6) << timestamp << " "
                               << std::setprecision(6) << pose.pose.position.x << " "
                               << std::setprecision(6) << pose.pose.position.y << " "
                               << std::setprecision(6) << pose.pose.position.z << " "
                               << std::setprecision(6) << pose.pose.orientation.x << " "
                               << std::setprecision(6) << pose.pose.orientation.y << " "
                               << std::setprecision(6) << pose.pose.orientation.z << " "
                               << std::setprecision(6) << pose.pose.orientation.w << "\n";
                pose_count++;
            }

            trajectory_file.close();
            
            response->success = true;
            response->message = "轨迹保存成功，位姿数量: " + std::to_string(pose_count);
            RCLCPP_INFO(this->get_logger(), "轨迹保存完成，位姿数: %zu", pose_count);
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "保存失败: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "轨迹保存失败: %s", e.what());
        }
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        
        // 自适应性能优化逻辑
        static double avg_processing_time = 20.0;
        static int skip_counter = 0;
        avg_processing_time = 0.9 * avg_processing_time + 0.1 * time_used;
        
        // 如果处理时间过长，跳过部分可视化发布
        bool skip_visualization = false;
        if (avg_processing_time > 30.0) {
            skip_counter++;
            if (skip_counter % 2 == 0) {
                skip_visualization = true;
            }
        } else {
            skip_counter = 0;
        }
        
        if (m_node_config.print_time_cost)
        {
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms, Avg: %.2f ms", time_used, avg_processing_time);
        }

        // 状态监控和调试信息
        static int debug_counter = 0;
        debug_counter++;
        if (debug_counter % 10 == 0) // 每10次循环打印一次状态
        {
            std::string status_str;
            switch(m_builder->status()) {
                case BuilderStatus::IMU_INIT:
                    status_str = "IMU_INIT";
                    break;
                case BuilderStatus::MAP_INIT:
                    status_str = "MAP_INIT";
                    break;
                case BuilderStatus::MAPPING:
                    status_str = "MAPPING";
                    break;
                default:
                    status_str = "UNKNOWN";
            }
            RCLCPP_INFO(this->get_logger(), "当前状态: %s, IMU缓冲区大小: %zu, 激光缓冲区大小: %zu", 
                       status_str.c_str(), m_state_data.imu_buffer.size(), m_state_data.lidar_buffer.size());
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        // 自适应发布策略：在高负载时优化发布频率
        if (!skip_visualization) {
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

            static int global_map_counter = 0;
            global_map_counter++;
            const auto now_time = this->now();

            bool has_internal_sub = m_world_cloud_pub_internal && m_world_cloud_pub_internal->get_subscription_count() > 0;
            bool has_legacy_sub = m_world_cloud_pub_legacy && m_world_cloud_pub_legacy->get_subscription_count() > 0;
            bool should_publish_world = !m_world_cloud_published_once || has_internal_sub || has_legacy_sub;

            if (should_publish_world &&
                global_map_counter % 10 == 0 &&
                (now_time - last_world_cloud_publish_time_).seconds() >= 1.0) {
                CloudType::Ptr world_cloud = m_builder->lidar_processor()->getGlobalMap();
                if (world_cloud && !world_cloud->empty()) {
                    publishCloud(m_world_cloud_pub_internal, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);
                    publishCloud(m_world_cloud_pub_legacy, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);
                    last_world_cloud_publish_time_ = now_time;
                    m_world_cloud_published_once = true;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "发布高密度全局地图，点数: %zu", world_cloud->size());
                }
            }
            
            // 发布IMU姿态可视化
            publishIMUPose(m_imu_pose_pub, m_node_config.world_frame, m_package.cloud_end_time);
        }

        // 始终发布关键数据（里程计和轨迹）
        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        
        // 发布性能监控数据
        publishPerformanceMetrics(m_performance_pub, time_used, m_package.cloud_end_time);
        
        // 发布诊断信息（降低频率）
        static int diag_counter = 0;
        if (++diag_counter % 5 == 0) {
            publishDiagnostics(m_diagnostics_pub, time_used, m_package.cloud_end_time);
        }

        // 内存池自适应管理
        manageMemoryPools();
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    static constexpr size_t kMaxPathSize = 2000;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub_internal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub_legacy;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_imu_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_performance_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnostics_pub;

    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_maps_service;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr m_save_poses_service;

    // 协同工作服务
    rclcpp::Service<interface::srv::UpdatePose>::SharedPtr m_update_pose_service;
    rclcpp::Service<interface::srv::SyncState>::SharedPtr m_sync_state_service;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_coop_status_pub;
    // 协同优化触发客户端（调用协调器）
    rclcpp::Client<interface::srv::TriggerOptimization>::SharedPtr m_trigger_opt_client;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    
    // 内存优化：点云内存池
    CloudType::Ptr m_body_cloud_pool;
    CloudType::Ptr m_world_cloud_pool;

    // 内存池管理统计
    size_t m_max_observed_cloud_size;
    int m_pool_resize_counter;

    // 地图发布节流
    rclcpp::Time last_world_cloud_publish_time_;
    bool m_world_cloud_published_once = false;

    // 协同状态
    struct CooperationState {
        int feedback_count = 0;
        double average_optimization_score = 0.0;
        rclcpp::Time last_pgo_update;
        rclcpp::Time last_hba_update;
        rclcpp::Time last_localizer_update;
    } m_coop_state;

    // 协同工作回调函数
    void updatePoseCB(const std::shared_ptr<interface::srv::UpdatePose::Request> request,
                      std::shared_ptr<interface::srv::UpdatePose::Response> response);

    void syncStateCB(const std::shared_ptr<interface::srv::SyncState::Request> request,
                     std::shared_ptr<interface::srv::SyncState::Response> response);

    // 内存池管理函数
    void manageMemoryPools();

    bool updateIESKFWithOptimizedPose(const geometry_msgs::msg::PoseWithCovariance& optimized_pose);
    double calculateUpdateWeight(double position_error);
    void updateCooperationState(const std::string& source, double score);
    geometry_msgs::msg::PoseWithCovariance getCurrentPoseWithCovariance();
    geometry_msgs::msg::Pose getCurrentGlobalPose();
};

// 协同工作回调函数实现
void LIONode::updatePoseCB(const std::shared_ptr<interface::srv::UpdatePose::Request> request,
                           std::shared_ptr<interface::srv::UpdatePose::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(),
            "🔄 收到来自 %s 的位姿优化反馈，分数: %.4f",
            request->source_component.c_str(), request->optimization_score);

        // 验证优化结果的可信度
        if (request->optimization_score < 0.5) {
            response->success = false;
            response->message = "优化分数低于0.5阈值，拒绝更新";
            return;
        }

        // 更新IESKF状态
        bool update_success = updateIESKFWithOptimizedPose(request->optimized_pose);

        if (update_success) {
            updateCooperationState(request->source_component, request->optimization_score);

            response->success = true;
            response->message = "位姿更新成功";
            response->current_pose = getCurrentPoseWithCovariance();
            response->state_update_time = this->get_clock()->now().seconds();

            m_coop_state.feedback_count++;

            RCLCPP_INFO(this->get_logger(),
                "✅ SLAM状态更新成功 (来源: %s, 累计反馈: %d次)",
                request->source_component.c_str(), m_coop_state.feedback_count);
        } else {
            response->success = false;
            response->message = "IESKF状态更新失败";
        }

    } catch (const std::exception& e) {
        response->success = false;
        response->message = "位姿更新异常: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "位姿更新异常: %s", e.what());
    }
}

void LIONode::syncStateCB(const std::shared_ptr<interface::srv::SyncState::Request> request,
                          std::shared_ptr<interface::srv::SyncState::Response> response)
{
    RCLCPP_INFO(this->get_logger(),
        "🔄 状态同步请求 - 组件: %s, 类型: %d",
        request->component_name.c_str(), request->optimization_type);

    try {
        auto now = this->get_clock()->now();
        switch(request->optimization_type) {
            case 0: // PGO优化
                m_coop_state.last_pgo_update = now;
                break;
            case 1: // HBA优化
                m_coop_state.last_hba_update = now;
                break;
            case 2: // Localizer重定位
                m_coop_state.last_localizer_update = now;
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "未知的优化类型: %d", request->optimization_type);
        }

        response->success = true;
        response->message = "状态同步成功";
        response->current_global_pose = getCurrentGlobalPose();
        response->sync_timestamp = now.seconds();

    } catch (const std::exception& e) {
        response->success = false;
        response->message = "状态同步失败: " + std::string(e.what());
    }
}

bool LIONode::updateIESKFWithOptimizedPose(const geometry_msgs::msg::PoseWithCovariance& optimized_pose)
{
    try {
        // 提取优化后的位姿
        V3D optimized_position(
            optimized_pose.pose.position.x,
            optimized_pose.pose.position.y,
            optimized_pose.pose.position.z
        );

        Eigen::Quaterniond optimized_quat(
            optimized_pose.pose.orientation.w,
            optimized_pose.pose.orientation.x,
            optimized_pose.pose.orientation.y,
            optimized_pose.pose.orientation.z
        );

        M3D optimized_rotation = optimized_quat.toRotationMatrix();

        // 计算与当前状态的差异
        V3D current_position = m_kf->x().t_wi;
        M3D current_rotation = m_kf->x().r_wi;

        V3D position_diff = optimized_position - current_position;
        double position_error = position_diff.norm();

        // 渐进式更新策略
        double update_weight = calculateUpdateWeight(position_error);

        if (update_weight > 0.01) {
            // 更新IESKF状态
            V3D updated_position = current_position + update_weight * position_diff;

            // 旋转插值更新
            Eigen::Quaterniond current_quat(current_rotation);
            Eigen::Quaterniond updated_quat = current_quat.slerp(update_weight, optimized_quat);
            M3D updated_rotation = updated_quat.toRotationMatrix();

            // 应用更新到IESKF
            m_kf->x().t_wi = updated_position;
            m_kf->x().r_wi = updated_rotation;

            RCLCPP_DEBUG(this->get_logger(),
                "IESKF状态更新: 位置误差=%.3fm, 更新权重=%.3f",
                position_error, update_weight);

            return true;
        }

        return false;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "IESKF状态更新异常: %s", e.what());
        return false;
    }
}

double LIONode::calculateUpdateWeight(double position_error)
{
    if (position_error < 0.1) {
        return 0.8;  // 小误差：高权重
    } else if (position_error < 0.5) {
        return 0.5;  // 中等误差：中权重
    } else if (position_error < 2.0) {
        return 0.2;  // 大误差：低权重
    } else {
        return 0.05; // 超大误差：很低权重
    }
}

void LIONode::updateCooperationState(const std::string& source, double score)
{
    // 更新平均分数
    double alpha = 0.1;
    m_coop_state.average_optimization_score =
        alpha * score + (1.0 - alpha) * m_coop_state.average_optimization_score;

    auto now = this->get_clock()->now();

    std::string source_lower = source;
    std::transform(source_lower.begin(), source_lower.end(), source_lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (source_lower.find("pgo") != std::string::npos) {
        m_coop_state.last_pgo_update = now;
    } else if (source_lower.find("hba") != std::string::npos) {
        m_coop_state.last_hba_update = now;
    } else if (source_lower.find("localizer") != std::string::npos ||
               source_lower.find("relocalization") != std::string::npos) {
        m_coop_state.last_localizer_update = now;
    }

    // 发布协同状态
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(6);
    msg.data[0] = m_coop_state.feedback_count;
    msg.data[1] = m_coop_state.average_optimization_score;
    msg.data[2] = (now - m_coop_state.last_pgo_update).seconds();
    msg.data[3] = (now - m_coop_state.last_hba_update).seconds();
    msg.data[4] = (now - m_coop_state.last_localizer_update).seconds();
    msg.data[5] = 1.0; // 协同功能已启用

    m_coop_status_pub->publish(msg);
}

geometry_msgs::msg::PoseWithCovariance LIONode::getCurrentPoseWithCovariance()
{
    geometry_msgs::msg::PoseWithCovariance pose_cov;

    V3D position = m_kf->x().t_wi;
    pose_cov.pose.position.x = position.x();
    pose_cov.pose.position.y = position.y();
    pose_cov.pose.position.z = position.z();

    Eigen::Quaterniond quat(m_kf->x().r_wi);
    pose_cov.pose.orientation.w = quat.w();
    pose_cov.pose.orientation.x = quat.x();
    pose_cov.pose.orientation.y = quat.y();
    pose_cov.pose.orientation.z = quat.z();

    // 简化协方差矩阵
    for (int i = 0; i < 36; i++) {
        pose_cov.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }

    return pose_cov;
}

geometry_msgs::msg::Pose LIONode::getCurrentGlobalPose()
{
    geometry_msgs::msg::Pose pose;

    V3D position = m_kf->x().t_wi;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();

    Eigen::Quaterniond quat(m_kf->x().r_wi);
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();

    return pose;
}

void LIONode::manageMemoryPools()
{
    // 每100次调用进行一次内存池检查（约1秒检查一次，因为定时器是10ms）
    static int check_counter = 0;
    if (++check_counter % 100 != 0) {
        return;
    }

    // 获取当前点云大小
    size_t current_cloud_size = 0;
    if (m_package.cloud && !m_package.cloud->empty()) {
        current_cloud_size = m_package.cloud->size();

        // 更新观察到的最大点云大小
        if (current_cloud_size > m_max_observed_cloud_size) {
            m_max_observed_cloud_size = current_cloud_size;
        }
    }

    // 检查是否需要调整内存池大小 - 使用points向量的容量
    size_t current_capacity = m_body_cloud_pool->points.capacity();
    size_t target_capacity = m_max_observed_cloud_size * 1.2; // 预留20%缓冲

    // 如果观察到的最大大小超过当前容量的80%，则扩容
    if (m_max_observed_cloud_size > current_capacity * 0.8 && target_capacity > current_capacity) {
        try {
            m_body_cloud_pool->points.reserve(target_capacity);
            m_world_cloud_pool->points.reserve(target_capacity);
            m_pool_resize_counter++;

            if (m_builder_config.enable_buffer_monitoring) {
                RCLCPP_INFO(this->get_logger(),
                           "内存池自适应调整 #%d: %zu -> %zu 点 (观察到最大: %zu)",
                           m_pool_resize_counter, current_capacity, target_capacity, m_max_observed_cloud_size);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "内存池扩容失败: %s", e.what());
        }
    }

    // 每1000次检查重置统计以适应环境变化
    if (check_counter % 10000 == 0) {
        m_max_observed_cloud_size = current_cloud_size;
        if (m_builder_config.enable_buffer_monitoring) {
            RCLCPP_INFO(this->get_logger(), "重置内存池统计，当前容量: %zu", m_body_cloud_pool->points.capacity());
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LIONode>();

    RCLCPP_INFO(node->get_logger(), "🚀 协同增强FAST-LIO2系统启动");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
