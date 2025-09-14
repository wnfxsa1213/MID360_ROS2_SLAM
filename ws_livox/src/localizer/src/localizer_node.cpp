#include <queue>
#include <mutex>
#include <filesystem>
#include <pcl/io/pcd_io.h>
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
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include "interface/srv/update_pose.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
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

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // Create client for FAST-LIO2 pose update feedback
        m_update_pose_client = this->create_client<interface::srv::UpdatePose>("/fastlio2/update_pose");

        // 添加自动地图加载功能
        m_auto_load_map_service = this->create_service<interface::srv::IsValid>(
            "auto_load_map", 
            std::bind(&LocalizerNode::autoLoadMapCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // 启动时自动尝试加载地图
        if (tryAutoLoadMap()) {
            RCLCPP_INFO(this->get_logger(), "✅ 自动地图加载成功");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 自动地图加载失败，使用服务模式");
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
                
                // Send relocalization feedback to FAST-LIO2
                sendRelocalizationFeedback(map_body_r, map_body_t, current_time);
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

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
        // 防止自引用变换（frame_id与child_frame_id相同）
        if (m_config.map_frame == m_config.local_frame) {
            RCLCPP_WARN_ONCE(this->get_logger(), "LOCALIZER跳过自引用TF变换: %s -> %s", 
                             m_config.map_frame.c_str(), m_config.local_frame.c_str());
            return;
        }

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

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
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

    void sendRelocalizationFeedback(const M3D& map_body_r, const V3D& map_body_t, const builtin_interfaces::msg::Time& timestamp)
    {
        if (!m_update_pose_client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(this->get_logger(), "FAST-LIO2 update_pose service not available");
            return;
        }

        // Convert rotation matrix to quaternion
        Eigen::Quaterniond quat(map_body_r);

        auto request = std::make_shared<interface::srv::UpdatePose::Request>();
        request->timestamp = rclcpp::Time(timestamp).seconds();
        request->x = map_body_t.x();
        request->y = map_body_t.y();
        request->z = map_body_t.z();
        request->qw = quat.w();
        request->qx = quat.x();
        request->qy = quat.y();
        request->qz = quat.z();

        // Send async request with callback
        auto result_future = m_update_pose_client->async_send_request(
            request,
            [this](rclcpp::Client<interface::srv::UpdatePose>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "重定位反馈发送成功: %s", response->message.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "重定位反馈失败: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "重定位反馈异常: %s", e.what());
                }
            }
        );
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Client<interface::srv::UpdatePose>::SharedPtr m_update_pose_client;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_auto_load_map_service;
    
    // 自动地图加载功能
    bool tryAutoLoadMap()
    {
        // 尝试加载默认地图路径
        std::vector<std::string> potential_map_paths = {
            "./saved_maps/optimized_map.pcd",
            "./saved_maps/map.pcd", 
            "./saved_maps/world_cloud.pcd",
            "../saved_maps/optimized_map.pcd",
            "../saved_maps/map.pcd"
        };
        
        for (const auto& path : potential_map_paths) {
            if (std::filesystem::exists(path)) {
                RCLCPP_INFO(this->get_logger(), "🗺️ 尝试加载地图: %s", path.c_str());
                
                CloudType::Ptr map_cloud(new CloudType);
                if (pcl::io::loadPCDFile(path, *map_cloud) == 0 && !map_cloud->empty()) {
                    m_localizer->loadMap(path);
                    
                    // 发布地图点云供可视化
                    sensor_msgs::msg::PointCloud2 map_msg;
                    pcl::toROSMsg(*map_cloud, map_msg);
                    map_msg.header.frame_id = m_config.map_frame;
                    map_msg.header.stamp = this->now();
                    m_map_cloud_pub->publish(map_msg);
                    
                    RCLCPP_INFO(this->get_logger(), "✅ 成功加载地图: %s (%zu点)", 
                               path.c_str(), map_cloud->size());
                    return true;
                } else {
                    RCLCPP_WARN(this->get_logger(), "❌ 地图加载失败: %s", path.c_str());
                }
            }
        }
        
        return false;
    }
    
    void autoLoadMapCB(const std::shared_ptr<interface::srv::IsValid::Request> request,
                       std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        (void)request; // 避免未使用参数警告
        response->valid = tryAutoLoadMap();
        if (response->valid) {
            RCLCPP_INFO(this->get_logger(), "📋 服务请求：地图重新加载成功");
        } else {
            RCLCPP_WARN(this->get_logger(), "📋 服务请求：地图加载失败");
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
