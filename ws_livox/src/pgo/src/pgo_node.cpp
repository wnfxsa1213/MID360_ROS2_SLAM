#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include "interface/srv/update_pose.hpp"
#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_optimized_poses_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/pgo/optimized_poses", 1000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // 创建位姿更新客户端，用于向FAST-LIO2反馈优化结果
        m_update_pose_client = this->create_client<interface::srv::UpdatePose>("fastlio2/update_pose");
        
        // 创建HBA客户端，用于在线精细化
        m_hba_refine_client = this->create_client<interface::srv::RefineMap>("/hba/refine_map");
        m_hba_save_client = this->create_client<interface::srv::SavePoses>("/hba/save_poses");
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
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
        
        // 加载新增的增强功能参数（带默认值）
        m_pgo_config.enable_pose_feedback = config["enable_pose_feedback"].as<bool>(true);
        m_pgo_config.feedback_frequency_hz = config["feedback_frequency_hz"].as<double>(2.0);
        m_pgo_config.max_optimization_time_ms = config["max_optimization_time_ms"].as<int>(200);
        m_pgo_config.enable_loop_visualization = config["enable_loop_visualization"].as<bool>(true);
        m_pgo_config.keyframe_skip_distance = config["keyframe_skip_distance"].as<double>(0.1);
        
        RCLCPP_INFO(this->get_logger(), "PGO配置已加载 - 关键帧阈值: %.2fm/%.1f°, 回环搜索: %.1fm/%.1fs", 
                   m_pgo_config.key_pose_delta_trans, m_pgo_config.key_pose_delta_deg,
                   m_pgo_config.loop_search_radius, m_pgo_config.loop_time_tresh);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        // 防止自引用变换（frame_id与child_frame_id相同）
        if (m_node_config.map_frame == m_node_config.local_frame) {
            RCLCPP_WARN_ONCE(this->get_logger(), "PGO跳过自引用TF变换: %s -> %s", 
                             m_node_config.map_frame.c_str(), m_node_config.local_frame.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    void timerCB()
    {
        if (m_state.cloud_buffer.size() == 0)
            return;
        CloudWithPose cp = m_state.cloud_buffer.front();
        // 清理队列
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
        }
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        if (!m_pgo->addKeyPose(cp))
        {

            sendBroadCastTF(cur_time);
            return;
        }

        m_pgo->searchForLoopPairs();

        m_pgo->smoothAndUpdate();

        // 将优化后的最新位姿反馈给FAST-LIO2（根据配置决定）
        if (m_pgo_config.enable_pose_feedback) {
            sendPoseUpdateToFASTLIO2();
        }

        sendBroadCastTF(cur_time);

        publishLoopMarkers(cur_time);
        
        // 发布优化后的关键帧位姿给HBA
        publishOptimizedPoses();
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }
        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        
        // 如果保存了patches，则自动触发HBA在线精细化
        if (request->save_patches && request->enable_hba_refine) {
            triggerHBARefinement(p_dir.string());
        }
        
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

    void triggerHBARefinement(const std::string& maps_path)
    {
        if (!m_hba_refine_client->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "HBA refine service not available, skipping refinement");
            return;
        }
        
        auto request = std::make_shared<interface::srv::RefineMap::Request>();
        request->maps_path = maps_path;
        
        RCLCPP_INFO(this->get_logger(), "触发HBA在线精细化优化: %s", maps_path.c_str());
        
        // 异步调用HBA精细化服务
        auto result_future = m_hba_refine_client->async_send_request(
            request,
            [this](rclcpp::Client<interface::srv::RefineMap>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "HBA在线精细化成功: %s", response->message.c_str());
                        // 精细化完成后，可以保存优化后的位姿
                        saveHBARefinedPoses();
                    } else {
                        RCLCPP_WARN(this->get_logger(), "HBA在线精细化失败: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "HBA在线精细化异常: %s", e.what());
                }
            }
        );
    }
    
    void saveHBARefinedPoses()
    {
        if (!m_hba_save_client->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "HBA save service not available");
            return;
        }
        
        auto request = std::make_shared<interface::srv::SavePoses::Request>();
        request->file_path = "/tmp/hba_refined_poses.txt";
        
        auto result_future = m_hba_save_client->async_send_request(
            request,
            [this](rclcpp::Client<interface::srv::SavePoses>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "HBA优化位姿保存成功: %s", response->message.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "HBA优化位姿保存失败: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "HBA位姿保存异常: %s", e.what());
                }
            }
        );
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_optimized_poses_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    rclcpp::Client<interface::srv::UpdatePose>::SharedPtr m_update_pose_client;
    rclcpp::Client<interface::srv::RefineMap>::SharedPtr m_hba_refine_client;
    rclcpp::Client<interface::srv::SavePoses>::SharedPtr m_hba_save_client;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;

    // 向FAST-LIO2发送优化后的位姿（增强版本）
    void sendPoseUpdateToFASTLIO2()
    {
        if (!m_update_pose_client || m_pgo->keyPoses().empty()) {
            RCLCPP_DEBUG(this->get_logger(), "UpdatePose客户端未初始化或无关键帧");
            return;
        }

        // 检查服务是否可用
        int timeout_ms = std::min(m_pgo_config.max_optimization_time_ms / 2, 100);
        if (!m_update_pose_client->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "FAST-LIO2 位姿更新服务不可用 (超时: %dms)", timeout_ms);
            return;
        }

        // 获取最新的优化后位姿
        const auto& latest_pose = m_pgo->keyPoses().back();
        
        // 创建服务请求
        auto request = std::make_shared<interface::srv::UpdatePose::Request>();
        request->timestamp = latest_pose.time;  // 时间戳
        request->x = latest_pose.t_global.x();
        request->y = latest_pose.t_global.y();
        request->z = latest_pose.t_global.z();
        
        Eigen::Quaterniond q(latest_pose.r_global);
        q.normalize(); // 确保四元数标准化
        request->qw = q.w();
        request->qx = q.x();
        request->qy = q.y();
        request->qz = q.z();

        RCLCPP_DEBUG(this->get_logger(), "发送位姿更新: [%.3f, %.3f, %.3f] at time %.3f",
                    request->x, request->y, request->z, request->timestamp);

        // 异步调用服务（避免阻塞）
        auto result_future = m_update_pose_client->async_send_request(request);
        
        // 根据配置的最大优化时间等待结果
        int wait_timeout = m_pgo_config.max_optimization_time_ms / 4; // 不超过1/4的优化时间
        if (result_future.wait_for(std::chrono::milliseconds(wait_timeout)) == std::future_status::ready) {
            try {
                auto response = result_future.get();
                if (response->success) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                        "✅ PGO位姿反馈成功: [%.3f, %.3f, %.3f] -> FAST-LIO2",
                        request->x, request->y, request->z);
                    
                    // 增加成功计数器（可用于统计）
                    static int success_count = 0;
                    success_count++;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "❌ FAST-LIO2位姿更新失败: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "位姿更新异常: %s", e.what());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "位姿更新超时 (%dms)，继续执行", wait_timeout);
        }
    }
    
    // 发布优化后的关键帧位姿给HBA（新增功能）
    void publishOptimizedPoses()
    {
        if (!m_optimized_poses_pub || m_pgo->keyPoses().empty()) {
            return;
        }
        
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->now();
        pose_array.header.frame_id = m_node_config.map_frame;
        
        // 获取所有优化后的关键帧位姿
        const auto& key_poses = m_pgo->keyPoses();
        pose_array.poses.reserve(key_poses.size());
        
        for (const auto& kp : key_poses) {
            geometry_msgs::msg::Pose pose;
            
            // 位置
            pose.position.x = kp.t_global.x();
            pose.position.y = kp.t_global.y();
            pose.position.z = kp.t_global.z();
            
            // 旋转（转换为四元数）
            Eigen::Quaterniond q(kp.r_global);
            q.normalize();
            pose.orientation.w = q.w();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            
            pose_array.poses.push_back(pose);
        }
        
        // 发布给HBA监控
        m_optimized_poses_pub->publish(pose_array);
        
        RCLCPP_DEBUG(this->get_logger(), "发布%zu个优化位姿给HBA", pose_array.poses.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}