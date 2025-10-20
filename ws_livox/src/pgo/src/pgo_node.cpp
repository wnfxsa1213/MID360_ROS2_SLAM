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
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cctype>
#include <rcutils/logging.h>
#include <filesystem>
#include <atomic>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include "interface/srv/get_optimized_pose.hpp"
#include "interface/srv/incremental_refine.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "fastlio2/body_cloud";
    std::string odom_topic = "fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    bool enable_hba_incremental = true;
    int hba_incremental_window = 20;
    int hba_incremental_iterations = 2;
    int hba_trigger_interval = 50;
    std::string log_level = "INFO";
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
        configureLogger();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("pgo/loop_markers", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
            "pgo/save_maps",
            std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

        // 提供优化结果查询服务（供协调器获取位姿与轨迹）
        m_get_optimized_pose_srv = this->create_service<interface::srv::GetOptimizedPose>(
            "pgo/get_optimized_pose",
            std::bind(&PGONode::getOptimizedPoseCB, this, std::placeholders::_1, std::placeholders::_2));

        m_hba_incremental_client = this->create_client<interface::srv::IncrementalRefine>(
            "hba/incremental_refine");
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

        // 可选：高级几何验证与约束管理参数（存在则覆盖默认值）
        if (config["min_inlier_ratio"]) {
            m_pgo_config.min_inlier_ratio = config["min_inlier_ratio"].as<double>();
        }
        if (config["icp_max_distance"]) {
            m_pgo_config.icp_max_distance = config["icp_max_distance"].as<double>();
        }
        if (config["max_candidates"]) {
            m_pgo_config.max_candidates = config["max_candidates"].as<int>();
        }
        if (config["min_index_separation"]) {
            m_pgo_config.min_index_separation = config["min_index_separation"].as<int>();
        }
        if (config["inlier_threshold"]) {
            m_pgo_config.inlier_threshold = config["inlier_threshold"].as<double>();
        }
        if (config["icp_max_iterations"]) {
            m_pgo_config.icp_max_iterations = config["icp_max_iterations"].as<int>();
        }
        if (config["icp_transformation_epsilon"]) {
            m_pgo_config.icp_transformation_epsilon = config["icp_transformation_epsilon"].as<double>();
        }
        if (config["icp_euclidean_fitness_epsilon"]) {
            m_pgo_config.icp_euclidean_fitness_epsilon = config["icp_euclidean_fitness_epsilon"].as<double>();
        }
        if (config["isam_relinearize_threshold"]) {
            m_pgo_config.isam_relinearize_threshold = config["isam_relinearize_threshold"].as<double>();
        }
        if (config["isam_relinearize_skip"]) {
            m_pgo_config.isam_relinearize_skip = config["isam_relinearize_skip"].as<int>();
        }
        if (config["loop_retry_max_attempts"]) {
            m_pgo_config.loop_retry_max_attempts = config["loop_retry_max_attempts"].as<int>();
        }
        if (config["loop_retry_interval"]) {
            m_pgo_config.loop_retry_interval = config["loop_retry_interval"].as<double>();
        }
        if (config["loop_retry_capacity"]) {
            m_pgo_config.loop_retry_capacity = config["loop_retry_capacity"].as<size_t>();
        }
        if (config["loop_retry_score_relax"]) {
            m_pgo_config.loop_retry_score_relax = config["loop_retry_score_relax"].as<double>();
        }
        if (config["loop_retry_inlier_relax"]) {
            m_pgo_config.loop_retry_inlier_relax = config["loop_retry_inlier_relax"].as<double>();
        }
        if (config["enable_hba_incremental"]) {
            m_node_config.enable_hba_incremental = config["enable_hba_incremental"].as<bool>();
        }
        if (config["hba_incremental_window"]) {
            m_node_config.hba_incremental_window = config["hba_incremental_window"].as<int>();
        }
        if (config["hba_incremental_iterations"]) {
            m_node_config.hba_incremental_iterations = config["hba_incremental_iterations"].as<int>();
        }
        if (config["hba_trigger_interval"]) {
            m_node_config.hba_trigger_interval = config["hba_trigger_interval"].as<int>();
        }
        if (config["log_level"]) {
            m_node_config.log_level = config["log_level"].as<std::string>();
        }
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
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
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
        }
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;

        bool loop_closed = false;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (!m_pgo->addKeyPose(cp))
            {
                sendBroadCastTF(cur_time);
                return;
            }

            m_pgo->searchForLoopPairs();

            loop_closed = m_pgo->smoothAndUpdate();

            sendBroadCastTF(cur_time);

            publishLoopMarkers(cur_time);
        }

        triggerIncrementalHBAAsync(loop_closed);
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
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    rclcpp::Service<interface::srv::GetOptimizedPose>::SharedPtr m_get_optimized_pose_srv;
    rclcpp::Client<interface::srv::IncrementalRefine>::SharedPtr m_hba_incremental_client;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::atomic<bool> m_hba_request_inflight{false};
    size_t m_last_hba_trigger_pose_count = 0;
    std::mutex m_pgo_mutex;

    void triggerIncrementalHBAAsync(bool loop_closed);
    PoseUpdate poseFromMsg(const geometry_msgs::msg::Pose &pose_msg) const;
    double computeOptimizationScore();
    void configureLogger();

    void getOptimizedPoseCB(const std::shared_ptr<interface::srv::GetOptimizedPose::Request> /*request*/,
                            std::shared_ptr<interface::srv::GetOptimizedPose::Response> response)
    {
        std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
        if (m_pgo->keyPoses().empty()) {
            response->success = false;
            response->message = "No key poses available";
            return;
        }

        // 使用最后一个关键帧的全局位姿作为当前优化估计
        const auto& last = m_pgo->keyPoses().back();
        geometry_msgs::msg::PoseWithCovariance pose;
        pose.pose.position.x = last.t_global.x();
        pose.pose.position.y = last.t_global.y();
        pose.pose.position.z = last.t_global.z();
        Eigen::Quaterniond q(last.r_global);
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        double score = computeOptimizationScore();

        // 填充优化后的轨迹（所有关键帧）
        geometry_msgs::msg::PoseArray traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = m_node_config.map_frame;
        for (const auto& kp : m_pgo->keyPoses()) {
            geometry_msgs::msg::Pose p;
            p.position.x = kp.t_global.x();
            p.position.y = kp.t_global.y();
            p.position.z = kp.t_global.z();
            Eigen::Quaterniond qq(kp.r_global);
            p.orientation.w = qq.w();
            p.orientation.x = qq.x();
            p.orientation.y = qq.y();
            p.orientation.z = qq.z();
            traj.poses.push_back(p);
        }

        response->success = true;
        response->message = "OK";
        response->optimized_pose = pose;
        response->optimized_trajectory = traj;
        response->optimization_score = score;
    }
};

void PGONode::triggerIncrementalHBAAsync(bool loop_closed)
{
    if (!m_node_config.enable_hba_incremental || !loop_closed) {
        return;
    }

    if (!m_hba_incremental_client || m_hba_request_inflight.load()) {
        return;
    }

    if (!m_hba_incremental_client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "HBA incremental service unavailable");
        return;
    }

    std::vector<KeyPoseWithCloud> keyposes_copy;
    {
        std::lock_guard<std::mutex> lock(m_pgo_mutex);
        keyposes_copy = m_pgo->keyPoses();
    }

    size_t total = keyposes_copy.size();
    if (total == 0) {
        return;
    }

    int window_cfg = std::max(1, m_node_config.hba_incremental_window);
    size_t window = std::min(static_cast<size_t>(window_cfg), total);
    if (window < static_cast<size_t>(window_cfg)) {
        return;
    }

    int interval_cfg = std::max(1, m_node_config.hba_trigger_interval);
    if (total - m_last_hba_trigger_pose_count < static_cast<size_t>(interval_cfg)) {
        return;
    }

    size_t start_idx = total - window;
    auto request = std::make_shared<interface::srv::IncrementalRefine::Request>();
    request->reset = true;
    request->run_optimization = true;
    request->max_iterations = m_node_config.hba_incremental_iterations;
    request->keyframe_poses.header.stamp = this->now();
    request->keyframe_poses.header.frame_id = m_node_config.map_frame;
    request->keyframe_poses.poses.reserve(window);
    request->keyframe_clouds.reserve(window);
    request->keyframe_indices.reserve(window);

    for (size_t i = start_idx; i < total; ++i) {
        const auto &kp = keyposes_copy[i];
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = kp.t_global.x();
        pose_msg.position.y = kp.t_global.y();
        pose_msg.position.z = kp.t_global.z();
        Eigen::Quaterniond q(kp.r_global);
        pose_msg.orientation.w = q.w();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        request->keyframe_poses.poses.push_back(pose_msg);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*kp.body_cloud, cloud_msg);
        cloud_msg.header.frame_id = m_node_config.local_frame;
        cloud_msg.header.stamp = this->now();
        request->keyframe_clouds.push_back(cloud_msg);
        request->keyframe_indices.push_back(static_cast<int32_t>(i));
    }

    std::vector<int32_t> index_copy = request->keyframe_indices;
    m_hba_request_inflight.store(true);
    m_last_hba_trigger_pose_count = total;

    auto future = m_hba_incremental_client->async_send_request(
        request,
        [this, index_copy, request](rclcpp::Client<interface::srv::IncrementalRefine>::SharedFuture future_resp) {
            try {
                auto response = future_resp.get();
                m_hba_request_inflight.store(false);
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "Incremental HBA failed: %s", response->message.c_str());
                    return;
                }

                std::vector<size_t> pose_indices;
                pose_indices.reserve(index_copy.size());
                for (int32_t id : index_copy) {
                    if (id < 0) continue;
                    pose_indices.push_back(static_cast<size_t>(id));
                }

                std::vector<PoseUpdate> optimized;
                optimized.reserve(response->optimized_poses.poses.size());
                for (const auto &pose_msg : response->optimized_poses.poses) {
                    optimized.push_back(poseFromMsg(pose_msg));
                }

                size_t count = std::min(pose_indices.size(), optimized.size());
                pose_indices.resize(count);
                optimized.resize(count);

                if (count == 0) {
                    RCLCPP_WARN(this->get_logger(), "Incremental HBA returned empty pose set");
                    return;
                }

                {
                    std::lock_guard<std::mutex> lock(m_pgo_mutex);
                    m_pgo->applyOptimizedPoses(pose_indices, optimized);
                }

                RCLCPP_INFO(this->get_logger(),
                            "Incremental HBA applied to %zu frames, residual %.6f",
                            count, response->final_residual);
            } catch (const std::exception &e) {
                m_hba_request_inflight.store(false);
                RCLCPP_ERROR(this->get_logger(), "Incremental HBA call failed: %s", e.what());
            }
        });

    (void)future;
}

double PGONode::computeOptimizationScore()
{
    std::vector<SimplePGO::LoopQualityMetrics> metrics_snapshot;
    size_t loop_count = 0;

    {
        std::lock_guard<std::mutex> lock(m_pgo_mutex);
        if (!m_pgo) {
            return 0.0;
        }
        loop_count = m_pgo->historyPairs().size();
        const auto &metrics = m_pgo->recentLoopMetrics();
        metrics_snapshot.assign(metrics.begin(), metrics.end());
    }

    if (metrics_snapshot.empty()) {
        double base = loop_count == 0 ? 0.3 : std::min(0.8, 0.2 + 0.05 * static_cast<double>(loop_count));
        return std::clamp(base, 0.0, 1.0);
    }

    const size_t window = std::min<size_t>(metrics_snapshot.size(), 8);
    double inlier_sum = 0.0;
    double fitness_sum = 0.0;
    double combined_sum = 0.0;
    for (size_t i = metrics_snapshot.size() - window; i < metrics_snapshot.size(); ++i) {
        inlier_sum += metrics_snapshot[i].inlier_ratio;
        fitness_sum += metrics_snapshot[i].fitness;
        combined_sum += metrics_snapshot[i].combined_score;
    }

    const double avg_inlier = inlier_sum / static_cast<double>(window);
    const double avg_fitness = fitness_sum / static_cast<double>(window);
    const double avg_combined = combined_sum / static_cast<double>(window);

    const double loop_factor = std::min(1.0, static_cast<double>(loop_count) / 20.0);
    const double inlier_factor = std::clamp(avg_inlier, 0.0, 1.0);
    const double fitness_factor = std::exp(-avg_fitness / std::max(1e-3, m_pgo_config.loop_score_tresh));
    const double consistency_factor = 1.0 / (1.0 + avg_combined);

    double score = 0.35 * inlier_factor +
                   0.25 * fitness_factor +
                   0.25 * consistency_factor +
                   0.15 * loop_factor;
    return std::clamp(score, 0.0, 1.0);
}

PoseUpdate PGONode::poseFromMsg(const geometry_msgs::msg::Pose &pose_msg) const
{
    PoseUpdate pose;
    pose.t = V3D(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    Eigen::Quaterniond q(pose_msg.orientation.w,
                         pose_msg.orientation.x,
                         pose_msg.orientation.y,
                         pose_msg.orientation.z);
    pose.r = q.normalized().toRotationMatrix();
    return pose;
}

void PGONode::configureLogger()
{
    auto to_lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    };

    const std::string desired = to_lower(m_node_config.log_level);
    int severity = RCUTILS_LOG_SEVERITY_INFO;
    if (desired == "debug") {
        severity = RCUTILS_LOG_SEVERITY_DEBUG;
    } else if (desired == "warn" || desired == "warning") {
        severity = RCUTILS_LOG_SEVERITY_WARN;
    } else if (desired == "error") {
        severity = RCUTILS_LOG_SEVERITY_ERROR;
    } else if (desired == "fatal") {
        severity = RCUTILS_LOG_SEVERITY_FATAL;
    } else if (desired == "info") {
        severity = RCUTILS_LOG_SEVERITY_INFO;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown log level '%s', fallback to INFO", m_node_config.log_level.c_str());
    }

    if (rcutils_logging_set_logger_level(this->get_logger().get_name(), severity) != RCUTILS_RET_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set logger level to %s", m_node_config.log_level.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Logger level set to %s", m_node_config.log_level.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}
