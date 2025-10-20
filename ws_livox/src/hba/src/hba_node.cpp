#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "hba/hba.h"
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_set>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32.hpp>

#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"
#include "interface/srv/incremental_refine.hpp"
#include "interface/error_codes.hpp"

using namespace std::chrono_literals;
void fromStr(const std::string &str, std::string &file_name, Pose &pose)
{
    std::stringstream ss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }
    assert(tokens.size() == 8);
    file_name = tokens[0];
    pose.t = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    pose.r = Eigen::Quaterniond(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7])).normalized().toRotationMatrix();
}

struct NodeConfig
{
    double scan_resolution = 0.1;
};

class HBANode : public rclcpp::Node
{
public:
    HBANode() : Node("hba_node")
    {
        RCLCPP_INFO(this->get_logger(), "HBA node started");
        loadParameters();
        m_hba = std::make_shared<HBA>(m_hba_config);
        m_voxel_grid.setLeafSize(m_node_config.scan_resolution, m_node_config.scan_resolution, m_node_config.scan_resolution);
        m_refine_map_srv = this->create_service<interface::srv::RefineMap>(
            "hba/refine_map",
            std::bind(&HBANode::refineMapCB, this, std::placeholders::_1, std::placeholders::_2));
        m_save_poses_srv = this->create_service<interface::srv::SavePoses>(
            "hba/save_poses",
            std::bind(&HBANode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));
        m_incremental_refine_srv = this->create_service<interface::srv::IncrementalRefine>(
            "hba/incremental_refine",
            std::bind(&HBANode::incrementalRefineCB, this, std::placeholders::_1, std::placeholders::_2));
        m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("hba/map_points", 10);
        m_progress_pub = this->create_publisher<std_msgs::msg::Float32>("hba/progress", 10);
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

        m_node_config.scan_resolution = config["scan_resolution"].as<double>();
        m_hba_config.window_size = config["window_size"].as<int>();
        m_hba_config.stride = config["stride"].as<int>();
        m_hba_config.voxel_size = config["voxel_size"].as<double>();
        m_hba_config.min_point_num = config["min_point_num"].as<int>();
        m_hba_config.max_layer = config["max_layer"].as<int>();
        m_hba_config.plane_thresh = config["plane_thresh"].as<double>();
        m_hba_config.ba_max_iter = config["ba_max_iter"].as<size_t>();
        m_hba_config.hba_iter = config["hba_iter"].as<size_t>();
        m_hba_config.down_sample = config["down_sample"].as<double>();
    }

    void refineMapCB(const std::shared_ptr<interface::srv::RefineMap::Request> request,
                     std::shared_ptr<interface::srv::RefineMap::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        try {
            using interface::errors::Code;
            using interface::errors::format;
            if (!std::filesystem::exists(request->maps_path)) {
                response->success = false;
                response->message = format(Code::FILE_NOT_FOUND, "maps_path not exists");
                return;
            }

            m_loaded_keyframes.clear();

            std::filesystem::path p_dir(request->maps_path);
            std::filesystem::path pcd_dir = p_dir / "patches";
            if (!std::filesystem::exists(pcd_dir)) {
                response->success = false;
                response->message = format(Code::FILE_NOT_FOUND, pcd_dir.string() + " not exists");
                return;
            }

            std::filesystem::path txt_file = p_dir / "poses.txt";
            if (!std::filesystem::exists(txt_file)) {
                response->success = false;
                response->message = format(Code::FILE_NOT_FOUND, txt_file.string() + " not exists");
                return;
            }

            // 清理旧数据，重新加载
            m_hba->reset();

            std::ifstream ifs(txt_file);
            std::string line;
            std::string file_name;
            Pose pose;
            pcl::PCDReader reader;

            while (std::getline(ifs, line)) {
                if (line.empty()) {
                    continue;
                }
                fromStr(line, file_name, pose);
                std::filesystem::path pcd_file = p_dir / "patches" / file_name;
                if (!std::filesystem::exists(pcd_file)) {
                    RCLCPP_WARN(this->get_logger(), "pcd file not found: %s", pcd_file.c_str());
                    continue;
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                if (reader.read(pcd_file.string(), *cloud) != 0) {
                    RCLCPP_WARN(this->get_logger(), "failed to read pcd: %s", pcd_file.c_str());
                    continue;
                }
                m_voxel_grid.setInputCloud(cloud);
                m_voxel_grid.filter(*cloud);
                m_hba->insert(cloud, pose);
            }

            if (m_hba->poses().empty()) {
                response->success = false;
                response->message = format(Code::DATA_MISMATCH, "poses.txt loaded no valid entries");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "LOAD POSE %lu;", m_hba->poses().size());

            Vec<Pose> initial_poses = m_hba->poses();
            int iterations_used = 0;
            std::vector<double> translation_errors;
            std::vector<double> rotation_errors;

            auto publish_progress = [&](float value) {
                if (!m_progress_pub || m_progress_pub->get_subscription_count() == 0) {
                    return;
                }
                std_msgs::msg::Float32 msg;
                msg.data = std::clamp(value, 0.0f, 1.0f);
                m_progress_pub->publish(msg);
            };

            publish_progress(0.0f);

            auto start_time = std::chrono::steady_clock::now();
            for (size_t iter = 0; iter < m_hba_config.hba_iter; ++iter) {
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu START======", iter + 1);
                m_hba->optimize();
                iterations_used++;
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu END========", iter + 1);
                publish_progress(static_cast<float>(iter + 1) / std::max<size_t>(1, m_hba_config.hba_iter));
            }
            auto end_time = std::chrono::steady_clock::now();

            double final_residual = computePoseResiduals(initial_poses,
                                                         m_hba->poses(),
                                                         translation_errors,
                                                         rotation_errors);

            geometry_msgs::msg::PoseArray optimized_array = buildPoseArray(m_hba->poses());
            response->optimized_poses = optimized_array;
            response->pose_covariances = buildPoseCovariances(translation_errors, rotation_errors);
            response->iterations_used = iterations_used;
            response->final_residual = final_residual;
            response->success = true;
            response->message = format(Code::SUCCESS, "HBA optimization completed");

            RCLCPP_INFO(this->get_logger(),
                        "HBA optimization finished in %.2fs, residual: %.6f",
                        std::chrono::duration<double>(end_time - start_time).count(),
                        final_residual);

            publishMap();
            publish_progress(1.0f);
        } catch (const std::exception& e) {
            response->success = false;
            auto formatted = interface::errors::format(interface::errors::Code::OPTIMIZATION_FAILED,
                                                       e.what());
            response->message = formatted;
            RCLCPP_ERROR(this->get_logger(), "%s", formatted.c_str());
        }
    }

    void incrementalRefineCB(const std::shared_ptr<interface::srv::IncrementalRefine::Request> request,
                             std::shared_ptr<interface::srv::IncrementalRefine::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        try {
            using interface::errors::Code;
            using interface::errors::format;
            const size_t pose_count = request->keyframe_poses.poses.size();
            if (pose_count == 0) {
                response->success = false;
                response->message = format(Code::INVALID_CONFIGURATION, "request contains no keyframes");
                return;
            }

            auto publish_progress = [&](float value) {
                if (!m_progress_pub || m_progress_pub->get_subscription_count() == 0) {
                    return;
                }
                std_msgs::msg::Float32 msg;
                msg.data = std::clamp(value, 0.0f, 1.0f);
                m_progress_pub->publish(msg);
            };

            publish_progress(0.0f);

            if (pose_count != request->keyframe_clouds.size() ||
                pose_count != request->keyframe_indices.size()) {
                response->success = false;
                response->message = format(Code::DATA_MISMATCH, "poses, clouds and indices size mismatch");
                return;
            }

            if (request->reset) {
                m_hba->reset();
                m_loaded_keyframes.clear();
            }

            size_t inserted_frames = 0;
            for (size_t idx = 0; idx < pose_count; ++idx) {
                int32_t keyframe_id = request->keyframe_indices[idx];
                if (!request->reset && m_loaded_keyframes.count(keyframe_id) > 0) {
                    continue;
                }

                Pose pose = fromGeometryPose(request->keyframe_poses.poses[idx]);
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(request->keyframe_clouds[idx], *cloud);
                m_voxel_grid.setInputCloud(cloud);
                m_voxel_grid.filter(*cloud);
                m_hba->insert(cloud, pose);
                m_loaded_keyframes.insert(keyframe_id);
                inserted_frames++;
            }

            if (m_hba->poses().empty()) {
                response->success = false;
                response->message = format(Code::DATA_MISMATCH, "HBA holds no keyframes");
                return;
            }

            if (inserted_frames == 0 && !request->run_optimization) {
                response->optimized_poses = buildPoseArray(m_hba->poses());
                response->pose_covariances.clear();
                response->iterations_used = 0;
                response->final_residual = 0.0;
                response->success = true;
                response->message = format(Code::SUCCESS, "no new keyframes provided");
                return;
            }

            Vec<Pose> initial_poses = m_hba->poses();
            int iterations_target = request->max_iterations > 0 ? request->max_iterations : static_cast<int>(m_hba_config.hba_iter);
            iterations_target = std::max(iterations_target, 0);
            int iterations_used = 0;
            std::vector<double> translation_errors;
            std::vector<double> rotation_errors;
            double final_residual = 0.0;

            if (request->run_optimization && iterations_target > 0) {
                for (int iter = 0; iter < iterations_target; ++iter) {
                    m_hba->optimize();
                    iterations_used++;
                    publish_progress(static_cast<float>(iter + 1) / std::max(1, iterations_target));
                }
                final_residual = computePoseResiduals(initial_poses, m_hba->poses(),
                                                      translation_errors, rotation_errors);
            } else {
                translation_errors.assign(m_hba->poses().size(), 0.0);
                rotation_errors.assign(m_hba->poses().size(), 0.0);
            }

            response->optimized_poses = buildPoseArray(m_hba->poses());
            response->pose_covariances = buildPoseCovariances(translation_errors, rotation_errors);
            response->iterations_used = iterations_used;
            response->final_residual = final_residual;
            response->success = true;
            response->message = request->run_optimization ?
                format(Code::SUCCESS, "HBA incremental refinement completed") :
                format(Code::SUCCESS, "HBA keyframes updated");

            if (request->run_optimization && iterations_used > 0) {
                publishMap();
            }

            publish_progress(1.0f);
        } catch (const std::exception &e) {
            response->success = false;
            auto formatted = interface::errors::format(interface::errors::Code::OPTIMIZATION_FAILED,
                                                       e.what());
            response->message = formatted;
            RCLCPP_ERROR(this->get_logger(), "%s", formatted.c_str());
        }
    }

    void savePosesCB(const std::shared_ptr<interface::srv::SavePoses::Request> request, std::shared_ptr<interface::srv::SavePoses::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        std::filesystem::path file_path(request->file_path);
        std::filesystem::path par_path = file_path.parent_path();
        if (!std::filesystem::exists(par_path))
        {
            response->success = false;
            response->message = interface::errors::format(interface::errors::Code::FILE_NOT_FOUND,
                                                          "parent path not exists");
            return;
        }
        if (m_hba->poses().size() < 1)
        {
            response->success = false;
            response->message = interface::errors::format(interface::errors::Code::DATA_MISMATCH,
                                                          "poses is empty");
            return;
        }

        if (std::filesystem::exists(file_path))
        {
            std::filesystem::remove(file_path);
        }

        m_hba->writePoses(file_path);
        response->success = true;
        response->message = interface::errors::format(interface::errors::Code::SUCCESS, "save poses success");
        return;
    }

    geometry_msgs::msg::PoseArray buildPoseArray(const Vec<Pose> &poses)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        pose_array.header.stamp = this->now();
        pose_array.poses.reserve(poses.size());
        for (const auto &pose : poses) {
            pose_array.poses.push_back(toGeometryPose(pose));
        }
        return pose_array;
    }

    geometry_msgs::msg::Pose toGeometryPose(const Pose &pose) const
    {
        geometry_msgs::msg::Pose ros_pose;
        ros_pose.position.x = pose.t.x();
        ros_pose.position.y = pose.t.y();
        ros_pose.position.z = pose.t.z();

        Eigen::Quaterniond q(pose.r);
        ros_pose.orientation.w = q.w();
        ros_pose.orientation.x = q.x();
        ros_pose.orientation.y = q.y();
        ros_pose.orientation.z = q.z();
        return ros_pose;
    }

    Pose fromGeometryPose(const geometry_msgs::msg::Pose &ros_pose) const
    {
        Pose pose;
        pose.t = V3D(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
        Eigen::Quaterniond q(ros_pose.orientation.w,
                             ros_pose.orientation.x,
                             ros_pose.orientation.y,
                             ros_pose.orientation.z);
        pose.r = q.normalized().toRotationMatrix();
        return pose;
    }

    std::vector<double> buildPoseCovariances(const std::vector<double> &translation_errors,
                                             const std::vector<double> &rotation_errors) const
    {
        size_t n = std::min(translation_errors.size(), rotation_errors.size());
        std::vector<double> covariances;
        covariances.reserve(n * 36);

        for (size_t i = 0; i < n; ++i) {
            double trans_var = std::max(translation_errors[i] * translation_errors[i], 1e-8);
            double rot_var = std::max(rotation_errors[i] * rotation_errors[i], 1e-8);

            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    double value = 0.0;
                    if (row == col) {
                        value = (row < 3) ? trans_var : rot_var;
                    }
                    covariances.push_back(value);
                }
            }
        }
        return covariances;
    }

    double computePoseResiduals(const Vec<Pose> &baseline,
                                const Vec<Pose> &optimized,
                                std::vector<double> &translation_errors,
                                std::vector<double> &rotation_errors) const
    {
        size_t n = std::min(baseline.size(), optimized.size());
        translation_errors.assign(n, 0.0);
        rotation_errors.assign(n, 0.0);

        if (n == 0) {
            return 0.0;
        }

        double accum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            V3D delta_t = optimized[i].t - baseline[i].t;
            double trans_err = delta_t.norm();

            Eigen::Quaterniond q_base(baseline[i].r);
            Eigen::Quaterniond q_opt(optimized[i].r);
            double rot_err = q_base.angularDistance(q_opt);

            translation_errors[i] = trans_err;
            rotation_errors[i] = rot_err;

            accum += trans_err * trans_err + rot_err * rot_err;
        }

        return std::sqrt(accum / static_cast<double>(n));
    }

    void publishMap()
    {
        if (m_cloud_pub->get_subscription_count() < 1)
            return;
        if (m_hba->poses().size() < 1)
            return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_hba->getMapPoints();
        m_voxel_grid.setInputCloud(cloud);
        m_voxel_grid.filter(*cloud);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        m_cloud_pub->publish(cloud_msg);
    }

private:
    NodeConfig m_node_config;
    HBAConfig m_hba_config;
    std::shared_ptr<HBA> m_hba;
    rclcpp::Service<interface::srv::RefineMap>::SharedPtr m_refine_map_srv;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr m_save_poses_srv;
    rclcpp::Service<interface::srv::IncrementalRefine>::SharedPtr m_incremental_refine_srv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_progress_pub;

    pcl::VoxelGrid<pcl::PointXYZI> m_voxel_grid;
    std::mutex m_service_mutex;
    std::unordered_set<int32_t> m_loaded_keyframes;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HBANode>());
    rclcpp::shutdown();
    return 0;
}
