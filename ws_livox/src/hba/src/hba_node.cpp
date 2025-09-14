#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "hba/hba.h"
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/nav_msgs/msg/path.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_array.hpp>

#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"

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
    
    // 自动触发机制配置
    bool auto_trigger_enabled = true;
    int trigger_keyframes_count = 100;
    int trigger_time_interval = 300;  // 秒
    std::string monitor_pgo_topic = "/pgo/optimized_poses";
    bool auto_save_enabled = true;
    int optimization_timeout = 300;  // 秒
    int min_poses_for_optimization = 20;
};

class HBANode : public rclcpp::Node
{
public:
    HBANode() : Node("hba_node"), m_last_optimization_time(std::chrono::steady_clock::now()), m_keyframe_count(0)
    {
        RCLCPP_INFO(this->get_logger(), "HBA node started");
        
        // 先加载参数
        loadParameters();
        
        // 初始化HBA核心
        m_hba = std::make_shared<HBA>(m_hba_config);
        m_voxel_grid.setLeafSize(m_node_config.scan_resolution, m_node_config.scan_resolution, m_node_config.scan_resolution);
        
        // 创建服务
        m_refine_map_srv = this->create_service<interface::srv::RefineMap>(
            "refine_map",
            std::bind(&HBANode::refineMapCB, this, std::placeholders::_1, std::placeholders::_2));
        m_save_poses_srv = this->create_service<interface::srv::SavePoses>(
            "save_poses",
            std::bind(&HBANode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // 创建发布器
        m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        
        // 设置自动触发（如果启用）
        if (m_node_config.auto_trigger_enabled) {
            setupPGOMonitoring();
            RCLCPP_INFO(this->get_logger(), "✅ HBA自动触发已启用 - 阈值: %d关键帧/%d秒", 
                       m_node_config.trigger_keyframes_count, m_node_config.trigger_time_interval);
        } else {
            RCLCPP_INFO(this->get_logger(), "📋 HBA运行在被动服务模式");
        }
        
        // 创建主循环定时器
        m_timer = this->create_wall_timer(100ms, std::bind(&HBANode::mainCB, this));
        
        RCLCPP_INFO(this->get_logger(), "HBA节点初始化完成");
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

        // 基础HBA参数
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
        
        // 自动触发机制参数 (带默认值)
        m_node_config.auto_trigger_enabled = config["auto_trigger_enabled"].as<bool>(true);
        m_node_config.trigger_keyframes_count = config["trigger_keyframes_count"].as<int>(100);
        m_node_config.trigger_time_interval = config["trigger_time_interval"].as<int>(300);
        m_node_config.monitor_pgo_topic = config["monitor_pgo_topic"].as<std::string>("/pgo/optimized_poses");
        m_node_config.auto_save_enabled = config["auto_save_enabled"].as<bool>(true);
        m_node_config.optimization_timeout = config["optimization_timeout"].as<int>(300);
        m_node_config.min_poses_for_optimization = config["min_poses_for_optimization"].as<int>(20);
        
        RCLCPP_INFO(this->get_logger(), "HBA配置加载完成 - 自动触发: %s, 阈值: %d帧/%ds",
                   m_node_config.auto_trigger_enabled ? "启用" : "禁用",
                   m_node_config.trigger_keyframes_count, 
                   m_node_config.trigger_time_interval);
    }

    void refineMapCB(const std::shared_ptr<interface::srv::RefineMap::Request> request, std::shared_ptr<interface::srv::RefineMap::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        if (!std::filesystem::exists(request->maps_path))
        {
            response->success = false;
            response->message = "maps_path not exists";
            return;
        }

        std::filesystem::path p_dir(request->maps_path);
        std::filesystem::path pcd_dir = p_dir / "patches";
        if (!std::filesystem::exists(pcd_dir))
        {
            response->success = false;
            response->message = pcd_dir.string() + " not exists";
            return;
        }

        std::filesystem::path txt_file = p_dir / "poses.txt";

        if (!std::filesystem::exists(txt_file))
        {
            response->success = false;
            response->message = txt_file.string() + " not exists";
            return;
        }

        std::ifstream ifs(txt_file);
        std::string line;
        std::string file_name;
        Pose pose;
        pcl::PCDReader reader;
        while (std::getline(ifs, line))
        {

            fromStr(line, file_name, pose);
            std::filesystem::path pcd_file = p_dir / "patches" / file_name;
            if (!std::filesystem::exists(pcd_file))
            {
                std::cerr << "pcd file not found" << std::endl;
                continue;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            reader.read(pcd_file, *cloud);
            m_voxel_grid.setInputCloud(cloud);
            m_voxel_grid.filter(*cloud);
            m_hba->insert(cloud, pose);
        }
        RCLCPP_INFO(this->get_logger(), "LOAD POSE %lu;", m_hba->poses().size());
        response->success = true;
        response->message = "load poses success!";
        m_do_optimize = true;
        return;
    }

    void savePosesCB(const std::shared_ptr<interface::srv::SavePoses::Request> request, std::shared_ptr<interface::srv::SavePoses::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        std::filesystem::path file_path(request->file_path);
        std::filesystem::path par_path = file_path.parent_path();
        if (!std::filesystem::exists(par_path))
        {
            response->success = false;
            response->message = "parent path not exists";
            return;
        }
        if (m_hba->poses().size() < 1)
        {
            response->success = false;
            response->message = "poses is empty";
            return;
        }

        if (std::filesystem::exists(file_path))
        {
            std::filesystem::remove(file_path);
        }

        m_hba->writePoses(file_path);
        response->success = true;
        response->message = "save poses success!";
        return;
    }
    void mainCB()
    {
        {
            std::lock_guard<std::mutex> lock(m_service_mutex);
            if (!m_do_optimize)
                return;
            RCLCPP_WARN(this->get_logger(), "START OPTIMIZE");
            publishMap();
            for (size_t i = 0; i < m_hba_config.hba_iter; i++)
            {
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu START======", i + 1);
                m_hba->optimize();
                publishMap();
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu END========", i + 1);
            }

            m_do_optimize = false;
            RCLCPP_WARN(this->get_logger(), "END OPTIMIZE");
        }
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

    // 添加PGO监控和自动触发功能
    void setupPGOMonitoring()
    {
        // 监控PGO优化位姿话题
        m_pgo_poses_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            m_node_config.monitor_pgo_topic,
            10,
            std::bind(&HBANode::pgoPosesCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "📡 监控PGO话题: %s", m_node_config.monitor_pgo_topic.c_str());
    }
    
    void pgoPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        
        // 更新关键帧计数
        size_t new_keyframe_count = msg->poses.size();
        
        if (new_keyframe_count > m_keyframe_count) {
            m_keyframe_count = new_keyframe_count;
            RCLCPP_DEBUG(this->get_logger(), "📊 关键帧数量更新: %zu", m_keyframe_count);
        }
        
        // 检查是否需要触发优化
        checkAutoTrigger();
    }
    
    void checkAutoTrigger()
    {
        if (!m_node_config.auto_trigger_enabled || m_do_optimize) {
            return;  // 自动触发已禁用或正在优化中
        }
        
        auto current_time = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - m_last_optimization_time).count();
        
        // 检查关键帧数量阈值
        bool keyframe_threshold_met = (m_keyframe_count >= m_node_config.trigger_keyframes_count);
        
        // 检查时间间隔阈值
        bool time_threshold_met = (time_diff >= m_node_config.trigger_time_interval);
        
        // 检查最少位姿数量要求
        bool min_poses_met = (m_hba->poses().size() >= m_node_config.min_poses_for_optimization);
        
        if ((keyframe_threshold_met || time_threshold_met) && min_poses_met) {
            RCLCPP_INFO(this->get_logger(), "🚀 自动触发HBA优化 - 关键帧: %zu/%d, 时间: %lds/%ds, 位姿: %zu/%d",
                       m_keyframe_count, m_node_config.trigger_keyframes_count,
                       time_diff, m_node_config.trigger_time_interval,
                       m_hba->poses().size(), m_node_config.min_poses_for_optimization);
            
            m_do_optimize = true;
            m_last_optimization_time = current_time;
            m_keyframe_count = 0;  // 重置计数器
            
            // 自动保存优化结果（如果启用）
            if (m_node_config.auto_save_enabled) {
                scheduleAutoSave();
            }
        }
    }
    
    void scheduleAutoSave()
    {
        // 创建自动保存定时器
        auto save_timer = this->create_wall_timer(
            std::chrono::seconds(m_node_config.optimization_timeout + 10),
            [this]() {
                if (!m_do_optimize && m_hba->poses().size() > 0) {
                    std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
                    std::string save_path = "./saved_maps/auto_optimized_" + timestamp + "_poses.txt";
                    
                    try {
                        m_hba->writePoses(save_path);
                        RCLCPP_INFO(this->get_logger(), "💾 自动保存优化结果: %s", save_path.c_str());
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "⚠️  自动保存失败: %s", e.what());
                    }
                }
            }
        );
        
        // 定时器只执行一次
        save_timer->cancel();  // 立即取消，避免重复执行
    }

private:
    NodeConfig m_node_config;
    HBAConfig m_hba_config;
    std::shared_ptr<HBA> m_hba;
    rclcpp::Service<interface::srv::RefineMap>::SharedPtr m_refine_map_srv;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr m_save_poses_srv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_pgo_poses_sub;

    pcl::VoxelGrid<pcl::PointXYZI> m_voxel_grid;
    std::mutex m_service_mutex;
    bool m_do_optimize = false;
    rclcpp::TimerBase::SharedPtr m_timer;
    
    // 自动触发状态变量
    std::chrono::steady_clock::time_point m_last_optimization_time;
    size_t m_keyframe_count;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HBANode>());
    rclcpp::shutdown();
    return 0;
}