#ifndef SLAM_GUI_ROS2_INTERFACE_H
#define SLAM_GUI_ROS2_INTERFACE_H

#include <memory>
#include <string>
#include <mutex>
#include <deque>
#include <QObject>
#include <QTimer>
#include <QVariant>
#include <QMap>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "interface/srv/save_maps.hpp"
#include "interface/srv/save_poses.hpp"
#include "interface/srv/update_pose.hpp"
#include "interface/srv/refine_map.hpp"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"

struct SLAMData
{
    sensor_msgs::msg::PointCloud2::SharedPtr raw_point_cloud;  // 原始雷达数据
    nav_msgs::msg::Odometry::SharedPtr odometry;
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud;     // SLAM处理后的点云(当前帧)
    sensor_msgs::msg::PointCloud2::SharedPtr world_cloud;     // 持久化全局地图
    nav_msgs::msg::Path::SharedPtr path;
    std_msgs::msg::Float64MultiArray::SharedPtr performance_metrics;
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics;
    visualization_msgs::msg::MarkerArray::SharedPtr loop_closures;
    
    // 统计信息
    double last_update_time = 0.0;
    size_t total_points = 0;
    size_t path_length = 0;
    bool is_valid = false;
};

struct SystemStatus
{
    bool fastlio2_active = false;
    bool pgo_active = false;
    bool hba_active = false;
    bool localizer_active = false;
    
    double fastlio2_frequency = 0.0;
    double pgo_frequency = 0.0;
    double point_cloud_frequency = 0.0;
    
    std::string last_error_message;
    rclcpp::Time last_update_time;
};

class ROS2Interface : public QObject
{
    Q_OBJECT

public:
    explicit ROS2Interface(QObject* parent = nullptr);
    ~ROS2Interface();

    bool initialize(const std::string& node_name = "slam_gui_interface");
    void shutdown();
    
    // 数据访问
    SLAMData getCurrentSLAMData() const;
    SystemStatus getSystemStatus() const;
    
    // 服务调用
    bool saveMap(const std::string& file_path, bool save_patches = true, bool enable_hba_refine = false);
    bool savePoses(const std::string& file_path);
    bool updatePose(double timestamp, double x, double y, double z, double qw, double qx, double qy, double qz);

    // HBA服务接口
    bool refineMap(const std::string& input_map_path, const std::string& output_map_path, double refinement_level = 1.0);

    // Localizer服务接口
    bool relocalize(const std::string& map_path, double x, double y, double z, double yaw);
    bool isLocalizationValid();
    
    // 控制接口
    void startDataCollection();
    void stopDataCollection();
    bool isDataCollectionActive() const { return data_collection_active_; }

    // 组件控制功能
    bool startComponent(const std::string& component_name);
    bool stopComponent(const std::string& component_name);
    bool isComponentRunning(const std::string& component_name);
    bool isComponentRunningByTopics(const std::string& component_name);

    // 参数同步功能
    bool setNodeParameter(const std::string& node_name, const std::string& parameter_name, const QVariant& value);
    bool updateFastLIO2Parameters(const QMap<QString, QVariant>& parameters);
    bool updatePGOParameters(const QMap<QString, QVariant>& parameters);
    bool updateHBAParameters(const QMap<QString, QVariant>& parameters);
    bool updateLocalizerParameters(const QMap<QString, QVariant>& parameters);

signals:
    void newSLAMData(const SLAMData& data);
    void systemStatusChanged(const SystemStatus& status);
    void errorOccurred(const QString& error_message);
    void serviceCallCompleted(const QString& service_name, bool success, const QString& message);

private slots:
    void processROSMessages();

private:
    void setupSubscribers();
    void setupServiceClients();
    void setupParameterClients();
    void setupTimers();
    
    // 回调函数
    void rawPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void worldCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void performanceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void loopClosuresCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    void updateSystemStatus();
    void checkNodeHealth();

private:
    // ROS2组件
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<std::thread> spin_thread_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr performance_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr loop_closures_sub_;
    
    // 服务客户端
    rclcpp::Client<interface::srv::SaveMaps>::SharedPtr save_maps_client_;
    rclcpp::Client<interface::srv::SavePoses>::SharedPtr save_poses_client_;
    rclcpp::Client<interface::srv::UpdatePose>::SharedPtr update_pose_client_;
    rclcpp::Client<interface::srv::RefineMap>::SharedPtr refine_map_client_;
    rclcpp::Client<interface::srv::Relocalize>::SharedPtr relocalize_client_;
    rclcpp::Client<interface::srv::IsValid>::SharedPtr is_valid_client_;

    // 参数客户端
    std::shared_ptr<rclcpp::AsyncParametersClient> fastlio2_param_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> pgo_param_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> hba_param_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> localizer_param_client_;
    
    // Qt定时器
    QTimer* ros_timer_;
    QTimer* status_timer_;
    
    // 数据存储
    mutable std::mutex data_mutex_;
    SLAMData current_slam_data_;
    SystemStatus system_status_;
    
    // 状态管理
    bool initialized_ = false;
    bool data_collection_active_ = false;
    
    // 频率计算
    std::deque<rclcpp::Time> odom_timestamps_;
    std::deque<rclcpp::Time> cloud_timestamps_;
    std::deque<rclcpp::Time> pgo_timestamps_;
    
    static constexpr size_t MAX_TIMESTAMP_QUEUE_SIZE = 100;
    static constexpr double FREQUENCY_CALC_WINDOW = 5.0; // 秒
};

#endif // SLAM_GUI_ROS2_INTERFACE_H