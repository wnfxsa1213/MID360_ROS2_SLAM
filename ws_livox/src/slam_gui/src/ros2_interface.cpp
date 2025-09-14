#include "slam_gui/ros2_interface.h"
#include <thread>
#include <chrono>
#include <algorithm>

ROS2Interface::ROS2Interface(QObject* parent)
    : QObject(parent)
{
    // 初始化Qt定时器
    ros_timer_ = new QTimer(this);
    status_timer_ = new QTimer(this);
    
    connect(ros_timer_, &QTimer::timeout, this, &ROS2Interface::processROSMessages);
    connect(status_timer_, &QTimer::timeout, this, &ROS2Interface::updateSystemStatus);
}

ROS2Interface::~ROS2Interface()
{
    shutdown();
}

bool ROS2Interface::initialize(const std::string& node_name)
{
    try {
        // 初始化ROS2
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // 创建节点
        node_ = rclcpp::Node::make_shared(node_name);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        // 设置订阅者和服务客户端
        setupSubscribers();
        setupServiceClients();
        setupTimers();
        
        // 启动ROS2处理线程
        spin_thread_ = std::make_unique<std::thread>([this]() {
            executor_->spin();
        });
        
        initialized_ = true;
        
        RCLCPP_INFO(node_->get_logger(), "SLAM GUI ROS2接口初始化成功");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("slam_gui"), "ROS2接口初始化失败: %s", e.what());
        emit errorOccurred(QString("ROS2接口初始化失败: %1").arg(e.what()));
        return false;
    }
}

void ROS2Interface::shutdown()
{
    if (!initialized_) return;
    
    // 停止定时器
    if (ros_timer_) ros_timer_->stop();
    if (status_timer_) status_timer_->stop();
    
    data_collection_active_ = false;
    
    // 停止ROS2处理
    if (executor_) {
        executor_->cancel();
    }
    
    if (spin_thread_ && spin_thread_->joinable()) {
        spin_thread_->join();
    }
    
    node_.reset();
    executor_.reset();
    
    initialized_ = false;
    
    RCLCPP_INFO(rclcpp::get_logger("slam_gui"), "SLAM GUI ROS2接口已关闭");
}

void ROS2Interface::setupSubscribers()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 订阅原始雷达数据
    raw_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", qos,
        std::bind(&ROS2Interface::rawPointCloudCallback, this, std::placeholders::_1));

    // 订阅SLAM核心数据
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastlio2/lio_odom", qos,
        std::bind(&ROS2Interface::odometryCallback, this, std::placeholders::_1));

    // 订阅实时点云 (当前帧)
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/fastlio2/body_cloud", qos,
        std::bind(&ROS2Interface::pointCloudCallback, this, std::placeholders::_1));

    // 订阅持久化全局地图 (优化后的累积地图)
    world_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/fastlio2/world_cloud", qos,
        std::bind(&ROS2Interface::worldCloudCallback, this, std::placeholders::_1));

    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/fastlio2/lio_path", qos,
        std::bind(&ROS2Interface::pathCallback, this, std::placeholders::_1));
    
    // 订阅性能和诊断数据
    performance_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/fastlio2/performance_metrics", qos,
        std::bind(&ROS2Interface::performanceCallback, this, std::placeholders::_1));
    
    diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/fastlio2/diagnostics", qos,
        std::bind(&ROS2Interface::diagnosticsCallback, this, std::placeholders::_1));
    
    // 订阅PGO回环数据
    loop_closures_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/pgo/loop_closures", qos,
        std::bind(&ROS2Interface::loopClosuresCallback, this, std::placeholders::_1));
}

void ROS2Interface::setupServiceClients()
{
    // 创建服务客户端
    save_maps_client_ = node_->create_client<interface::srv::SaveMaps>("/fastlio2/save_maps");
    save_poses_client_ = node_->create_client<interface::srv::SavePoses>("/fastlio2/save_poses");
    update_pose_client_ = node_->create_client<interface::srv::UpdatePose>("/fastlio2/update_pose");

    // HBA服务客户端
    refine_map_client_ = node_->create_client<interface::srv::RefineMap>("/hba/refine_map");

    // Localizer服务客户端
    relocalize_client_ = node_->create_client<interface::srv::Relocalize>("/localizer/relocalize");
    is_valid_client_ = node_->create_client<interface::srv::IsValid>("/localizer/is_valid");

    // 创建参数客户端
    setupParameterClients();
}

void ROS2Interface::setupParameterClients()
{
    // 为各个SLAM组件创建参数客户端
    fastlio2_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "lio_node");
    pgo_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "pgo_node");
    hba_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "hba_node");
    localizer_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "localizer_node");
}

void ROS2Interface::setupTimers()
{
    // 启动定时器
    ros_timer_->start(50); // 20Hz处理ROS消息
    status_timer_->start(1000); // 1Hz更新系统状态
}

void ROS2Interface::startDataCollection()
{
    data_collection_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "开始SLAM数据收集");
}

void ROS2Interface::stopDataCollection()
{
    data_collection_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "停止SLAM数据收集");
}

SLAMData ROS2Interface::getCurrentSLAMData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_slam_data_;
}

SystemStatus ROS2Interface::getSystemStatus() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return system_status_;
}

void ROS2Interface::processROSMessages()
{
    if (!initialized_ || !data_collection_active_) return;
    
    // 这个函数由Qt定时器调用，ROS消息在spin线程中处理
    // 这里主要是触发数据更新信号
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_slam_data_.is_valid) {
            emit newSLAMData(current_slam_data_);
        }
    }
}

void ROS2Interface::rawPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_slam_data_.raw_point_cloud = msg;
    current_slam_data_.last_update_time = node_->get_clock()->now().seconds();

    // 如果没有SLAM数据，原始点云也算是有效数据
    if (!current_slam_data_.point_cloud) {
        current_slam_data_.total_points = msg->width * msg->height;
        current_slam_data_.is_valid = true;
    }
}

void ROS2Interface::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_slam_data_.odometry = msg;
    current_slam_data_.last_update_time = node_->get_clock()->now().seconds();
    current_slam_data_.is_valid = true;
    
    // 更新频率计算
    auto now = node_->get_clock()->now();
    odom_timestamps_.push_back(now);
    if (odom_timestamps_.size() > MAX_TIMESTAMP_QUEUE_SIZE) {
        odom_timestamps_.pop_front();
    }
}

void ROS2Interface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_slam_data_.point_cloud = msg;
    current_slam_data_.total_points = msg->width * msg->height;

    // 更新频率计算
    auto now = node_->get_clock()->now();
    cloud_timestamps_.push_back(now);
    if (cloud_timestamps_.size() > MAX_TIMESTAMP_QUEUE_SIZE) {
        cloud_timestamps_.pop_front();
    }
}

void ROS2Interface::worldCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_slam_data_.world_cloud = msg;
    current_slam_data_.last_update_time = node_->get_clock()->now().seconds();
    current_slam_data_.is_valid = true;

    // 更新全局地图统计信息
    if (msg) {
        size_t world_points = msg->width * msg->height;
        RCLCPP_DEBUG(node_->get_logger(), "接收到全局地图，点数: %zu", world_points);
    }
}

void ROS2Interface::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_slam_data_.path = msg;
    current_slam_data_.path_length = msg->poses.size();
}

void ROS2Interface::performanceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_slam_data_.performance_metrics = msg;
}

void ROS2Interface::diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_slam_data_.diagnostics = msg;
    
    // 更新系统状态
    for (const auto& status : msg->status) {
        if (status.name == "fastlio2") {
            system_status_.fastlio2_active = (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK);
        }
        // 可以添加更多状态检查
    }
}

void ROS2Interface::loopClosuresCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_slam_data_.loop_closures = msg;
}

void ROS2Interface::updateSystemStatus()
{
    if (!initialized_) return;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto now = node_->get_clock()->now();
    
    // 计算频率
    auto calculateFrequency = [&](const std::deque<rclcpp::Time>& timestamps) -> double {
        if (timestamps.size() < 2) return 0.0;
        
        auto cutoff_time = now - rclcpp::Duration::from_seconds(FREQUENCY_CALC_WINDOW);
        auto valid_stamps = std::count_if(timestamps.begin(), timestamps.end(),
            [&cutoff_time](const rclcpp::Time& t) { return t > cutoff_time; });
        
        return static_cast<double>(valid_stamps) / FREQUENCY_CALC_WINDOW;
    };
    
    system_status_.fastlio2_frequency = calculateFrequency(odom_timestamps_);
    system_status_.point_cloud_frequency = calculateFrequency(cloud_timestamps_);
    system_status_.pgo_frequency = calculateFrequency(pgo_timestamps_);
    
    system_status_.last_update_time = now;
    
    emit systemStatusChanged(system_status_);
}

void ROS2Interface::checkNodeHealth()
{
    // 实现节点健康检查逻辑
    // 可以检查话题发布状态、服务可用性等
}

bool ROS2Interface::saveMap(const std::string& file_path, bool save_patches, bool enable_hba_refine)
{
    if (!save_maps_client_->wait_for_service(std::chrono::seconds(2))) {
        emit errorOccurred("地图保存服务不可用");
        return false;
    }
    
    auto request = std::make_shared<interface::srv::SaveMaps::Request>();
    request->file_path = file_path;
    request->save_patches = save_patches;
    request->enable_hba_refine = enable_hba_refine;
    
    auto future = save_maps_client_->async_send_request(request);
    
    // 等待响应
    if (future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
        auto response = future.get();
        emit serviceCallCompleted("SaveMaps", response->success, QString::fromStdString(response->message));
        return response->success;
    } else {
        emit errorOccurred("地图保存服务超时");
        return false;
    }
}

bool ROS2Interface::savePoses(const std::string& file_path)
{
    if (!save_poses_client_->wait_for_service(std::chrono::seconds(2))) {
        emit errorOccurred("位姿保存服务不可用");
        return false;
    }
    
    auto request = std::make_shared<interface::srv::SavePoses::Request>();
    request->file_path = file_path;
    
    auto future = save_poses_client_->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        auto response = future.get();
        emit serviceCallCompleted("SavePoses", response->success, QString::fromStdString(response->message));
        return response->success;
    } else {
        emit errorOccurred("位姿保存服务超时");
        return false;
    }
}

bool ROS2Interface::updatePose(double timestamp, double x, double y, double z, 
                              double qw, double qx, double qy, double qz)
{
    if (!update_pose_client_->wait_for_service(std::chrono::seconds(1))) {
        emit errorOccurred("位姿更新服务不可用");
        return false;
    }
    
    auto request = std::make_shared<interface::srv::UpdatePose::Request>();
    request->timestamp = timestamp;
    request->x = x;
    request->y = y;
    request->z = z;
    request->qw = qw;
    request->qx = qx;
    request->qy = qy;
    request->qz = qz;
    
    auto future = update_pose_client_->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = future.get();
        emit serviceCallCompleted("UpdatePose", response->success, QString::fromStdString(response->message));
        return response->success;
    } else {
        emit errorOccurred("位姿更新服务超时");
        return false;
    }
}

// 组件控制功能实现
bool ROS2Interface::startComponent(const std::string& component_name)
{
    // 组件启动实际在MainWindow中通过QProcess实现
    // 这里只验证组件名称的有效性
    if (component_name == "fastlio2" || component_name == "pgo" ||
        component_name == "hba" || component_name == "localizer") {
        emit serviceCallCompleted(QString::fromStdString("Start" + component_name), true,
                                QString::fromStdString("启动" + component_name + "组件命令已准备"));
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"),
                    "Unknown component: %s", component_name.c_str());
        emit errorOccurred(QString::fromStdString("未知组件: " + component_name));
        return false;
    }
}

bool ROS2Interface::stopComponent(const std::string& component_name)
{
    // 组件停止实际在MainWindow中通过QProcess实现
    // 这里只验证组件名称的有效性
    if (component_name == "fastlio2" || component_name == "pgo" ||
        component_name == "hba" || component_name == "localizer") {
        emit serviceCallCompleted(QString::fromStdString("Stop" + component_name), true,
                                QString::fromStdString("停止" + component_name + "组件命令已准备"));
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"),
                    "Unknown component: %s", component_name.c_str());
        emit errorOccurred(QString::fromStdString("未知组件: " + component_name));
        return false;
    }
}

bool ROS2Interface::isComponentRunning(const std::string& component_name)
{
    if (!node_) {
        return false;
    }

    try {
        // 首先使用话题检测方法（更可靠）
        if (isComponentRunningByTopics(component_name)) {
            return true;
        }
        
        // 备用方案：节点名称检测（带重试机制）
        for (int retry = 0; retry < 3; ++retry) {
            auto node_names = node_->get_node_names();
            
            std::string target_node_name;
            if (component_name == "fastlio2") {
                target_node_name = "lio_node";
            } else if (component_name == "pgo") {
                target_node_name = "pgo_node";
            } else if (component_name == "hba") {
                target_node_name = "hba_node";
            } else if (component_name == "localizer") {
                target_node_name = "localizer_node";
            } else {
                return false;
            }

            // 检查目标节点是否在活动节点列表中
            for (const auto& node_name : node_names) {
                if (node_name.find(target_node_name) != std::string::npos) {
                    return true;
                }
            }
            
            // 短暂等待后重试
            if (retry < 2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        return false;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"),
                    "Error checking component status %s: %s", component_name.c_str(), e.what());
        return false;
    }
}

bool ROS2Interface::isComponentRunningByTopics(const std::string& component_name)
{
    if (!node_) {
        return false;
    }
    
    try {
        // 根据组件检查特定话题是否活跃
        auto topic_names_and_types = node_->get_topic_names_and_types();
        
        std::vector<std::string> expected_topics;
        if (component_name == "fastlio2") {
            expected_topics = {"/fastlio2/lio_odom", "/fastlio2/body_cloud"};
        } else if (component_name == "pgo") {
            expected_topics = {"/pgo/loop_closures", "/pgo/optimized_poses"};
        } else if (component_name == "hba") {
            expected_topics = {"/hba/refined_map"};
        } else if (component_name == "localizer") {
            expected_topics = {"/localizer/relocalized_pose"};
        } else {
            return false;
        }
        
        // 检查至少一个预期话题是否存在
        for (const auto& expected_topic : expected_topics) {
            for (const auto& [topic_name, topic_types] : topic_names_and_types) {
                if (topic_name == expected_topic) {
                    return true;
                }
            }
        }
        
        return false;
        
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(rclcpp::get_logger("ROS2Interface"),
                    "Topic detection failed for %s: %s", component_name.c_str(), e.what());
        return false;
    }
}

// 参数同步功能实现
bool ROS2Interface::setNodeParameter(const std::string& node_name, const std::string& parameter_name, const QVariant& value)
{
    try {
        std::shared_ptr<rclcpp::AsyncParametersClient> param_client;

        // 选择合适的参数客户端
        if (node_name == "lio_node") {
            param_client = fastlio2_param_client_;
        } else if (node_name == "pgo_node") {
            param_client = pgo_param_client_;
        } else if (node_name == "hba_node") {
            param_client = hba_param_client_;
        } else if (node_name == "localizer_node") {
            param_client = localizer_param_client_;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Unknown node name: %s", node_name.c_str());
            return false;
        }

        if (!param_client) {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Parameter client for %s is not initialized", node_name.c_str());
            return false;
        }

        // 等待服务可用
        if (!param_client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Parameter service for %s is not available", node_name.c_str());
            return false;
        }

        // 转换QVariant到ROS2参数值
        rclcpp::Parameter param;
        if (value.type() == QVariant::Double) {
            param = rclcpp::Parameter(parameter_name, value.toDouble());
        } else if (value.type() == QVariant::Int) {
            param = rclcpp::Parameter(parameter_name, value.toInt());
        } else if (value.type() == QVariant::Bool) {
            param = rclcpp::Parameter(parameter_name, value.toBool());
        } else if (value.type() == QVariant::String) {
            param = rclcpp::Parameter(parameter_name, value.toString().toStdString());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Unsupported parameter type for %s", parameter_name.c_str());
            return false;
        }

        // 设置参数
        auto future = param_client->set_parameters({param});

        // 等待结果 (非阻塞方式)
        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            auto results = future.get();
            if (!results.empty() && results[0].successful) {
                RCLCPP_INFO(rclcpp::get_logger("ROS2Interface"),
                           "Successfully set parameter %s.%s", node_name.c_str(), parameter_name.c_str());
                return true;
            }
        }

        RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"),
                   "Failed to set parameter %s.%s", node_name.c_str(), parameter_name.c_str());
        return false;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"),
                    "Exception in setNodeParameter: %s", e.what());
        return false;
    }
}

bool ROS2Interface::updateFastLIO2Parameters(const QMap<QString, QVariant>& parameters)
{
    bool all_success = true;
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
        if (!setNodeParameter("lio_node", it.key().toStdString(), it.value())) {
            all_success = false;
        }
    }
    return all_success;
}

bool ROS2Interface::updatePGOParameters(const QMap<QString, QVariant>& parameters)
{
    bool all_success = true;
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
        if (!setNodeParameter("pgo_node", it.key().toStdString(), it.value())) {
            all_success = false;
        }
    }
    return all_success;
}

bool ROS2Interface::updateHBAParameters(const QMap<QString, QVariant>& parameters)
{
    bool all_success = true;
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
        if (!setNodeParameter("hba_node", it.key().toStdString(), it.value())) {
            all_success = false;
        }
    }
    return all_success;
}

bool ROS2Interface::updateLocalizerParameters(const QMap<QString, QVariant>& parameters)
{
    bool all_success = true;
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
        if (!setNodeParameter("localizer_node", it.key().toStdString(), it.value())) {
            all_success = false;
        }
    }
    return all_success;
}

// HBA服务接口实现
bool ROS2Interface::refineMap(const std::string& input_map_path, const std::string& output_map_path, double refinement_level)
{
    if (!refine_map_client_) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "RefineMap client not initialized");
        return false;
    }

    if (!refine_map_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "RefineMap service not available");
        return false;
    }

    auto request = std::make_shared<interface::srv::RefineMap::Request>();
    request->maps_path = input_map_path;  // RefineMap.srv只有maps_path字段

    try {
        auto future = refine_map_client_->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(rclcpp::get_logger("ROS2Interface"),
                           "Map refinement completed: %s", response->message.c_str());
                emit serviceCallCompleted("RefineMap", true, QString::fromStdString(response->message));
                return true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"),
                           "Map refinement failed: %s", response->message.c_str());
                emit serviceCallCompleted("RefineMap", false, QString::fromStdString(response->message));
                return false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "RefineMap service call timeout");
            emit serviceCallCompleted("RefineMap", false, "Service call timeout");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "RefineMap service call exception: %s", e.what());
        emit serviceCallCompleted("RefineMap", false, QString("Exception: %1").arg(e.what()));
        return false;
    }
}

// Localizer服务接口实现
bool ROS2Interface::relocalize(const std::string& map_path, double x, double y, double z, double yaw)
{
    if (!relocalize_client_) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "Relocalize client not initialized");
        return false;
    }

    if (!relocalize_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Relocalize service not available");
        return false;
    }

    auto request = std::make_shared<interface::srv::Relocalize::Request>();
    request->pcd_path = map_path;  // Relocalize.srv使用pcd_path
    request->x = x;
    request->y = y;
    request->z = z;
    request->yaw = yaw;
    request->pitch = 0.0;  // 设置默认值
    request->roll = 0.0;   // 设置默认值

    try {
        auto future = relocalize_client_->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(rclcpp::get_logger("ROS2Interface"),
                           "Relocalization successful: %s", response->message.c_str());
                emit serviceCallCompleted("Relocalize", true, QString::fromStdString(response->message));
                return true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"),
                           "Relocalization failed: %s", response->message.c_str());
                emit serviceCallCompleted("Relocalize", false, QString::fromStdString(response->message));
                return false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2Interface"), "Relocalize service call timeout");
            emit serviceCallCompleted("Relocalize", false, "Service call timeout");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "Relocalize service call exception: %s", e.what());
        emit serviceCallCompleted("Relocalize", false, QString("Exception: %1").arg(e.what()));
        return false;
    }
}

bool ROS2Interface::isLocalizationValid()
{
    if (!is_valid_client_) {
        RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "IsValid client not initialized");
        return false;
    }

    if (!is_valid_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_DEBUG(rclcpp::get_logger("ROS2Interface"), "IsValid service not available");
        return false;
    }

    auto request = std::make_shared<interface::srv::IsValid::Request>();

    try {
        auto future = is_valid_client_->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            return response->valid;
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("ROS2Interface"), "IsValid service call timeout");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(rclcpp::get_logger("ROS2Interface"), "IsValid service call exception: %s", e.what());
        return false;
    }
}

// MOC文件由CMake自动处理