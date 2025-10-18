#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "interface/srv/update_pose.hpp"
#include "interface/srv/sync_state.hpp"
#include "interface/srv/trigger_optimization.hpp"
#include "interface/srv/get_optimized_pose.hpp"
#include "interface/srv/refine_map.hpp"
#include "interface/srv/relocalize.hpp"
#include <algorithm>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

struct OptimizationTask {
    enum TaskType { PGO_LOOP_CLOSURE, HBA_REFINEMENT, LOCALIZER_GLOBAL, FEEDBACK_UPDATE };
    TaskType type;
    int priority;
    rclcpp::Time created_time;
    std::string component_name;
    bool emergency = false;
};

struct TaskComparator {
    bool operator()(const OptimizationTask& lhs, const OptimizationTask& rhs) const {
        if (lhs.emergency != rhs.emergency) {
            return rhs.emergency; // emergency == true takes precedence
        }
        if (lhs.priority != rhs.priority) {
            return lhs.priority < rhs.priority; // higher priority first
        }
        return lhs.created_time > rhs.created_time; // FIFO for equal priority
    }
};

struct SystemMetrics {
    double cumulative_drift = 0.0;
    double last_optimization_score = 0.0;
    int successful_optimizations = 0;
    int failed_optimizations = 0;
    rclcpp::Time last_optimization_time;
    bool optimization_in_progress = false;
};

class OptimizationCoordinator : public rclcpp::Node
{
public:
    OptimizationCoordinator() : Node("optimization_coordinator")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 优化协调器启动 - 真正协同SLAM系统");

        loadConfiguration();
        initializeClients();
        initializeServices();
        initializeSubscriptions();

        // 初始化系统指标时间戳
        m_metrics.last_optimization_time = this->get_clock()->now();

        // 系统监控定时器 - 每2秒检查一次
        m_monitor_timer = this->create_wall_timer(
            2000ms, std::bind(&OptimizationCoordinator::systemMonitor, this));

        // 任务处理定时器 - 每500ms处理一次队列
        m_task_timer = this->create_wall_timer(
            500ms, std::bind(&OptimizationCoordinator::processTaskQueue, this));

        // 性能统计定时器 - 每10秒发布一次
        m_metrics_timer = this->create_wall_timer(
            10000ms, std::bind(&OptimizationCoordinator::publishMetrics, this));
    }

private:
    // 配置参数
    struct Config {
        double drift_threshold = 0.5;
        double time_threshold = 60.0;
        double emergency_threshold = 2.0;
        int max_concurrent_tasks = 3;
        bool auto_optimization = true;
    } m_config;

    // 客户端和服务
    rclcpp::Client<interface::srv::RefineMap>::SharedPtr m_hba_client;
    rclcpp::Client<interface::srv::Relocalize>::SharedPtr m_localizer_client;
    rclcpp::Client<interface::srv::UpdatePose>::SharedPtr m_fastlio_client;
    rclcpp::Client<interface::srv::SyncState>::SharedPtr m_sync_client;
    rclcpp::Client<interface::srv::GetOptimizedPose>::SharedPtr m_pgo_status_client;

    rclcpp::Service<interface::srv::TriggerOptimization>::SharedPtr m_trigger_service;

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_metrics_sub;

    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_coordination_pub;

    // 定时器
    rclcpp::TimerBase::SharedPtr m_monitor_timer;
    rclcpp::TimerBase::SharedPtr m_task_timer;
    rclcpp::TimerBase::SharedPtr m_metrics_timer;

    // 系统状态
    SystemMetrics m_metrics;
    std::priority_queue<OptimizationTask, std::vector<OptimizationTask>, TaskComparator> m_task_queue;
    std::mutex m_queue_mutex;
    std::mutex m_metrics_mutex;

    geometry_msgs::msg::Pose m_last_pose;
    geometry_msgs::msg::Pose m_reference_pose;

    void loadConfiguration()
    {
        this->declare_parameter("drift_threshold", 0.5);
        this->declare_parameter("time_threshold", 60.0);
        this->declare_parameter("emergency_threshold", 2.0);
        this->declare_parameter("auto_optimization", true);

        m_config.drift_threshold = this->get_parameter("drift_threshold").as_double();
        m_config.time_threshold = this->get_parameter("time_threshold").as_double();
        m_config.emergency_threshold = this->get_parameter("emergency_threshold").as_double();
        m_config.auto_optimization = this->get_parameter("auto_optimization").as_bool();

        RCLCPP_INFO(this->get_logger(), "配置加载: 漂移阈值=%.2fm, 时间阈值=%.1fs",
                   m_config.drift_threshold, m_config.time_threshold);
    }

    void initializeClients()
    {
        m_hba_client = this->create_client<interface::srv::RefineMap>("hba/refine_map");
        m_localizer_client = this->create_client<interface::srv::Relocalize>("localizer/relocalize");
        m_fastlio_client = this->create_client<interface::srv::UpdatePose>("fastlio2/update_pose");
        m_sync_client = this->create_client<interface::srv::SyncState>("fastlio2/sync_state");
        m_pgo_status_client = this->create_client<interface::srv::GetOptimizedPose>("pgo/get_optimized_pose");

        RCLCPP_INFO(this->get_logger(), "✅ 协同客户端初始化完成");
    }

    void initializeServices()
    {
        m_trigger_service = this->create_service<interface::srv::TriggerOptimization>(
            "coordinator/trigger_optimization",
            std::bind(&OptimizationCoordinator::triggerOptimizationCB, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

    void initializeSubscriptions()
    {
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10,
            std::bind(&OptimizationCoordinator::odometryCallback, this, std::placeholders::_1));

        m_metrics_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/fastlio2/performance_metrics", 10,
            std::bind(&OptimizationCoordinator::metricsCallback, this, std::placeholders::_1));

        m_coordination_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/coordinator/metrics", 10);
    }

    // 核心监控逻辑
    void systemMonitor()
    {
        if (!m_config.auto_optimization) return;

        auto now = this->get_clock()->now();

        // 计算累积漂移
        updateDriftEstimate();

        // 检查紧急优化触发条件
        if (shouldTriggerEmergencyOptimization()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 触发紧急优化！漂移: %.3fm", m_metrics.cumulative_drift);
            scheduleEmergencyOptimization();
            return;
        }

        // 检查常规优化触发条件
        if (shouldTriggerRegularOptimization(now)) {
            RCLCPP_INFO(this->get_logger(), "🔄 触发常规优化，漂移: %.3fm", m_metrics.cumulative_drift);
            scheduleRegularOptimization();
        }
    }

    bool shouldTriggerEmergencyOptimization() const
    {
        return m_metrics.cumulative_drift > m_config.emergency_threshold;
    }

    bool shouldTriggerRegularOptimization(rclcpp::Time now) const
    {
        bool drift_exceeded = m_metrics.cumulative_drift > m_config.drift_threshold;
        bool time_exceeded = (now - m_metrics.last_optimization_time).seconds() > m_config.time_threshold;
        bool not_busy = !m_metrics.optimization_in_progress;

        return (drift_exceeded || time_exceeded) && not_busy;
    }

    void scheduleEmergencyOptimization()
    {
        OptimizationTask task;
        task.type = OptimizationTask::PGO_LOOP_CLOSURE;
        task.priority = 10; // 最高优先级
        task.created_time = this->get_clock()->now();
        task.component_name = "emergency_pgo";
        task.emergency = true;

        addTaskToQueue(task);
        processTaskQueue();
    }

    void scheduleRegularOptimization()
    {
        OptimizationTask pgo_task;
        pgo_task.type = OptimizationTask::PGO_LOOP_CLOSURE;
        pgo_task.priority = 5;
        pgo_task.created_time = this->get_clock()->now();
        pgo_task.component_name = "pgo_optimization";
        addTaskToQueue(pgo_task);

        // 如果漂移较大，同时安排HBA优化
        if (m_metrics.cumulative_drift > m_config.drift_threshold * 1.5) {
            OptimizationTask hba_task;
            hba_task.type = OptimizationTask::HBA_REFINEMENT;
            hba_task.priority = 3;
            hba_task.created_time = this->get_clock()->now();
            hba_task.component_name = "hba_refinement";
            addTaskToQueue(hba_task);
        }
    }

    // 任务队列管理
    void addTaskToQueue(const OptimizationTask& task)
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);
        m_task_queue.push(task);
        RCLCPP_DEBUG(this->get_logger(), "任务加入队列: %s, 优先级: %d",
                    task.component_name.c_str(), task.priority);
    }

    void processTaskQueue()
    {
        std::lock_guard<std::mutex> lock(m_queue_mutex);

        if (m_task_queue.empty()) return;
        const OptimizationTask& peek_task = m_task_queue.top();
        if (m_metrics.optimization_in_progress && !peek_task.emergency) return;

        OptimizationTask task = peek_task;
        m_task_queue.pop();

        // 异步执行任务
        executeTask(task);
    }

    // 任务执行
    void executeTask(const OptimizationTask& task)
    {
        m_metrics.optimization_in_progress = true;

        switch (task.type) {
            case OptimizationTask::PGO_LOOP_CLOSURE:
                executePGOOptimization(task);
                break;
            case OptimizationTask::HBA_REFINEMENT:
                executeHBAOptimization(task);
                break;
            case OptimizationTask::LOCALIZER_GLOBAL:
                executeLocalizerOptimization(task);
                break;
            case OptimizationTask::FEEDBACK_UPDATE:
                executeFeedbackUpdate(task);
                break;
        }
    }

    // 具体优化执行
    void executePGOOptimization(const OptimizationTask& /* task */)
    {
        RCLCPP_INFO(this->get_logger(), "🔄 执行PGO优化...");

        // 主动查询PGO优化结果（关键帧优化位姿与轨迹）
        if (!m_pgo_status_client->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "PGO状态服务不可用，进行被动同步");
            requestStateSync("pgo_node", 0);
            finalizeOptimization(true);
            return;
        }

        auto req = std::make_shared<interface::srv::GetOptimizedPose::Request>();
        try {
            auto future = m_pgo_status_client->async_send_request(req);
            auto resp = future.get();
            if (resp && resp->success) {
                // 将优化位姿回写至FAST-LIO2（平滑更新）
                pushUpdatePoseToLIO(resp->optimized_pose, resp->optimization_score, "pgo_node");

                // 向FAST-LIO2同步轨迹（便于内部状态/可视化对齐）
                pushSyncStateToLIO("pgo_node", 0, resp->optimized_trajectory);

                finalizeOptimization(true);
                return;
            } else {
                RCLCPP_WARN(this->get_logger(), "PGO结果无效，进行被动同步");
                requestStateSync("pgo_node", 0);
                finalizeOptimization(false);
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "PGO优化查询异常: %s", e.what());
            requestStateSync("pgo_node", 0);
            finalizeOptimization(false);
            return;
        }
    }

    void executeHBAOptimization(const OptimizationTask& task)
    {
        RCLCPP_INFO(this->get_logger(), "🔧 执行HBA精细化...");

        if (!m_hba_client->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "HBA服务不可用");
            m_metrics.failed_optimizations++;
            m_metrics.optimization_in_progress = false;
            return;
        }

        auto request = std::make_shared<interface::srv::RefineMap::Request>();
        request->maps_path = "/tmp/current_map.pcd";

        // 同步调用简化处理
        try {
            auto future = m_hba_client->async_send_request(request);
            auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(this->get_logger(),
                            "✅ HBA优化完成，迭代:%d，残差:%.6f",
                            response->iterations_used,
                            response->final_residual);

                if (!response->optimized_poses.poses.empty()) {
                    auto pose_with_cov = extractPoseWithCovariance(
                        response->optimized_poses,
                        response->pose_covariances,
                        response->optimized_poses.poses.size() - 1);

                    double score = evaluateHBAScore(response->final_residual);
                    pushUpdatePoseToLIO(pose_with_cov, score, "hba_node");
                    pushSyncStateToLIO("hba_node", 1, response->optimized_poses);
                } else {
                    RCLCPP_WARN(this->get_logger(), "HBA返回的pose数组为空，跳过反馈");
                }

                m_metrics.successful_optimizations++;
                requestStateSync("hba_node", 1);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ HBA优化失败: %s", response->message.c_str());
                m_metrics.failed_optimizations++;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "HBA优化异常: %s", e.what());
            m_metrics.failed_optimizations++;
        }

        m_metrics.optimization_in_progress = false;
        m_metrics.last_optimization_time = this->get_clock()->now();
    }

    void executeLocalizerOptimization(const OptimizationTask& /* task */)
    {
        RCLCPP_INFO(this->get_logger(), "🎯 执行重定位优化...");
        // 实现重定位优化逻辑
        m_metrics.optimization_in_progress = false;
    }

    void executeFeedbackUpdate(const OptimizationTask& /* task */)
    {
        RCLCPP_INFO(this->get_logger(), "📤 执行反馈更新...");
        // 实现反馈更新逻辑
        m_metrics.optimization_in_progress = false;
    }

    void requestStateSync(const std::string& component, int opt_type)
    {
        if (!m_sync_client->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "状态同步服务不可用");
            return;
        }

        auto request = std::make_shared<interface::srv::SyncState::Request>();
        request->component_name = component;
        request->optimization_type = opt_type;

        // 同步调用状态同步
        try {
            auto future = m_sync_client->async_send_request(request);
            auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✅ 状态同步成功: %s", component.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ 状态同步失败: %s", response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "状态同步异常: %s", e.what());
        }
    }

    void pushUpdatePoseToLIO(const geometry_msgs::msg::PoseWithCovariance& optimized_pose,
                              double score,
                              const std::string& source)
    {
        if (!m_fastlio_client->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "FAST-LIO2位姿更新服务不可用");
            return;
        }
        auto req = std::make_shared<interface::srv::UpdatePose::Request>();
        req->optimized_pose = optimized_pose;
        req->header.stamp = this->now();
        req->reset_covariance = false;
        req->optimization_score = score;
        req->source_component = source;
        try {
            auto fut = m_fastlio_client->async_send_request(req);
            (void)fut.get();
        } catch (...) {
        }
    }

    void pushSyncStateToLIO(const std::string& component,
                            int opt_type,
                            const geometry_msgs::msg::PoseArray& traj)
    {
        if (!m_sync_client->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "状态同步服务不可用");
            return;
        }
        auto req = std::make_shared<interface::srv::SyncState::Request>();
        req->component_name = component;
        req->optimization_type = opt_type;
        req->optimized_trajectory = traj;
        req->force_update = true;
        try {
            auto fut = m_sync_client->async_send_request(req);
            (void)fut.get();
        } catch (...) {
        }
    }

    void finalizeOptimization(bool success)
    {
        m_metrics.optimization_in_progress = false;
        m_metrics.last_optimization_time = this->get_clock()->now();
        if (success) m_metrics.successful_optimizations++;
        else m_metrics.failed_optimizations++;
    }

    // 回调函数
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        m_last_pose = msg->pose.pose;

        if (m_reference_pose.position.x == 0 && m_reference_pose.position.y == 0) {
            m_reference_pose = m_last_pose;
        }
    }

    void metricsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) {
            return;
        }

        const double processing_time_ms = msg->data[0];
        const double point_count = msg->data.size() > 1 ? msg->data[1] : 0.0;
        const double imu_buffer = msg->data.size() > 2 ? msg->data[2] : 0.0;
        const double lidar_buffer = msg->data.size() > 3 ? msg->data[3] : 0.0;

        double penalty = 0.0;
        if (processing_time_ms > 40.0) {
            penalty += (processing_time_ms - 40.0) * 0.002; // 时间越久越容易漂移
        }
        if (point_count > 0.0 && point_count < 12000.0) {
            penalty += 0.02; // 点云过稀
        }
        if (imu_buffer > 5.0) {
            penalty += (imu_buffer - 5.0) * 0.002;
        }
        if (lidar_buffer > 3.0) {
            penalty += (lidar_buffer - 3.0) * 0.003;
        }

        // 指标低通滤波，避免抖动
        m_metrics.cumulative_drift = std::clamp(
            m_metrics.cumulative_drift * 0.98 + penalty,
            0.0,
            m_config.emergency_threshold * 2.0);

        m_metrics.last_optimization_score = std::clamp(1.0 - penalty, 0.0, 1.0);
    }

    void triggerOptimizationCB(
        const std::shared_ptr<interface::srv::TriggerOptimization::Request> request,
        std::shared_ptr<interface::srv::TriggerOptimization::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "📞 收到优化触发请求，级别: %d", request->optimization_level);

        if (request->include_pgo) {
            OptimizationTask pgo_task;
            pgo_task.type = OptimizationTask::PGO_LOOP_CLOSURE;
            pgo_task.priority = request->optimization_level * 2;
            pgo_task.created_time = this->get_clock()->now();
            pgo_task.component_name = "manual_pgo";
            pgo_task.emergency = request->emergency_mode;
            addTaskToQueue(pgo_task);
        }

        if (request->include_hba) {
            OptimizationTask hba_task;
            hba_task.type = OptimizationTask::HBA_REFINEMENT;
            hba_task.priority = request->optimization_level * 2 - 1;
            hba_task.created_time = this->get_clock()->now();
            hba_task.component_name = "manual_hba";
            addTaskToQueue(hba_task);
        }

        response->success = true;
        response->message = "优化任务已加入队列";
        response->optimization_score = m_metrics.last_optimization_score;
        response->estimated_duration = 10.0;
        response->active_optimizations = getActiveOptimizationCount();

        if (request->emergency_mode) {
            processTaskQueue();
        }
    }

    // 辅助函数
    void updateDriftEstimate()
    {
        if (m_reference_pose.position.x != 0 || m_reference_pose.position.y != 0) {
            double dx = m_last_pose.position.x - m_reference_pose.position.x;
            double dy = m_last_pose.position.y - m_reference_pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance > 10.0) {
                m_metrics.cumulative_drift += distance * 0.001;
                m_reference_pose = m_last_pose;
            }
        }
    }

    int getActiveOptimizationCount() const
    {
        return m_metrics.optimization_in_progress ? 1 : 0;
    }

    void publishMetrics()
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(8);

        msg.data[0] = m_metrics.cumulative_drift;
        msg.data[1] = m_metrics.last_optimization_score;
        msg.data[2] = m_metrics.successful_optimizations;
        msg.data[3] = m_metrics.failed_optimizations;
        msg.data[4] = getActiveOptimizationCount();
        msg.data[5] = m_task_queue.size();
        msg.data[6] = (this->get_clock()->now() - m_metrics.last_optimization_time).seconds();
        msg.data[7] = m_config.auto_optimization ? 1.0 : 0.0;

        m_coordination_pub->publish(msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "📊 协调器状态 - 漂移:%.3fm, 成功:%d, 失败:%d, 队列:%lu",
            m_metrics.cumulative_drift, m_metrics.successful_optimizations,
            m_metrics.failed_optimizations, m_task_queue.size());
    }

    geometry_msgs::msg::PoseWithCovariance extractPoseWithCovariance(
        const geometry_msgs::msg::PoseArray& poses,
        const std::vector<double>& covariances,
        size_t index) const
    {
        geometry_msgs::msg::PoseWithCovariance result;
        if (poses.poses.size() <= index) {
            return result;
        }
        result.pose = poses.poses[index];
        std::fill(result.covariance.begin(), result.covariance.end(), 0.0);

        const size_t offset = index * 36;
        if (covariances.size() >= offset + 36) {
            std::copy_n(covariances.begin() + offset, 36, result.covariance.begin());
        }
        return result;
    }

    double evaluateHBAScore(double residual) const
    {
        double clamped = std::max(0.0, std::min(residual, 10.0));
        return 1.0 / (1.0 + clamped);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto coordinator = std::make_shared<OptimizationCoordinator>();

    RCLCPP_INFO(coordinator->get_logger(),
        "🚀🚀🚀 真正协同SLAM系统启动 - 优化协调器运行中！🚀🚀🚀");

    rclcpp::spin(coordinator);
    rclcpp::shutdown();

    return 0;
}
