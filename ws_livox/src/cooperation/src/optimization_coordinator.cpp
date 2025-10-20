#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include "interface/msg/system_performance.hpp"
#include "interface/srv/update_pose.hpp"
#include "interface/srv/sync_state.hpp"
#include "interface/srv/trigger_optimization.hpp"
#include "interface/srv/get_optimized_pose.hpp"
#include "interface/srv/refine_map.hpp"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <deque>
#include <numeric>
#include <fstream>
#include <sys/resource.h>
#include <unistd.h>
#include <Eigen/Geometry>
#include <atomic>

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
        bool enable_relocalization = true;
        double min_pose_delta = 0.03;
        double min_orientation_delta_deg = 0.5;
        double optimization_score_threshold = 0.6;
        int drift_history_size = 20;
        double relocalization_score_threshold = 0.15;
        double relocalization_timeout_sec = 10.0;
        double relocalization_min_translation = 0.5;
        double relocalization_min_yaw = 5.0; // degrees
        int service_timeout_ms = 3000;
        int emergency_failure_threshold = 3;
        std::string hba_maps_path = "/tmp/current_map.pcd";
    } m_config;

    // 客户端和服务
    rclcpp::Client<interface::srv::RefineMap>::SharedPtr m_hba_client;
    rclcpp::Client<interface::srv::Relocalize>::SharedPtr m_localizer_client;
    rclcpp::Client<interface::srv::IsValid>::SharedPtr m_reloc_check_client;
    rclcpp::Client<interface::srv::UpdatePose>::SharedPtr m_fastlio_client;
    rclcpp::Client<interface::srv::SyncState>::SharedPtr m_sync_client;
    rclcpp::Client<interface::srv::GetOptimizedPose>::SharedPtr m_pgo_status_client;

    rclcpp::Service<interface::srv::TriggerOptimization>::SharedPtr m_trigger_service;

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_metrics_sub;

    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_coordination_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_health_pub;
    rclcpp::Publisher<interface::msg::SystemPerformance>::SharedPtr m_performance_pub;

    // 定时器
    rclcpp::TimerBase::SharedPtr m_monitor_timer;
    rclcpp::TimerBase::SharedPtr m_task_timer;
    rclcpp::TimerBase::SharedPtr m_metrics_timer;

    // 系统状态
    SystemMetrics m_metrics;
    std::deque<double> m_recent_drift_changes;
    std::priority_queue<OptimizationTask, std::vector<OptimizationTask>, TaskComparator> m_task_queue;
    mutable std::mutex m_queue_mutex;
    mutable std::mutex m_metrics_mutex;
    mutable std::mutex m_pose_mutex;
    std::atomic<bool> m_pending_emergency{false};

    geometry_msgs::msg::Pose m_last_pose;
    geometry_msgs::msg::Pose m_reference_pose;
    double m_last_cpu_total = 0.0;
    double m_last_cpu_usage = 0.0;
    rclcpp::Time m_last_cpu_sample;
    long m_cpu_cores = 1;

    void loadConfiguration()
    {
        this->declare_parameter("drift_threshold", 0.5);
        this->declare_parameter("time_threshold", 60.0);
        this->declare_parameter("emergency_threshold", 2.0);
        this->declare_parameter("auto_optimization", true);
        this->declare_parameter("enable_relocalization", m_config.enable_relocalization);
        this->declare_parameter("min_pose_delta", m_config.min_pose_delta);
        this->declare_parameter("min_orientation_delta_deg", m_config.min_orientation_delta_deg);
        this->declare_parameter("optimization_score_threshold", m_config.optimization_score_threshold);
        this->declare_parameter("drift_history_size", m_config.drift_history_size);
        this->declare_parameter("relocalization_score_threshold", m_config.relocalization_score_threshold);
        this->declare_parameter("relocalization_timeout_sec", m_config.relocalization_timeout_sec);
        this->declare_parameter("relocalization_min_translation", m_config.relocalization_min_translation);
        this->declare_parameter("relocalization_min_yaw", m_config.relocalization_min_yaw);
        this->declare_parameter("service_timeout_ms", m_config.service_timeout_ms);
        this->declare_parameter("emergency_failure_threshold", m_config.emergency_failure_threshold);
        this->declare_parameter("hba_maps_path", m_config.hba_maps_path);

        m_config.drift_threshold = this->get_parameter("drift_threshold").as_double();
        m_config.time_threshold = this->get_parameter("time_threshold").as_double();
        m_config.emergency_threshold = this->get_parameter("emergency_threshold").as_double();
        m_config.auto_optimization = this->get_parameter("auto_optimization").as_bool();
        m_config.enable_relocalization = this->get_parameter("enable_relocalization").as_bool();
        m_config.min_pose_delta = this->get_parameter("min_pose_delta").as_double();
        m_config.min_orientation_delta_deg = this->get_parameter("min_orientation_delta_deg").as_double();
        m_config.optimization_score_threshold = this->get_parameter("optimization_score_threshold").as_double();
        m_config.drift_history_size = this->get_parameter("drift_history_size").as_int();
        m_config.relocalization_score_threshold = this->get_parameter("relocalization_score_threshold").as_double();
        m_config.relocalization_timeout_sec = this->get_parameter("relocalization_timeout_sec").as_double();
        m_config.relocalization_min_translation = this->get_parameter("relocalization_min_translation").as_double();
        m_config.relocalization_min_yaw = this->get_parameter("relocalization_min_yaw").as_double();
        auto timeout_param = this->get_parameter("service_timeout_ms").as_int();
        m_config.service_timeout_ms = static_cast<int>(std::max<int64_t>(0, timeout_param));
        int failure_threshold = static_cast<int>(this->get_parameter("emergency_failure_threshold").as_int());
        m_config.emergency_failure_threshold = std::max(1, failure_threshold);
        m_config.hba_maps_path = this->get_parameter("hba_maps_path").as_string();
        if (m_config.hba_maps_path.empty()) {
            m_config.hba_maps_path = "/tmp/current_map.pcd";
        }

        RCLCPP_INFO(this->get_logger(), "配置加载: 漂移阈值=%.2fm, 时间阈值=%.1fs",
                   m_config.drift_threshold, m_config.time_threshold);
    }

    void initializeClients()
    {
        m_hba_client = this->create_client<interface::srv::RefineMap>("hba/refine_map");
        m_localizer_client = this->create_client<interface::srv::Relocalize>("localizer/relocalize");
        m_reloc_check_client = this->create_client<interface::srv::IsValid>("localizer/relocalize_check");
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
        m_health_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/coordinator/health", 10);
        m_performance_pub = this->create_publisher<interface::msg::SystemPerformance>(
            "/coordinator/system_performance", 10);

        m_cpu_cores = std::max<long>(1, sysconf(_SC_NPROCESSORS_ONLN));
        m_last_cpu_sample = this->get_clock()->now();
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
            double drift_snapshot = 0.0;
            {
                std::lock_guard<std::mutex> lock(m_metrics_mutex);
                drift_snapshot = m_metrics.cumulative_drift;
            }
            RCLCPP_WARN(this->get_logger(), "⚠️ 触发紧急优化！漂移: %.3fm", drift_snapshot);
            scheduleEmergencyOptimization();
            return;
        }

        // 检查常规优化触发条件
        if (shouldTriggerRegularOptimization(now)) {
            double drift_snapshot = 0.0;
            {
                std::lock_guard<std::mutex> lock(m_metrics_mutex);
                drift_snapshot = m_metrics.cumulative_drift;
            }
            RCLCPP_INFO(this->get_logger(), "🔄 触发常规优化，漂移: %.3fm", drift_snapshot);
            scheduleRegularOptimization();
        }
    }

    bool shouldTriggerEmergencyOptimization()
    {
        std::lock_guard<std::mutex> lock(m_metrics_mutex);
        if (m_metrics.optimization_in_progress) {
            return false;
        }
        bool drift_critical = m_metrics.cumulative_drift > m_config.emergency_threshold;
        bool failure_exceeded = m_metrics.failed_optimizations >= m_config.emergency_failure_threshold;
        return drift_critical && failure_exceeded;
    }

    bool shouldTriggerRegularOptimization(rclcpp::Time now)
    {
        SystemMetrics snapshot;
        {
            std::lock_guard<std::mutex> lock(m_metrics_mutex);
            snapshot = m_metrics;
        }
        bool drift_exceeded = snapshot.cumulative_drift > m_config.drift_threshold;
        bool time_exceeded = (now - snapshot.last_optimization_time).seconds() > m_config.time_threshold;
        bool not_busy = !snapshot.optimization_in_progress;

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
        OptimizationTask task;
        {
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            if (m_task_queue.empty()) return;
            task = m_task_queue.top();
            m_task_queue.pop();
        }

        if (!tryAcquireOptimization(task))
        {
            addTaskToQueue(task);
            return;
        }

        // 异步执行任务
        executeTask(task);
    }

    bool tryAcquireOptimization(const OptimizationTask& task)
    {
        std::lock_guard<std::mutex> lock(m_metrics_mutex);
        if (m_metrics.optimization_in_progress) {
            if (task.emergency) {
                m_pending_emergency.store(true, std::memory_order_relaxed);
            }
            return false;
        }
        m_metrics.optimization_in_progress = true;
        m_pending_emergency.store(false, std::memory_order_relaxed);
        return true;
    }

    // 任务执行
    void executeTask(const OptimizationTask& task)
    {
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
        auto timeout = serviceWaitTimeout();
        if (!m_pgo_status_client->wait_for_service(timeout)) {
            RCLCPP_WARN(this->get_logger(), "PGO状态服务不可用，进行被动同步");
            requestStateSync("pgo_node", 0);
            finalizeOptimization(true);
            return;
        }

        auto req = std::make_shared<interface::srv::GetOptimizedPose::Request>();
        geometry_msgs::msg::Pose pre_pose;
        {
            std::lock_guard<std::mutex> lock(m_pose_mutex);
            pre_pose = m_last_pose;
        }

        auto callback = [this, pre_pose](rclcpp::Client<interface::srv::GetOptimizedPose>::SharedFuture result) {
            try {
                auto resp = result.get();
                if (resp && resp->success) {
                    double rotation_deg = 0.0;
                    double translation = calculatePoseDelta(pre_pose, resp->optimized_pose.pose, rotation_deg);
                    if (resp->optimization_score < m_config.optimization_score_threshold &&
                        translation < m_config.min_pose_delta &&
                        rotation_deg < m_config.min_orientation_delta_deg) {
                        RCLCPP_WARN(this->get_logger(),
                                    "PGO optimization rejected: delta %.4fm, rotation %.3fdeg, score %.3f",
                                    translation, rotation_deg, resp->optimization_score);
                        requestStateSync("pgo_node", 0);
                        finalizeOptimization(false);
                        return;
                    }

                    // 将优化位姿回写至FAST-LIO2（平滑更新）
                    pushUpdatePoseToLIO(resp->optimized_pose, resp->optimization_score, "pgo_node");

                    // 向FAST-LIO2同步轨迹（便于内部状态/可视化对齐）
                    pushSyncStateToLIO("pgo_node", 0, resp->optimized_trajectory);

                    finalizeOptimization(true);
                } else {
                    RCLCPP_WARN(this->get_logger(), "PGO结果无效，进行被动同步");
                    requestStateSync("pgo_node", 0);
                    finalizeOptimization(false);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PGO优化查询异常: %s", e.what());
                requestStateSync("pgo_node", 0);
                finalizeOptimization(false);
            }
        };

        m_pgo_status_client->async_send_request(req, callback);
    }

    void executeHBAOptimization(const OptimizationTask& /*task*/)
    {
        RCLCPP_INFO(this->get_logger(), "🔧 执行HBA精细化...");

        auto timeout = serviceWaitTimeout();
        if (!m_hba_client->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "HBA服务不可用");
            finalizeOptimization(false);
            return;
        }

        auto request = std::make_shared<interface::srv::RefineMap::Request>();
        request->maps_path = m_config.hba_maps_path;

        auto callback = [this](rclcpp::Client<interface::srv::RefineMap>::SharedFuture result) {
            try {
                auto response = result.get();

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

                    requestStateSync("hba_node", 1);
                    finalizeOptimization(true);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ HBA优化失败: %s", response->message.c_str());
                    finalizeOptimization(false);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "HBA优化异常: %s", e.what());
                finalizeOptimization(false);
            }
        };

        m_hba_client->async_send_request(request, callback);
    }

    void executeLocalizerOptimization(const OptimizationTask& /* task */)
    {
        if (!m_config.enable_relocalization) {
            RCLCPP_WARN(this->get_logger(), "重定位功能已禁用，直接跳过");
            finalizeOptimization(false);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 执行重定位优化...");

        geometry_msgs::msg::PoseWithCovariance current_pose;
        {
            std::lock_guard<std::mutex> lock(m_pose_mutex);
            current_pose.pose = m_last_pose;
        }

        geometry_msgs::msg::Pose best_pose;
        if (!requestLocalizerRelocalization(current_pose.pose, best_pose)) {
            RCLCPP_WARN(this->get_logger(), "重定位请求失败或服务不可用");
            finalizeOptimization(false);
            return;
        }

        double rotation_deg = 0.0;
        double translation = calculatePoseDelta(current_pose.pose, best_pose, rotation_deg);
        if (translation < m_config.relocalization_min_translation &&
            rotation_deg < m_config.relocalization_min_yaw) {
            RCLCPP_INFO(this->get_logger(),
                        "重定位偏移过小(%.3fm, %.2fdeg)，不推送FAST-LIO2", translation, rotation_deg);
            finalizeOptimization(false);
            return;
        }

        geometry_msgs::msg::PoseWithCovariance pose_with_cov;
        pose_with_cov.pose = best_pose;
        std::fill(pose_with_cov.covariance.begin(), pose_with_cov.covariance.end(), 0.0);

        pushUpdatePoseToLIO(pose_with_cov,
                            m_config.relocalization_score_threshold,
                            "localizer");
        pushSyncStateToLIO("localizer", 2, geometry_msgs::msg::PoseArray());
        finalizeOptimization(true);
    }

    void executeFeedbackUpdate(const OptimizationTask& /* task */)
    {
        RCLCPP_INFO(this->get_logger(), "📤 执行反馈更新...");
        // 实现反馈更新逻辑
        finalizeOptimization(true);
    }

    void requestStateSync(const std::string& component, int opt_type)
    {
        auto timeout = serviceWaitTimeout();
        if (!m_sync_client->wait_for_service(timeout)) {
            RCLCPP_WARN(this->get_logger(), "状态同步服务不可用");
            return;
        }

        auto request = std::make_shared<interface::srv::SyncState::Request>();
        request->component_name = component;
        request->optimization_type = opt_type;

        auto callback = [this, component](rclcpp::Client<interface::srv::SyncState>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "✅ 状态同步成功: %s", component.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 状态同步失败: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "状态同步异常: %s", e.what());
            }
        };

        m_sync_client->async_send_request(request, callback);
    }

    void pushUpdatePoseToLIO(const geometry_msgs::msg::PoseWithCovariance& optimized_pose,
                              double score,
                              const std::string& source)
    {
        auto timeout = serviceWaitTimeout();
        if (!m_fastlio_client->wait_for_service(timeout)) {
            RCLCPP_WARN(this->get_logger(), "FAST-LIO2位姿更新服务不可用");
            return;
        }
        auto req = std::make_shared<interface::srv::UpdatePose::Request>();
        req->optimized_pose = optimized_pose;
        req->header.stamp = this->now();
        req->reset_covariance = false;
        req->optimization_score = score;
        req->source_component = source;
        auto callback = [this, source](rclcpp::Client<interface::srv::UpdatePose>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(),
                                "FAST-LIO2拒绝位姿更新(来源:%s): %s",
                                source.c_str(), response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                             "FAST-LIO2位姿更新异常(来源:%s): %s",
                             source.c_str(), e.what());
            }
        };
        m_fastlio_client->async_send_request(req, callback);
    }

    void pushSyncStateToLIO(const std::string& component,
                            int opt_type,
                            const geometry_msgs::msg::PoseArray& traj)
    {
        auto timeout = serviceWaitTimeout();
        if (!m_sync_client->wait_for_service(timeout)) {
            RCLCPP_WARN(this->get_logger(), "状态同步服务不可用");
            return;
        }
        auto req = std::make_shared<interface::srv::SyncState::Request>();
        req->component_name = component;
        req->optimization_type = opt_type;
        req->optimized_trajectory = traj;
        req->force_update = true;

        auto callback = [this, component](rclcpp::Client<interface::srv::SyncState>::SharedFuture future) {
            try {
                auto resp = future.get();
                if (!resp->success) {
                    RCLCPP_WARN(this->get_logger(),
                                "FAST-LIO2状态同步失败: %s", resp->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                             "FAST-LIO2状态同步异常: %s", e.what());
            }
        };

        m_sync_client->async_send_request(req, callback);
    }

    void finalizeOptimization(bool success)
    {
        auto now = this->get_clock()->now();
        bool had_pending_emergency = false;
        {
            std::lock_guard<std::mutex> lock(m_metrics_mutex);
            m_metrics.optimization_in_progress = false;
            m_metrics.last_optimization_time = now;
            if (success) {
                m_metrics.successful_optimizations++;
            } else {
                m_metrics.failed_optimizations++;
            }
            had_pending_emergency = m_pending_emergency.exchange(false, std::memory_order_relaxed);
        }

        if (had_pending_emergency) {
            RCLCPP_INFO(this->get_logger(), "⚡ 紧急优化请求抢占触发，立即调度");
        }

        processTaskQueue();
    }

    std::chrono::milliseconds serviceWaitTimeout() const
    {
        return std::chrono::milliseconds(std::max(0, m_config.service_timeout_ms));
    }

    // 回调函数
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_pose_mutex);
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

        double drift_increment = penalty;
        if (m_recent_drift_changes.empty()) {
            drift_increment += m_config.drift_threshold * 0.02;
        }

        {
            std::lock_guard<std::mutex> lock(m_metrics_mutex);
            m_recent_drift_changes.push_back(drift_increment);
            while (m_recent_drift_changes.size() > static_cast<size_t>(m_config.drift_history_size)) {
                m_recent_drift_changes.pop_front();
            }

            double drift_sum = std::accumulate(m_recent_drift_changes.begin(), m_recent_drift_changes.end(), 0.0);
            m_metrics.cumulative_drift = std::clamp(drift_sum,
                                                    0.0,
                                                    m_config.emergency_threshold * 2.0);

            double avg_penalty = drift_sum / std::max<size_t>(m_recent_drift_changes.size(), 1);
            m_metrics.last_optimization_score = std::clamp(1.0 - avg_penalty, 0.0, 1.0);
        }
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
        std::lock_guard<std::mutex> pose_lock(m_pose_mutex);
        if (m_reference_pose.position.x != 0 || m_reference_pose.position.y != 0) {
            double dx = m_last_pose.position.x - m_reference_pose.position.x;
            double dy = m_last_pose.position.y - m_reference_pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance > 10.0) {
                std::lock_guard<std::mutex> lock(m_metrics_mutex);
                m_recent_drift_changes.push_back(distance * 0.001);
                while (m_recent_drift_changes.size() > static_cast<size_t>(m_config.drift_history_size)) {
                    m_recent_drift_changes.pop_front();
                }
                double drift_sum = std::accumulate(m_recent_drift_changes.begin(), m_recent_drift_changes.end(), 0.0);
                m_metrics.cumulative_drift = std::min(drift_sum, m_config.emergency_threshold * 2.0);
                m_reference_pose = m_last_pose;
            }
        }
    }

    int getActiveOptimizationCount() const
    {
        std::lock_guard<std::mutex> lock(m_metrics_mutex);
        return m_metrics.optimization_in_progress ? 1 : 0;
    }

    void publishMetrics()
    {
        SystemMetrics metrics_snapshot;
        {
            std::lock_guard<std::mutex> lock(m_metrics_mutex);
            metrics_snapshot = m_metrics;
        }

        size_t queue_size = 0;
        {
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            queue_size = m_task_queue.size();
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(8);

        msg.data[0] = metrics_snapshot.cumulative_drift;
        msg.data[1] = metrics_snapshot.last_optimization_score;
        msg.data[2] = metrics_snapshot.successful_optimizations;
        msg.data[3] = metrics_snapshot.failed_optimizations;
        msg.data[4] = metrics_snapshot.optimization_in_progress ? 1.0 : 0.0;
        msg.data[5] = static_cast<double>(queue_size);
        msg.data[6] = (this->get_clock()->now() - metrics_snapshot.last_optimization_time).seconds();
        msg.data[7] = m_config.auto_optimization ? 1.0 : 0.0;

        m_coordination_pub->publish(msg);
        publishComponentHealth(metrics_snapshot, queue_size);
        if (m_performance_pub && m_performance_pub->get_subscription_count() > 0) {
            m_performance_pub->publish(buildSystemPerformance(metrics_snapshot));
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "📊 协调器状态 - 漂移:%.3fm, 成功:%d, 失败:%d, 队列:%lu",
            metrics_snapshot.cumulative_drift, metrics_snapshot.successful_optimizations,
            metrics_snapshot.failed_optimizations, queue_size);
    }

    void publishComponentHealth(const SystemMetrics& metrics_snapshot, size_t queue_size)
    {
        if (!m_health_pub || m_health_pub->get_subscription_count() == 0) {
            return;
        }

        diagnostic_msgs::msg::DiagnosticArray array_msg;
        array_msg.header.stamp = this->get_clock()->now();

        auto make_status = [](const std::string& name, bool ok, const std::string& ok_msg, const std::string& warn_msg) {
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = name;
            status.hardware_id = "mid360_coordinator";
            if (ok) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = ok_msg;
            } else {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = warn_msg;
            }
            return status;
        };

        array_msg.status.push_back(make_status(
            "pgo_service",
            m_pgo_status_client && m_pgo_status_client->service_is_ready(),
            "ready",
            "pgo/get_optimized_pose unavailable"));

        array_msg.status.push_back(make_status(
            "fastlio_update_pose",
            m_fastlio_client && m_fastlio_client->service_is_ready(),
            "ready",
            "fastlio2/update_pose unavailable"));

        array_msg.status.push_back(make_status(
            "sync_state",
            m_sync_client && m_sync_client->service_is_ready(),
            "ready",
            "fastlio2/sync_state unavailable"));

        array_msg.status.push_back(make_status(
            "hba_refine_map",
            m_hba_client && m_hba_client->service_is_ready(),
            "ready",
            "hba/refine_map unavailable"));

        diagnostic_msgs::msg::DiagnosticStatus metrics_status;
        metrics_status.name = "optimization_metrics";
        metrics_status.hardware_id = "mid360_coordinator";
        metrics_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        metrics_status.message = "metrics snapshot";
        metrics_status.values.reserve(4);

        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "cumulative_drift";
        kv.value = std::to_string(metrics_snapshot.cumulative_drift);
        metrics_status.values.push_back(kv);

        kv.key = "last_score";
        kv.value = std::to_string(metrics_snapshot.last_optimization_score);
        metrics_status.values.push_back(kv);

        kv.key = "failed_optimizations";
        kv.value = std::to_string(metrics_snapshot.failed_optimizations);
        metrics_status.values.push_back(kv);

        kv.key = "queue_size";
        kv.value = std::to_string(queue_size);
        metrics_status.values.push_back(kv);
        array_msg.status.push_back(metrics_status);

        m_health_pub->publish(array_msg);
    }

    interface::msg::SystemPerformance buildSystemPerformance(const SystemMetrics& metrics_snapshot)
    {
        interface::msg::SystemPerformance msg;
        msg.stamp = this->get_clock()->now();
        msg.cpu_usage = sampleCpuUsage();
        msg.memory_usage_mb = sampleMemoryUsageMb();
        msg.drift = metrics_snapshot.cumulative_drift;
        msg.optimization_score = metrics_snapshot.last_optimization_score;
        msg.successful_optimizations = static_cast<uint32_t>(metrics_snapshot.successful_optimizations);
        msg.failed_optimizations = static_cast<uint32_t>(metrics_snapshot.failed_optimizations);
        return msg;
    }

    double sampleCpuUsage()
    {
        struct rusage usage {};
        if (getrusage(RUSAGE_SELF, &usage) != 0) {
            return m_last_cpu_usage;
        }

        double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
        double sys = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
        double total = user + sys;

        auto now = this->get_clock()->now();
        double dt = (m_last_cpu_sample.nanoseconds() > 0) ? (now - m_last_cpu_sample).seconds() : 0.0;
        double cpu = m_last_cpu_usage;
        if (dt > 0.0) {
            double delta = std::max(0.0, total - m_last_cpu_total);
            cpu = std::max(0.0, std::min(1.0, delta / (dt * static_cast<double>(m_cpu_cores))));
        }

        m_last_cpu_total = total;
        m_last_cpu_sample = now;
        m_last_cpu_usage = cpu;
        return cpu;
    }

    double sampleMemoryUsageMb() const
    {
        std::ifstream statm("/proc/self/statm");
        long total = 0;
        long resident = 0;
        if (!(statm >> total >> resident)) {
            return 0.0;
        }
        const long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
        return resident * static_cast<double>(page_size_kb) / 1024.0;
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

    double calculatePoseDelta(const geometry_msgs::msg::Pose& before,
                               const geometry_msgs::msg::Pose& after,
                               double& rotation_deg) const
    {
        Eigen::Vector3d t_before(before.position.x, before.position.y, before.position.z);
        Eigen::Vector3d t_after(after.position.x, after.position.y, after.position.z);
        Eigen::Quaterniond q_before(before.orientation.w, before.orientation.x,
                                    before.orientation.y, before.orientation.z);
        Eigen::Quaterniond q_after(after.orientation.w, after.orientation.x,
                                   after.orientation.y, after.orientation.z);
        q_before.normalize();
        q_after.normalize();
        rotation_deg = q_before.angularDistance(q_after) * 180.0 / M_PI;
        return (t_before - t_after).norm();
    }

    bool requestLocalizerRelocalization(const geometry_msgs::msg::Pose& guess,
                                        geometry_msgs::msg::Pose& refined_pose)
    {
        if (!m_localizer_client || !m_localizer_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "重定位服务不可用");
            return false;
        }

        auto request = std::make_shared<interface::srv::Relocalize::Request>();
        request->x = guess.position.x;
        request->y = guess.position.y;
        request->z = guess.position.z;
        Eigen::Quaterniond q(guess.orientation.w, guess.orientation.x,
                             guess.orientation.y, guess.orientation.z);
        Eigen::Vector3d euler = q.normalized().toRotationMatrix().eulerAngles(2, 0, 1);
        request->yaw = euler[0];
        request->roll = euler[1];
        request->pitch = euler[2];
        request->pcd_path = "";
        request->force_relocalize = true;

        interface::srv::Relocalize::Response::SharedPtr response;
        try {
            auto future = m_localizer_client->async_send_request(request);
            if (future.wait_for(std::chrono::duration<double>(m_config.relocalization_timeout_sec))
                != std::future_status::ready) {
                RCLCPP_WARN(this->get_logger(), "重定位服务超时%.1fs", m_config.relocalization_timeout_sec);
                return false;
            }
            response = future.get();
            if (!response || !response->success) {
                RCLCPP_WARN(this->get_logger(), "重定位失败: %s",
                            response ? response->message.c_str() : "空响应");
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用重定位服务异常: %s", e.what());
            return false;
        }

        auto check_request = std::make_shared<interface::srv::IsValid::Request>();
        check_request->code = 0;
        try {
            auto check_future = m_reloc_check_client->async_send_request(check_request);
            if (check_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
                RCLCPP_WARN(this->get_logger(), "重定位结果校验超时" );
                return false;
            }
            auto check_resp = check_future.get();
            if (!check_resp->valid) {
                RCLCPP_WARN(this->get_logger(), "重定位校验失败" );
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "重定位校验异常: %s", e.what());
            return false;
        }

        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "重定位响应为空，没法更新姿态");
            return false;
        }

        refined_pose = response->refined_pose;
        // 这个SB服务有时候不归一化四元数，老王顺手帮它归一一下
        Eigen::Quaterniond refined_q(
            refined_pose.orientation.w,
            refined_pose.orientation.x,
            refined_pose.orientation.y,
            refined_pose.orientation.z);
        if (refined_q.norm() > 1e-6) {
            refined_q.normalize();
            refined_pose.orientation.w = refined_q.w();
            refined_pose.orientation.x = refined_q.x();
            refined_pose.orientation.y = refined_q.y();
            refined_pose.orientation.z = refined_q.z();
        }
        return true;
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
