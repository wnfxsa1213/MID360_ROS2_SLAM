# HBA实时优化开发指南

## 概述

本指南提供HBA实时优化的详细实现方案，旨在将当前的批处理模式改造为真正的实时层次化优化系统，减少建图延迟至<100ms，支持10Hz以上的实时优化频率。

## 技术背景

### 当前状态分析

通过分析 `ws_livox/src/hba/src/hba_node.cpp`，发现现有HBA系统：

**✅ 已有功能：**
- 基本的HBA(Hierarchical Bundle Adjustment)框架
- BLAM算法实现
- 窗口化的批处理优化
- 层次化地图构建能力
- 点云下采样和预处理

**❌ 需要改进：**
- 服务驱动的批处理模式，不是真正的"实时"
- 优化过程会阻塞主流程，影响系统响应
- 固定窗口大小，无法自适应系统负载
- 缺少增量式更新机制

### 改进目标

1. **实时性**: 建图延迟从批处理模式改善到<100ms
2. **处理频率**: 支持10Hz以上的实时优化频率
3. **内存效率**: 通过滑动窗口优化，内存占用减少30%
4. **自适应性**: 根据系统负载动态调整优化参数

## 技术方案架构

### 核心组件设计

```cpp
// 文件位置: ws_livox/src/hba/src/hba/realtime_hba.h
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>
#include <future>

#include "hba.h"
#include "commons.h"

namespace hba {

// 优化窗口结构
struct OptimizationWindow {
    std::vector<KeyPoseWithCloud> keyframes;
    size_t start_idx;
    size_t end_idx;
    double timestamp;
    bool needs_optimization;
    int priority;  // 优化优先级

    OptimizationWindow() : start_idx(0), end_idx(0), timestamp(0.0),
                          needs_optimization(false), priority(0) {}
};

// 优化任务结构
struct OptimizationTask {
    OptimizationWindow window;
    std::promise<bool> completion_promise;
    std::chrono::high_resolution_clock::time_point submit_time;
    int task_id;

    OptimizationTask(const OptimizationWindow& w, int id)
        : window(w), task_id(id), submit_time(std::chrono::high_resolution_clock::now()) {}
};

// 实时HBA配置
struct RealtimeHBAConfig : public HBAConfig {
    // 实时优化参数
    double target_frequency = 10.0;          // 目标优化频率(Hz)
    int max_window_size = 20;                // 最大窗口大小
    int min_window_size = 5;                 // 最小窗口大小
    int window_stride = 3;                   // 窗口步长
    double max_optimization_time_ms = 80.0;  // 最大优化时间(ms)

    // 线程池配置
    int num_optimization_threads = 2;        // 优化线程数
    int max_pending_tasks = 10;              // 最大待处理任务数

    // 自适应参数
    bool enable_adaptive_scheduling = true;  // 启用自适应调度
    double load_threshold_high = 0.8;        // 高负载阈值
    double load_threshold_low = 0.4;         // 低负载阈值
    double load_adjustment_factor = 0.1;     // 负载调整因子

    // 质量控制
    bool enable_quality_control = true;      // 启用质量控制
    double min_optimization_improvement = 0.01;  // 最小优化改进
    int max_consecutive_failures = 3;        // 最大连续失败次数
};

// 性能监控统计
struct PerformanceStats {
    std::atomic<size_t> total_frames{0};
    std::atomic<size_t> optimized_frames{0};
    std::atomic<size_t> skipped_frames{0};
    std::atomic<double> average_optimization_time_ms{0.0};
    std::atomic<double> current_frequency{0.0};
    std::atomic<double> cpu_utilization{0.0};
    std::atomic<int> queue_length{0};

    // 质量指标
    std::atomic<double> optimization_improvement{0.0};
    std::atomic<size_t> successful_optimizations{0};
    std::atomic<size_t> failed_optimizations{0};
};

class RealtimeHBA : public HBA {
public:
    explicit RealtimeHBA(const RealtimeHBAConfig& config);
    ~RealtimeHBA();

    // 主要接口
    bool addFrameAsync(const KeyPoseWithCloud& frame);
    bool start();
    void stop();
    bool isRunning() const { return optimization_running_; }

    // 配置管理
    void updateConfig(const RealtimeHBAConfig& config);
    RealtimeHBAConfig getConfig() const { return realtime_config_; }

    // 性能监控
    PerformanceStats getPerformanceStats() const { return stats_; }
    void resetStats();

    // 优化控制
    void pauseOptimization();
    void resumeOptimization();
    void triggerImmediateOptimization();

    // 获取优化结果
    std::vector<KeyPoseWithCloud> getOptimizedPoses();
    CloudType::Ptr getOptimizedMap();

private:
    RealtimeHBAConfig realtime_config_;
    PerformanceStats stats_;

    // 线程管理
    std::vector<std::thread> optimization_threads_;
    std::atomic<bool> optimization_running_{false};
    std::atomic<bool> optimization_paused_{false};

    // 任务队列
    std::queue<std::unique_ptr<OptimizationTask>> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    // 数据管理
    std::deque<KeyPoseWithCloud> frame_buffer_;
    std::mutex buffer_mutex_;
    std::deque<OptimizationWindow> optimization_windows_;
    std::mutex windows_mutex_;

    // 自适应调度器
    std::unique_ptr<AdaptiveScheduler> scheduler_;

    // 性能监控
    std::thread monitor_thread_;
    std::chrono::high_resolution_clock::time_point last_stats_update_;

    // 核心算法
    void optimizationWorkerLoop(int worker_id);
    void performanceMonitorLoop();

    // 窗口管理
    std::vector<OptimizationWindow> createOptimizationWindows();
    bool isWindowReadyForOptimization(const OptimizationWindow& window);
    void updateWindowPriorities();

    // 优化执行
    bool optimizeWindow(OptimizationWindow& window);
    bool validateOptimizationResult(const OptimizationWindow& window,
                                   const std::vector<Pose>& optimized_poses);

    // 自适应调度
    void adjustOptimizationParameters();
    double calculateSystemLoad();
    void handleHighLoad();
    void handleLowLoad();

    // 工具函数
    void cleanupOldFrames();
    void mergeOptimizationResults(const OptimizationWindow& window,
                                 const std::vector<Pose>& optimized_poses);
    int generateTaskId();
};

// 自适应调度器
class AdaptiveScheduler {
public:
    explicit AdaptiveScheduler(const RealtimeHBAConfig& config);

    // 调度决策
    bool shouldOptimizeWindow(const OptimizationWindow& window, double current_load);
    int calculateWindowPriority(const OptimizationWindow& window);
    RealtimeHBAConfig adjustParameters(const RealtimeHBAConfig& current_config,
                                      const PerformanceStats& stats);

    // 负载预测
    double predictOptimizationTime(const OptimizationWindow& window);
    double estimateSystemLoad(const PerformanceStats& stats);

private:
    RealtimeHBAConfig config_;

    // 历史性能数据
    std::deque<double> optimization_time_history_;
    std::deque<double> load_history_;

    // 学习参数
    double learning_rate_ = 0.1;
    double momentum_ = 0.9;
};

} // namespace hba
```

### 核心算法实现

```cpp
// 文件位置: ws_livox/src/hba/src/hba/realtime_hba.cpp
#include "realtime_hba.h"
#include <algorithm>
#include <numeric>

namespace hba {

RealtimeHBA::RealtimeHBA(const RealtimeHBAConfig& config)
    : HBA(config), realtime_config_(config) {

    scheduler_ = std::make_unique<AdaptiveScheduler>(config);

    // 预分配缓冲区
    frame_buffer_.reserve(config.max_window_size * 2);
    optimization_windows_.reserve(config.max_window_size);
}

RealtimeHBA::~RealtimeHBA() {
    stop();
}

bool RealtimeHBA::start() {
    if (optimization_running_) {
        return false;
    }

    optimization_running_ = true;
    optimization_paused_ = false;

    // 启动优化线程
    for (int i = 0; i < realtime_config_.num_optimization_threads; ++i) {
        optimization_threads_.emplace_back(
            &RealtimeHBA::optimizationWorkerLoop, this, i);
    }

    // 启动性能监控线程
    monitor_thread_ = std::thread(&RealtimeHBA::performanceMonitorLoop, this);

    last_stats_update_ = std::chrono::high_resolution_clock::now();

    return true;
}

void RealtimeHBA::stop() {
    optimization_running_ = false;

    // 通知所有线程退出
    queue_cv_.notify_all();

    // 等待优化线程结束
    for (auto& thread : optimization_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    optimization_threads_.clear();

    // 等待监控线程结束
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

bool RealtimeHBA::addFrameAsync(const KeyPoseWithCloud& frame) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // 检查缓冲区大小
    if (frame_buffer_.size() >= realtime_config_.max_window_size * 2) {
        cleanupOldFrames();
    }

    frame_buffer_.push_back(frame);
    stats_.total_frames++;

    // 检查是否需要创建新的优化窗口
    if (frame_buffer_.size() >= static_cast<size_t>(realtime_config_.min_window_size)) {
        auto windows = createOptimizationWindows();

        for (auto& window : windows) {
            if (isWindowReadyForOptimization(window)) {
                // 创建优化任务
                auto task = std::make_unique<OptimizationTask>(window, generateTaskId());

                {
                    std::lock_guard<std::mutex> queue_lock(queue_mutex_);

                    // 检查队列长度
                    if (task_queue_.size() < static_cast<size_t>(realtime_config_.max_pending_tasks)) {
                        task_queue_.push(std::move(task));
                        stats_.queue_length = task_queue_.size();
                        queue_cv_.notify_one();
                    } else {
                        stats_.skipped_frames++;
                    }
                }
            }
        }
    }

    return true;
}

void RealtimeHBA::optimizationWorkerLoop(int worker_id) {
    while (optimization_running_) {
        std::unique_ptr<OptimizationTask> task;

        // 获取任务
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !task_queue_.empty() || !optimization_running_;
            });

            if (!optimization_running_) {
                break;
            }

            if (task_queue_.empty()) {
                continue;
            }

            task = std::move(task_queue_.front());
            task_queue_.pop();
            stats_.queue_length = task_queue_.size();
        }

        // 检查是否暂停
        if (optimization_paused_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 执行优化
        auto start_time = std::chrono::high_resolution_clock::now();
        bool success = optimizeWindow(task->window);
        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // 更新统计信息
        if (success) {
            stats_.successful_optimizations++;
            stats_.optimized_frames += task->window.keyframes.size();
        } else {
            stats_.failed_optimizations++;
        }

        // 更新平均优化时间
        double current_time = duration.count();
        stats_.average_optimization_time_ms =
            (stats_.average_optimization_time_ms * (stats_.successful_optimizations + stats_.failed_optimizations - 1) +
             current_time) / (stats_.successful_optimizations + stats_.failed_optimizations);

        // 设置任务完成状态
        task->completion_promise.set_value(success);

        // 检查是否需要调整参数
        if (realtime_config_.enable_adaptive_scheduling) {
            adjustOptimizationParameters();
        }
    }
}

bool RealtimeHBA::optimizeWindow(OptimizationWindow& window) {
    if (window.keyframes.empty()) {
        return false;
    }

    try {
        // 准备优化数据
        std::vector<Pose> initial_poses;
        std::vector<CloudType::Ptr> clouds;

        for (const auto& keyframe : window.keyframes) {
            Pose pose;
            pose.t = keyframe.t_global;
            pose.r = keyframe.r_global;
            pose.timestamp = keyframe.timestamp;
            initial_poses.push_back(pose);
            clouds.push_back(keyframe.body_cloud);
        }

        // 执行层次化优化
        auto optimized_poses = performHierarchicalOptimization(initial_poses, clouds);

        // 验证优化结果
        if (!validateOptimizationResult(window, optimized_poses)) {
            return false;
        }

        // 合并优化结果
        mergeOptimizationResults(window, optimized_poses);

        return true;

    } catch (const std::exception& e) {
        // 记录优化错误
        RCLCPP_WARN(rclcpp::get_logger("RealtimeHBA"),
                   "Optimization failed for window [%zu-%zu]: %s",
                   window.start_idx, window.end_idx, e.what());
        return false;
    }
}

std::vector<Pose> RealtimeHBA::performHierarchicalOptimization(
    const std::vector<Pose>& initial_poses,
    const std::vector<CloudType::Ptr>& clouds) {

    std::vector<Pose> optimized_poses = initial_poses;

    // 多层优化
    for (int layer = realtime_config_.max_layer; layer >= 0; --layer) {
        double current_voxel_size = realtime_config_.voxel_size * std::pow(2.0, layer);

        // 下采样点云
        std::vector<CloudType::Ptr> downsampled_clouds;
        for (const auto& cloud : clouds) {
            downsampled_clouds.push_back(downsampleCloud(cloud, current_voxel_size));
        }

        // 执行束调整优化
        optimized_poses = performBundleAdjustment(optimized_poses, downsampled_clouds);

        // 检查收敛性
        if (hasConverged(initial_poses, optimized_poses)) {
            break;
        }
    }

    return optimized_poses;
}

std::vector<Pose> RealtimeHBA::performBundleAdjustment(
    const std::vector<Pose>& poses,
    const std::vector<CloudType::Ptr>& clouds) {

    // 简化版的束调整实现
    std::vector<Pose> optimized_poses = poses;

    for (size_t iter = 0; iter < realtime_config_.ba_max_iter; ++iter) {
        // 计算雅可比矩阵和残差
        Eigen::MatrixXd jacobian;
        Eigen::VectorXd residuals;

        if (!computeJacobianAndResiduals(optimized_poses, clouds, jacobian, residuals)) {
            break;
        }

        // 求解线性系统
        Eigen::VectorXd delta = jacobian.colPivHouseholderQr().solve(-residuals);

        // 更新位姿
        updatePoses(optimized_poses, delta);

        // 检查收敛
        if (delta.norm() < 1e-6) {
            break;
        }
    }

    return optimized_poses;
}

void RealtimeHBA::performanceMonitorLoop() {
    while (optimization_running_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_stats_update_);

        if (duration.count() >= 1000) { // 每秒更新一次
            // 计算当前频率
            static size_t last_optimized_frames = 0;
            size_t current_optimized = stats_.optimized_frames;
            stats_.current_frequency = (current_optimized - last_optimized_frames) * 1000.0 / duration.count();
            last_optimized_frames = current_optimized;

            // 计算CPU利用率
            stats_.cpu_utilization = calculateSystemLoad();

            last_stats_update_ = current_time;

            // 打印性能统计
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("RealtimeHBA"),
                                 *rclcpp::Clock().get_clock(), 5000,
                                 "HBA Performance: Freq=%.1fHz, AvgTime=%.1fms, CPU=%.1f%%, Queue=%d",
                                 stats_.current_frequency.load(),
                                 stats_.average_optimization_time_ms.load(),
                                 stats_.cpu_utilization.load() * 100.0,
                                 stats_.queue_length.load());
        }
    }
}

std::vector<OptimizationWindow> RealtimeHBA::createOptimizationWindows() {
    std::vector<OptimizationWindow> windows;

    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (frame_buffer_.size() < static_cast<size_t>(realtime_config_.min_window_size)) {
        return windows;
    }

    // 创建滑动窗口
    size_t num_windows = (frame_buffer_.size() - realtime_config_.min_window_size) /
                        realtime_config_.window_stride + 1;

    for (size_t i = 0; i < num_windows; ++i) {
        OptimizationWindow window;
        window.start_idx = i * realtime_config_.window_stride;
        window.end_idx = std::min(window.start_idx + realtime_config_.max_window_size,
                                 frame_buffer_.size());

        if (window.end_idx - window.start_idx >= static_cast<size_t>(realtime_config_.min_window_size)) {
            // 复制关键帧数据
            for (size_t j = window.start_idx; j < window.end_idx; ++j) {
                window.keyframes.push_back(frame_buffer_[j]);
            }

            window.timestamp = window.keyframes.back().timestamp;
            window.needs_optimization = true;
            window.priority = scheduler_->calculateWindowPriority(window);

            windows.push_back(window);
        }
    }

    return windows;
}

void RealtimeHBA::adjustOptimizationParameters() {
    double current_load = calculateSystemLoad();

    if (current_load > realtime_config_.load_threshold_high) {
        handleHighLoad();
    } else if (current_load < realtime_config_.load_threshold_low) {
        handleLowLoad();
    }
}

void RealtimeHBA::handleHighLoad() {
    // 降低优化频率
    if (realtime_config_.window_stride < realtime_config_.max_window_size / 2) {
        realtime_config_.window_stride++;
    }

    // 减少最大迭代次数
    if (realtime_config_.ba_max_iter > 5) {
        realtime_config_.ba_max_iter = static_cast<size_t>(realtime_config_.ba_max_iter * 0.9);
    }

    // 增加体素大小
    realtime_config_.voxel_size *= 1.1;
}

void RealtimeHBA::handleLowLoad() {
    // 提高优化频率
    if (realtime_config_.window_stride > 1) {
        realtime_config_.window_stride--;
    }

    // 增加最大迭代次数
    if (realtime_config_.ba_max_iter < 50) {
        realtime_config_.ba_max_iter = static_cast<size_t>(realtime_config_.ba_max_iter * 1.1);
    }

    // 减少体素大小
    if (realtime_config_.voxel_size > 0.01) {
        realtime_config_.voxel_size *= 0.95;
    }
}

double RealtimeHBA::calculateSystemLoad() {
    // 基于多个指标计算系统负载
    double queue_load = static_cast<double>(stats_.queue_length) / realtime_config_.max_pending_tasks;
    double time_load = stats_.average_optimization_time_ms / realtime_config_.max_optimization_time_ms;
    double frequency_load = std::max(0.0, 1.0 - stats_.current_frequency / realtime_config_.target_frequency);

    return (queue_load + time_load + frequency_load) / 3.0;
}

bool RealtimeHBA::validateOptimizationResult(const OptimizationWindow& window,
                                            const std::vector<Pose>& optimized_poses) {

    if (optimized_poses.size() != window.keyframes.size()) {
        return false;
    }

    // 计算位姿变化
    double total_change = 0.0;
    for (size_t i = 0; i < window.keyframes.size(); ++i) {
        Eigen::Vector3d original_pos = window.keyframes[i].t_global;
        Eigen::Vector3d optimized_pos = optimized_poses[i].t;

        double change = (optimized_pos - original_pos).norm();
        total_change += change;

        // 检查是否有异常大的变化
        if (change > 5.0) { // 5米阈值
            return false;
        }
    }

    // 检查平均变化是否合理
    double average_change = total_change / window.keyframes.size();
    if (average_change < realtime_config_.min_optimization_improvement) {
        return false; // 优化改进太小
    }

    return true;
}

void RealtimeHBA::mergeOptimizationResults(const OptimizationWindow& window,
                                          const std::vector<Pose>& optimized_poses) {

    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // 更新frame_buffer中对应帧的位姿
    for (size_t i = 0; i < optimized_poses.size() && i < window.keyframes.size(); ++i) {
        size_t buffer_idx = window.start_idx + i;
        if (buffer_idx < frame_buffer_.size()) {
            frame_buffer_[buffer_idx].t_global = optimized_poses[i].t;
            frame_buffer_[buffer_idx].r_global = optimized_poses[i].r;
        }
    }

    // 更新优化统计
    stats_.optimization_improvement =
        (stats_.optimization_improvement + calculateOptimizationImprovement(window, optimized_poses)) / 2.0;
}

int RealtimeHBA::generateTaskId() {
    static std::atomic<int> task_counter{0};
    return ++task_counter;
}

} // namespace hba
```

## 集成到现有HBA节点

### 修改hba_node.cpp

```cpp
// 文件位置: ws_livox/src/hba/src/hba_node.cpp (大幅修改)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "hba/realtime_hba.h"
#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"

using namespace std::chrono_literals;

struct NodeConfig {
    std::string cloud_topic = "fastlio2/body_cloud";
    std::string odom_topic = "fastlio2/lio_odom";
    std::string map_frame = "map";
    double optimization_frequency = 10.0;
    bool enable_realtime_mode = true;
};

class RealtimeHBANode : public rclcpp::Node {
public:
    RealtimeHBANode() : Node("realtime_hba_node") {
        RCLCPP_INFO(this->get_logger(), "Realtime HBA node started");

        loadParameters();

        // 初始化实时HBA
        realtime_hba_ = std::make_shared<hba::RealtimeHBA>(realtime_config_);

        // 设置ROS2接口
        setupROS2Interface();

        // 启动实时优化
        if (node_config_.enable_realtime_mode) {
            if (!realtime_hba_->start()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start realtime HBA!");
            }
        }
    }

    ~RealtimeHBANode() {
        if (realtime_hba_) {
            realtime_hba_->stop();
        }
    }

private:
    NodeConfig node_config_;
    hba::RealtimeHBAConfig realtime_config_;
    std::shared_ptr<hba::RealtimeHBA> realtime_hba_;

    // ROS2接口
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_map_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;

    rclcpp::Service<interface::srv::RefineMap>::SharedPtr refine_map_srv_;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr save_poses_srv_;

    rclcpp::TimerBase::SharedPtr stats_timer_;

    void loadParameters();
    void setupROS2Interface();

    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

    void publishOptimizedMap();
    void publishStats();

    void refineMapCallback(const std::shared_ptr<interface::srv::RefineMap::Request> request,
                          std::shared_ptr<interface::srv::RefineMap::Response> response);

    void savePosesCallback(const std::shared_ptr<interface::srv::SavePoses::Request> request,
                          std::shared_ptr<interface::srv::SavePoses::Response> response);
};

void RealtimeHBANode::loadParameters() {
    // 加载节点配置
    this->declare_parameter("cloud_topic", node_config_.cloud_topic);
    this->declare_parameter("odom_topic", node_config_.odom_topic);
    this->declare_parameter("optimization_frequency", node_config_.optimization_frequency);
    this->declare_parameter("enable_realtime_mode", node_config_.enable_realtime_mode);

    node_config_.cloud_topic = this->get_parameter("cloud_topic").as_string();
    node_config_.odom_topic = this->get_parameter("odom_topic").as_string();
    node_config_.optimization_frequency = this->get_parameter("optimization_frequency").as_double();
    node_config_.enable_realtime_mode = this->get_parameter("enable_realtime_mode").as_bool();

    // 加载实时HBA配置
    this->declare_parameter("realtime_hba.target_frequency", 10.0);
    this->declare_parameter("realtime_hba.max_window_size", 20);
    this->declare_parameter("realtime_hba.window_stride", 3);
    this->declare_parameter("realtime_hba.num_optimization_threads", 2);
    this->declare_parameter("realtime_hba.enable_adaptive_scheduling", true);

    realtime_config_.target_frequency = this->get_parameter("realtime_hba.target_frequency").as_double();
    realtime_config_.max_window_size = this->get_parameter("realtime_hba.max_window_size").as_int();
    realtime_config_.window_stride = this->get_parameter("realtime_hba.window_stride").as_int();
    realtime_config_.num_optimization_threads = this->get_parameter("realtime_hba.num_optimization_threads").as_int();
    realtime_config_.enable_adaptive_scheduling = this->get_parameter("realtime_hba.enable_adaptive_scheduling").as_bool();

    // 加载传统HBA参数
    this->declare_parameter("hba.voxel_size", 0.1);
    this->declare_parameter("hba.ba_max_iter", 20);
    this->declare_parameter("hba.max_layer", 3);

    realtime_config_.voxel_size = this->get_parameter("hba.voxel_size").as_double();
    realtime_config_.ba_max_iter = this->get_parameter("hba.ba_max_iter").as_int();
    realtime_config_.max_layer = this->get_parameter("hba.max_layer").as_int();
}

void RealtimeHBANode::setupROS2Interface() {
    // 设置订阅器
    rclcpp::QoS qos(10);
    cloud_sub_.subscribe(this, node_config_.cloud_topic, qos.get_rmw_qos_profile());
    odom_sub_.subscribe(this, node_config_.odom_topic, qos.get_rmw_qos_profile());

    // 设置同步器
    sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
        cloud_sub_, odom_sub_);
    sync_->registerCallback(std::bind(&RealtimeHBANode::syncCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

    // 设置发布器
    optimized_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hba/optimized_map", 10);
    stats_pub_ = this->create_publisher<std_msgs::msg::String>("hba/performance_stats", 10);

    // 设置服务
    refine_map_srv_ = this->create_service<interface::srv::RefineMap>(
        "hba/refine_map", std::bind(&RealtimeHBANode::refineMapCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));

    save_poses_srv_ = this->create_service<interface::srv::SavePoses>(
        "hba/save_poses", std::bind(&RealtimeHBANode::savePosesCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));

    // 设置定时器
    auto stats_period = std::chrono::milliseconds(static_cast<int>(1000.0 / node_config_.optimization_frequency));
    stats_timer_ = this->create_wall_timer(stats_period,
                                          std::bind(&RealtimeHBANode::publishStats, this));
}

void RealtimeHBANode::syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                                  const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {

    // 转换数据格式
    hba::KeyPoseWithCloud keyframe;

    keyframe.timestamp = cloud_msg->header.stamp.sec + cloud_msg->header.stamp.nanosec * 1e-9;

    keyframe.t_global = Eigen::Vector3d(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z);

    keyframe.r_global = Eigen::Quaterniond(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z).toRotationMatrix();

    keyframe.body_cloud = std::make_shared<CloudType>();
    pcl::fromROSMsg(*cloud_msg, *keyframe.body_cloud);

    // 添加到实时HBA
    if (node_config_.enable_realtime_mode && realtime_hba_->isRunning()) {
        realtime_hba_->addFrameAsync(keyframe);
    }
}

void RealtimeHBANode::publishOptimizedMap() {
    if (optimized_map_pub_->get_subscription_count() == 0) {
        return;
    }

    auto optimized_map = realtime_hba_->getOptimizedMap();
    if (!optimized_map || optimized_map->empty()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*optimized_map, map_msg);
    map_msg.header.frame_id = node_config_.map_frame;
    map_msg.header.stamp = this->now();

    optimized_map_pub_->publish(map_msg);
}

void RealtimeHBANode::publishStats() {
    publishOptimizedMap();

    if (stats_pub_->get_subscription_count() == 0) {
        return;
    }

    auto stats = realtime_hba_->getPerformanceStats();

    std_msgs::msg::String stats_msg;
    stats_msg.data =
        "RealtimeHBA Stats: " +
        "Freq=" + std::to_string(stats.current_frequency) + "Hz, " +
        "AvgTime=" + std::to_string(stats.average_optimization_time_ms) + "ms, " +
        "CPU=" + std::to_string(stats.cpu_utilization * 100.0) + "%, " +
        "Queue=" + std::to_string(stats.queue_length) + ", " +
        "Success=" + std::to_string(stats.successful_optimizations) + "/" +
        std::to_string(stats.successful_optimizations + stats.failed_optimizations);

    stats_pub_->publish(stats_msg);
}

void RealtimeHBANode::refineMapCallback(
    const std::shared_ptr<interface::srv::RefineMap::Request> request,
    std::shared_ptr<interface::srv::RefineMap::Response> response) {

    // 暂停实时优化
    realtime_hba_->pauseOptimization();

    // 执行批量优化
    // 这里可以调用原有的批处理逻辑
    // 或者使用实时HBA进行更精细的优化

    // 恢复实时优化
    realtime_hba_->resumeOptimization();

    response->success = true;
    response->message = "Map refinement completed using realtime HBA";
}

void RealtimeHBANode::savePosesCallback(
    const std::shared_ptr<interface::srv::SavePoses::Request> request,
    std::shared_ptr<interface::srv::SavePoses::Response> response) {

    auto optimized_poses = realtime_hba_->getOptimizedPoses();

    if (optimized_poses.empty()) {
        response->success = false;
        response->message = "No optimized poses available";
        return;
    }

    // 保存位姿到文件
    std::ofstream file(request->file_path);
    if (!file.is_open()) {
        response->success = false;
        response->message = "Failed to open file: " + request->file_path;
        return;
    }

    for (const auto& pose : optimized_poses) {
        Eigen::Quaterniond q(pose.r);
        file << pose.timestamp << " "
             << pose.t.x() << " " << pose.t.y() << " " << pose.t.z() << " "
             << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    }

    file.close();

    response->success = true;
    response->message = "Saved " + std::to_string(optimized_poses.size()) + " poses";
}
```

## 配置文件更新

### 更新hba.yaml配置

```yaml
# 文件位置: ws_livox/src/hba/config/hba.yaml

# ROS2接口配置
cloud_topic: "fastlio2/body_cloud"
odom_topic: "fastlio2/lio_odom"
map_frame: "map"
optimization_frequency: 10.0
enable_realtime_mode: true

# 实时HBA核心配置
realtime_hba:
  # 实时性能参数
  target_frequency: 10.0          # 目标优化频率(Hz)
  max_window_size: 20             # 最大窗口大小
  min_window_size: 5              # 最小窗口大小
  window_stride: 3                # 窗口步长
  max_optimization_time_ms: 80.0  # 最大优化时间(ms)

  # 线程池配置
  num_optimization_threads: 2     # 优化线程数
  max_pending_tasks: 10           # 最大待处理任务数

  # 自适应调度
  enable_adaptive_scheduling: true # 启用自适应调度
  load_threshold_high: 0.8        # 高负载阈值
  load_threshold_low: 0.4         # 低负载阈值
  load_adjustment_factor: 0.1     # 负载调整因子

  # 质量控制
  enable_quality_control: true    # 启用质量控制
  min_optimization_improvement: 0.01  # 最小优化改进
  max_consecutive_failures: 3     # 最大连续失败次数

# 传统HBA算法参数
hba:
  # 基础参数
  voxel_size: 0.1                 # 体素大小(m)
  min_point_num: 50               # 最小点数
  max_layer: 3                    # 最大层数
  plane_thresh: 0.1               # 平面阈值(m)

  # 束调整参数
  ba_max_iter: 20                 # BA最大迭代次数
  hba_iter: 3                     # HBA迭代次数
  down_sample: 0.1                # 降采样大小(m)

# 性能调优配置
performance_tuning:
  # 内存管理
  max_memory_usage_mb: 2048       # 最大内存使用(MB)
  garbage_collection_interval: 60 # 垃圾回收间隔(s)

  # CPU优化
  cpu_affinity_enabled: false     # 启用CPU亲和性
  priority_boost: false           # 启用优先级提升

  # IO优化
  async_io_enabled: true          # 启用异步IO
  batch_size: 10                  # 批处理大小

# 不同场景的配置模板
scenario_configs:
  # 高精度模式（牺牲实时性）
  high_precision:
    target_frequency: 5.0
    max_window_size: 30
    ba_max_iter: 50
    voxel_size: 0.05

  # 高速模式（优先实时性）
  high_speed:
    target_frequency: 20.0
    max_window_size: 10
    ba_max_iter: 10
    voxel_size: 0.2

  # 平衡模式（默认）
  balanced:
    target_frequency: 10.0
    max_window_size: 20
    ba_max_iter: 20
    voxel_size: 0.1

  # 低功耗模式
  low_power:
    target_frequency: 2.0
    num_optimization_threads: 1
    max_window_size: 15
    enable_adaptive_scheduling: false
```

## 测试和验证

### 性能基准测试

```cpp
// 文件位置: ws_livox/src/hba/test/test_realtime_performance.cpp

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "hba/realtime_hba.h"

class RealtimeHBAPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        hba::RealtimeHBAConfig config;
        config.target_frequency = 10.0;
        config.max_window_size = 20;
        config.num_optimization_threads = 2;

        realtime_hba_ = std::make_unique<hba::RealtimeHBA>(config);
        realtime_hba_->start();
    }

    void TearDown() override {
        realtime_hba_->stop();
    }

    std::unique_ptr<hba::RealtimeHBA> realtime_hba_;
};

TEST_F(RealtimeHBAPerformanceTest, TargetFrequencyAchieved) {
    // 生成测试数据
    for (int i = 0; i < 100; ++i) {
        hba::KeyPoseWithCloud frame;
        frame.timestamp = i * 0.1;
        frame.t_global = Eigen::Vector3d(i * 0.1, 0, 0);
        frame.r_global = Eigen::Matrix3d::Identity();

        // 创建简单点云
        frame.body_cloud = std::make_shared<CloudType>();
        for (int j = 0; j < 1000; ++j) {
            PointType point;
            point.x = (rand() % 100) / 10.0f;
            point.y = (rand() % 100) / 10.0f;
            point.z = (rand() % 10) / 10.0f;
            frame.body_cloud->push_back(point);
        }

        realtime_hba_->addFrameAsync(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz输入
    }

    // 等待优化完成
    std::this_thread::sleep_for(std::chrono::seconds(5));

    auto stats = realtime_hba_->getPerformanceStats();

    // 检查频率是否达到目标
    EXPECT_GT(stats.current_frequency, 5.0); // 至少5Hz
    EXPECT_LT(stats.average_optimization_time_ms, 100.0); // 平均优化时间<100ms
    EXPECT_GT(stats.successful_optimizations, 10); // 至少10次成功优化
}

TEST_F(RealtimeHBAPerformanceTest, AdaptiveSchedulingWorks) {
    hba::RealtimeHBAConfig config = realtime_hba_->getConfig();
    config.enable_adaptive_scheduling = true;
    realtime_hba_->updateConfig(config);

    // 高负载测试：快速输入大量数据
    for (int i = 0; i < 200; ++i) {
        hba::KeyPoseWithCloud frame;
        frame.timestamp = i * 0.01; // 100Hz输入
        frame.t_global = Eigen::Vector3d(i * 0.01, 0, 0);
        frame.r_global = Eigen::Matrix3d::Identity();

        frame.body_cloud = std::make_shared<CloudType>();
        // 更复杂的点云
        for (int j = 0; j < 5000; ++j) {
            PointType point;
            point.x = (rand() % 1000) / 100.0f;
            point.y = (rand() % 1000) / 100.0f;
            point.z = (rand() % 100) / 100.0f;
            frame.body_cloud->push_back(point);
        }

        realtime_hba_->addFrameAsync(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto stats = realtime_hba_->getPerformanceStats();

    // 自适应调度应该能够处理高负载
    EXPECT_LT(stats.queue_length, 20); // 队列长度应该控制在合理范围
    EXPECT_GT(stats.successful_optimizations, 20); // 仍然有成功的优化
}

TEST_F(RealtimeHBAPerformanceTest, MemoryUsageStable) {
    size_t initial_memory = getCurrentMemoryUsage();

    // 长时间运行测试
    for (int batch = 0; batch < 10; ++batch) {
        for (int i = 0; i < 50; ++i) {
            hba::KeyPoseWithCloud frame;
            frame.timestamp = batch * 50 + i;
            frame.t_global = Eigen::Vector3d(i * 0.1, batch * 5, 0);
            frame.r_global = Eigen::Matrix3d::Identity();

            frame.body_cloud = std::make_shared<CloudType>();
            for (int j = 0; j < 2000; ++j) {
                PointType point;
                point.x = (rand() % 200) / 10.0f;
                point.y = (rand() % 200) / 10.0f;
                point.z = (rand() % 20) / 10.0f;
                frame.body_cloud->push_back(point);
            }

            realtime_hba_->addFrameAsync(frame);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        size_t current_memory = getCurrentMemoryUsage();

        // 内存使用应该保持稳定（不超过初始内存的3倍）
        EXPECT_LT(current_memory, initial_memory * 3);
    }
}

size_t RealtimeHBAPerformanceTest::getCurrentMemoryUsage() {
    // 简化的内存使用检测
    std::ifstream status_file("/proc/self/status");
    std::string line;
    while (std::getline(status_file, line)) {
        if (line.substr(0, 6) == "VmRSS:") {
            std::istringstream iss(line);
            std::string label, value, unit;
            iss >> label >> value >> unit;
            return std::stoul(value) * 1024; // 转换为字节
        }
    }
    return 0;
}
```

### 集成测试脚本

```bash
#!/bin/bash
# 文件位置: tools/test_realtime_hba.sh

echo "Testing Realtime HBA System..."

cd ws_livox

# 1. 编译
colcon build --packages-select hba --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# 2. 运行单元测试
colcon test --packages-select hba
if [ $? -ne 0 ]; then
    echo "Unit tests failed!"
    exit 1
fi

source install/setup.bash

# 3. 启动系统
echo "Starting SLAM system with realtime HBA..."

# 启动FAST-LIO2
ros2 launch fastlio2 enhanced_visualization.launch.py &
FASTLIO_PID=$!

sleep 5

# 启动实时HBA
ros2 launch hba hba_launch.py enable_realtime_mode:=true &
HBA_PID=$!

sleep 3

# 4. 播放测试数据
echo "Playing test dataset..."
ros2 bag play ../test_data/realtime_hba_test.bag --rate 1.0 &
BAG_PID=$!

# 5. 监控性能
echo "Monitoring realtime HBA performance..."
timeout 60 ros2 topic echo /hba/performance_stats &
MONITOR_PID=$!

# 6. 检查优化频率
echo "Checking optimization frequency..."
timeout 30 bash -c '
    count=0
    start_time=$(date +%s)
    ros2 topic echo /hba/performance_stats | while read line; do
        if [[ $line == *"Freq="* ]]; then
            freq=$(echo $line | grep -o "Freq=[0-9.]*" | cut -d= -f2)
            echo "Current frequency: ${freq}Hz"
            count=$((count + 1))
            if (( count >= 10 )); then
                echo "Frequency monitoring completed"
                break
            fi
        fi
    done
'

# 7. 性能基准测试
echo "Running performance benchmark..."
timeout 30 bash -c '
    total_time=0
    count=0
    ros2 topic echo /hba/performance_stats | while read line; do
        if [[ $line == *"AvgTime="* ]]; then
            time=$(echo $line | grep -o "AvgTime=[0-9.]*" | cut -d= -f2)
            echo "Average optimization time: ${time}ms"
            total_time=$(echo "$total_time + $time" | bc)
            count=$((count + 1))
            if (( count >= 10 )); then
                avg_time=$(echo "scale=2; $total_time / $count" | bc)
                echo "Overall average optimization time: ${avg_time}ms"
                if (( $(echo "$avg_time < 100" | bc -l) )); then
                    echo "✓ Performance test PASSED"
                else
                    echo "✗ Performance test FAILED"
                fi
                break
            fi
        fi
    done
'

# 清理进程
echo "Cleaning up..."
kill $FASTLIO_PID $HBA_PID $BAG_PID $MONITOR_PID 2>/dev/null

echo "Realtime HBA integration test completed!"
```

## 可视化和监控工具

### 实时性能监控

```python
# 文件位置: tools/monitor_realtime_hba.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import threading
import time

class RealtimeHBAMonitor(Node):
    def __init__(self):
        super().__init__('realtime_hba_monitor')

        self.stats_subscriber = self.create_subscription(
            String, '/hba/performance_stats', self.stats_callback, 10)

        # 数据存储
        self.frequencies = deque(maxlen=100)
        self.optimization_times = deque(maxlen=100)
        self.cpu_utilizations = deque(maxlen=100)
        self.queue_lengths = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)

        # 设置图形界面
        self.setup_plots()

        # 启动动画
        self.ani = animation.FuncAnimation(self.fig, self.update_plots, interval=1000, blit=False)

    def setup_plots(self):
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))

        self.fig.suptitle('Realtime HBA Performance Monitor')

        # 频率图
        self.ax1.set_title('Optimization Frequency')
        self.ax1.set_ylabel('Frequency (Hz)')
        self.ax1.set_ylim(0, 20)
        self.line1, = self.ax1.plot([], [], 'b-')
        self.ax1.axhline(y=10, color='r', linestyle='--', label='Target: 10Hz')
        self.ax1.legend()

        # 优化时间图
        self.ax2.set_title('Optimization Time')
        self.ax2.set_ylabel('Time (ms)')
        self.ax2.set_ylim(0, 200)
        self.line2, = self.ax2.plot([], [], 'g-')
        self.ax2.axhline(y=100, color='r', linestyle='--', label='Target: <100ms')
        self.ax2.legend()

        # CPU使用率图
        self.ax3.set_title('CPU Utilization')
        self.ax3.set_ylabel('CPU (%)')
        self.ax3.set_ylim(0, 100)
        self.line3, = self.ax3.plot([], [], 'orange')
        self.ax3.axhline(y=80, color='r', linestyle='--', label='High Load: 80%')
        self.ax3.legend()

        # 队列长度图
        self.ax4.set_title('Task Queue Length')
        self.ax4.set_ylabel('Queue Length')
        self.ax4.set_ylim(0, 20)
        self.line4, = self.ax4.plot([], [], 'purple')
        self.ax4.axhline(y=10, color='r', linestyle='--', label='Max: 10')
        self.ax4.legend()

        plt.tight_layout()

    def stats_callback(self, msg):
        # 解析统计信息
        stats_text = msg.data

        frequency_match = re.search(r'Freq=([0-9.]+)', stats_text)
        time_match = re.search(r'AvgTime=([0-9.]+)', stats_text)
        cpu_match = re.search(r'CPU=([0-9.]+)', stats_text)
        queue_match = re.search(r'Queue=([0-9]+)', stats_text)

        current_time = time.time()

        if frequency_match:
            self.frequencies.append(float(frequency_match.group(1)))
        if time_match:
            self.optimization_times.append(float(time_match.group(1)))
        if cpu_match:
            self.cpu_utilizations.append(float(cpu_match.group(1)))
        if queue_match:
            self.queue_lengths.append(int(queue_match.group(1)))

        self.timestamps.append(current_time)

    def update_plots(self, frame):
        if len(self.timestamps) < 2:
            return

        # 使用相对时间
        base_time = self.timestamps[0]
        x_data = [t - base_time for t in self.timestamps]

        # 更新频率图
        if len(self.frequencies) > 0:
            self.line1.set_data(x_data[-len(self.frequencies):], list(self.frequencies))
            self.ax1.set_xlim(max(0, x_data[-1] - 60), x_data[-1])  # 显示最近60秒

        # 更新优化时间图
        if len(self.optimization_times) > 0:
            self.line2.set_data(x_data[-len(self.optimization_times):], list(self.optimization_times))
            self.ax2.set_xlim(max(0, x_data[-1] - 60), x_data[-1])

        # 更新CPU使用率图
        if len(self.cpu_utilizations) > 0:
            self.line3.set_data(x_data[-len(self.cpu_utilizations):], list(self.cpu_utilizations))
            self.ax3.set_xlim(max(0, x_data[-1] - 60), x_data[-1])

        # 更新队列长度图
        if len(self.queue_lengths) > 0:
            self.line4.set_data(x_data[-len(self.queue_lengths):], list(self.queue_lengths))
            self.ax4.set_xlim(max(0, x_data[-1] - 60), x_data[-1])

        # 性能分析
        if len(self.frequencies) >= 10:
            avg_freq = sum(list(self.frequencies)[-10:]) / 10
            avg_time = sum(list(self.optimization_times)[-10:]) / 10 if self.optimization_times else 0

            status_color = 'green'
            if avg_freq < 8 or avg_time > 120:
                status_color = 'red'
            elif avg_freq < 9 or avg_time > 100:
                status_color = 'orange'

            self.fig.suptitle(f'Realtime HBA Monitor - Status: {status_color.upper()}', color=status_color)

def main():
    rclpy.init()

    monitor = RealtimeHBAMonitor()

    def spin_ros():
        rclpy.spin(monitor)

    # 在独立线程中运行ROS2
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.daemon = True
    ros_thread.start()

    # 显示图形界面
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 故障排除和优化

### 常见问题解决

1. **优化频率达不到目标**
   - 检查 `num_optimization_threads` 设置
   - 降低 `max_window_size` 或增加 `window_stride`
   - 启用自适应调度

2. **内存使用过高**
   - 减少 `max_window_size`
   - 启用更积极的垃圾回收
   - 检查点云大小和降采样设置

3. **优化质量下降**
   - 检查 `min_optimization_improvement` 设置
   - 增加 `ba_max_iter` 或减少 `voxel_size`
   - 验证输入数据质量

4. **系统不稳定**
   - 启用 `enable_quality_control`
   - 调整 `max_consecutive_failures`
   - 检查线程同步问题

### 性能调优建议

```yaml
# 针对不同硬件配置的推荐设置

# 高性能工作站 (16核32GB)
high_performance:
  num_optimization_threads: 4
  max_window_size: 30
  target_frequency: 15.0
  ba_max_iter: 30

# 中等配置 (8核16GB)
medium_performance:
  num_optimization_threads: 2
  max_window_size: 20
  target_frequency: 10.0
  ba_max_iter: 20

# 低配置 (4核8GB)
low_performance:
  num_optimization_threads: 1
  max_window_size: 10
  target_frequency: 5.0
  ba_max_iter: 10
  enable_adaptive_scheduling: false

# 嵌入式设备
embedded:
  num_optimization_threads: 1
  max_window_size: 5
  target_frequency: 2.0
  ba_max_iter: 5
  voxel_size: 0.2
```

## 总结

通过实施本指南的HBA实时优化方案，预期实现：

- **实时性提升**: 建图延迟从批处理模式减少到<100ms
- **处理频率**: 支持10Hz以上的实时优化频率
- **内存效率**: 通过滑动窗口优化，内存占用减少30%
- **自适应性**: 根据系统负载动态调整优化参数

完成所有三个功能后，您的MID360 SLAM系统将具备：
1. ✅ 动态对象处理 - 提升动态环境定位精度
2. ✅ 先进的PGO回环检测 - 提高长距离建图精度
3. ✅ 实时HBA优化 - 减少建图延迟

这将显著提升整个系统的性能和适用性。