# 测试与验证指南

## 概述

本指南提供三大功能增强的全面测试与验证策略，确保动态对象处理、PGO回环检测和HBA实时优化功能的正确性、性能和可靠性。

## 测试架构

### 测试分类

```
测试金字塔结构:
┌─────────────────────────────────────┐
│           E2E Tests (少量)            │  ← 端到端系统测试
├─────────────────────────────────────┤
│        Integration Tests (中等)       │  ← 组件集成测试
├─────────────────────────────────────┤
│          Unit Tests (大量)           │  ← 单元测试
└─────────────────────────────────────┘
```

### 测试环境

1. **开发环境**: 单元测试和快速验证
2. **集成环境**: 组件集成和系统测试
3. **性能环境**: 性能基准和压力测试
4. **生产模拟**: 真实场景验证

## 单元测试

### 动态对象过滤器测试

```cpp
// 文件位置: ws_livox/src/localizer/test/test_dynamic_filter_comprehensive.cpp

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "localizers/dynamic_object_filter.h"
#include <pcl/io/pcd_io.h>
#include <random>

class DynamicFilterComprehensiveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 标准配置
        config_.motion_threshold = 0.1f;
        config_.history_size = 5;
        config_.stability_threshold = 0.8f;
        config_.search_radius = 0.2f;

        filter_ = std::make_unique<localizers::DynamicObjectFilter>(config_);

        // 设置随机种子确保可重复性
        rng_.seed(42);
    }

    localizers::DynamicFilterConfig config_;
    std::unique_ptr<localizers::DynamicObjectFilter> filter_;
    std::mt19937 rng_;

    // 测试辅助函数
    CloudType::Ptr createStaticScene(int num_points = 1000);
    CloudType::Ptr createDynamicScene(int frame_id, int num_static = 500, int num_dynamic = 100);
    CloudType::Ptr addNoise(const CloudType::Ptr& cloud, double noise_level = 0.01);
    void validateFilterRatio(double expected_ratio, double tolerance = 0.1);
};

CloudType::Ptr DynamicFilterComprehensiveTest::createStaticScene(int num_points) {
    CloudType::Ptr cloud(new CloudType);

    std::uniform_real_distribution<float> x_dist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> y_dist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> z_dist(0.0f, 3.0f);

    for (int i = 0; i < num_points; ++i) {
        PointType point;
        point.x = x_dist(rng_);
        point.y = y_dist(rng_);
        point.z = z_dist(rng_);
        point.intensity = 100.0f;
        cloud->push_back(point);
    }

    return cloud;
}

CloudType::Ptr DynamicFilterComprehensiveTest::createDynamicScene(
    int frame_id, int num_static, int num_dynamic) {

    CloudType::Ptr cloud(new CloudType);

    // 静态背景
    std::uniform_real_distribution<float> static_x(-5.0f, 5.0f);
    std::uniform_real_distribution<float> static_y(-5.0f, 5.0f);
    std::uniform_real_distribution<float> static_z(0.0f, 2.0f);

    for (int i = 0; i < num_static; ++i) {
        PointType point;
        point.x = static_x(rng_);
        point.y = static_y(rng_);
        point.z = static_z(rng_);
        point.intensity = 80.0f;
        cloud->push_back(point);
    }

    // 动态对象 (随时间移动)
    float dynamic_offset = frame_id * 0.2f; // 每帧移动0.2m
    std::uniform_real_distribution<float> dynamic_local(-0.5f, 0.5f);

    for (int i = 0; i < num_dynamic; ++i) {
        PointType point;
        point.x = 8.0f + dynamic_offset + dynamic_local(rng_);
        point.y = dynamic_local(rng_);
        point.z = 1.0f + dynamic_local(rng_) * 0.5f;
        point.intensity = 120.0f;
        cloud->push_back(point);
    }

    return cloud;
}

TEST_F(DynamicFilterComprehensiveTest, PureStaticScene) {
    // 测试纯静态场景
    CloudType::Ptr static_cloud = createStaticScene(1000);

    // 连续输入相同静态场景
    for (int frame = 0; frame < 10; ++frame) {
        auto filtered = filter_->filterDynamicObjects(static_cloud, frame * 0.1);

        if (frame >= config_.history_size) {
            // 有足够历史数据后，大部分点应该被保留
            double retention_ratio = static_cast<double>(filtered->size()) / static_cloud->size();
            EXPECT_GT(retention_ratio, 0.85) << "Frame " << frame << " retention too low";
        }
    }

    auto stats = filter_->getLastFilterStats();
    EXPECT_LT(stats.filter_ratio, 0.15); // 过滤率应该很低
}

TEST_F(DynamicFilterComprehensiveTest, MixedDynamicScene) {
    // 测试动静混合场景
    for (int frame = 0; frame < 15; ++frame) {
        CloudType::Ptr mixed_cloud = createDynamicScene(frame, 500, 100);
        auto filtered = filter_->filterDynamicObjects(mixed_cloud, frame * 0.1);

        if (frame >= config_.history_size) {
            auto stats = filter_->getLastFilterStats();

            // 应该过滤掉一些动态点
            EXPECT_GT(stats.filter_ratio, 0.05) << "Frame " << frame;
            EXPECT_LT(stats.filter_ratio, 0.5) << "Frame " << frame;

            // 大部分静态点应该被保留
            EXPECT_GT(filtered->size(), mixed_cloud->size() * 0.6) << "Frame " << frame;
        }
    }
}

TEST_F(DynamicFilterComprehensiveTest, NoiseRobustness) {
    // 测试噪声鲁棒性
    CloudType::Ptr base_cloud = createStaticScene(500);

    for (int frame = 0; frame < 8; ++frame) {
        CloudType::Ptr noisy_cloud = addNoise(base_cloud, 0.02); // 2cm噪声
        auto filtered = filter_->filterDynamicObjects(noisy_cloud, frame * 0.1);

        if (frame >= config_.history_size) {
            // 即使有噪声，大部分静态点应该被保留
            double retention_ratio = static_cast<double>(filtered->size()) / noisy_cloud->size();
            EXPECT_GT(retention_ratio, 0.7) << "Noise robustness failed at frame " << frame;
        }
    }
}

TEST_F(DynamicFilterComprehensiveTest, PerformanceBenchmark) {
    // 性能基准测试
    CloudType::Ptr large_cloud = createStaticScene(50000); // 大点云

    auto start = std::chrono::high_resolution_clock::now();
    auto filtered = filter_->filterDynamicObjects(large_cloud, 0.0);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_LT(duration.count(), 100) << "Processing too slow for large cloud";

    auto stats = filter_->getLastFilterStats();
    EXPECT_LT(stats.processing_time_ms, 100.0) << "Reported processing time inconsistent";
}

TEST_F(DynamicFilterComprehensiveTest, EdgeCases) {
    // 边界情况测试

    // 空点云
    CloudType::Ptr empty_cloud(new CloudType);
    auto filtered_empty = filter_->filterDynamicObjects(empty_cloud, 0.0);
    EXPECT_EQ(filtered_empty->size(), 0);

    // 单点云
    CloudType::Ptr single_point(new CloudType);
    PointType point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    single_point->push_back(point);

    auto filtered_single = filter_->filterDynamicObjects(single_point, 0.0);
    EXPECT_EQ(filtered_single->size(), 1); // 单点应该被保留

    // 巨大时间跳跃
    CloudType::Ptr normal_cloud = createStaticScene(100);
    filter_->filterDynamicObjects(normal_cloud, 0.0);
    auto filtered_jump = filter_->filterDynamicObjects(normal_cloud, 1000.0); // 1000秒后
    EXPECT_GT(filtered_jump->size(), normal_cloud->size() * 0.8); // 应该重置历史
}

// 参数化测试
class DynamicFilterParameterTest : public DynamicFilterComprehensiveTest,
                                  public ::testing::WithParamInterface<std::tuple<float, int, float>> {
protected:
    void SetUp() override {
        auto params = GetParam();
        config_.motion_threshold = std::get<0>(params);
        config_.history_size = std::get<1>(params);
        config_.stability_threshold = std::get<2>(params);

        filter_ = std::make_unique<localizers::DynamicObjectFilter>(config_);
        rng_.seed(42);
    }
};

TEST_P(DynamicFilterParameterTest, ParameterSensitivity) {
    // 测试不同参数组合的影响
    for (int frame = 0; frame < 10; ++frame) {
        CloudType::Ptr cloud = createDynamicScene(frame, 300, 50);
        auto filtered = filter_->filterDynamicObjects(cloud, frame * 0.1);

        if (frame >= config_.history_size) {
            auto stats = filter_->getLastFilterStats();

            // 基本性能要求
            EXPECT_LT(stats.processing_time_ms, 200.0);
            EXPECT_GT(filtered->size(), cloud->size() * 0.3);
            EXPECT_LT(filtered->size(), cloud->size() * 1.0);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    ParameterVariations,
    DynamicFilterParameterTest,
    ::testing::Values(
        std::make_tuple(0.05f, 3, 0.7f),  // 敏感设置
        std::make_tuple(0.1f, 5, 0.8f),   // 标准设置
        std::make_tuple(0.2f, 7, 0.9f)    // 保守设置
    )
);
```

### PGO回环检测测试

```cpp
// 文件位置: ws_livox/src/pgo/test/test_advanced_loop_detector_comprehensive.cpp

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "pgos/advanced_loop_detector.h"
#include <pcl/io/pcd_io.h>

class AdvancedLoopDetectorComprehensiveTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.candidate_search_radius = 10.0;
        config_.min_time_separation = 5.0;
        config_.min_confidence_score = 0.7;
        config_.feature_radius = 0.5;

        detector_ = std::make_unique<pgos::AdvancedLoopDetector>(config_);
    }

    pgos::AdvancedLoopConfig config_;
    std::unique_ptr<pgos::AdvancedLoopDetector> detector_;

    // 测试辅助函数
    std::vector<pgos::KeyPoseWithCloud> createLinearTrajectory(int num_poses, double spacing = 1.0);
    std::vector<pgos::KeyPoseWithCloud> createLoopTrajectory(int num_poses, double radius = 10.0);
    pgos::KeyPoseWithCloud createSimilarFrame(const pgos::KeyPoseWithCloud& reference, double noise_level = 0.1);
};

std::vector<pgos::KeyPoseWithCloud> AdvancedLoopDetectorComprehensiveTest::createLinearTrajectory(
    int num_poses, double spacing) {

    std::vector<pgos::KeyPoseWithCloud> trajectory;

    for (int i = 0; i < num_poses; ++i) {
        pgos::KeyPoseWithCloud pose;
        pose.t_global = Eigen::Vector3d(i * spacing, 0, 0);
        pose.r_global = Eigen::Matrix3d::Identity();
        pose.timestamp = i * 1.0;

        // 创建特征丰富的点云
        pose.body_cloud = std::make_shared<CloudType>();
        for (int j = 0; j < 500; ++j) {
            PointType point;
            point.x = (j % 20) * 0.1f - 1.0f;
            point.y = (j / 20) * 0.1f - 1.0f;
            point.z = std::sin(j * 0.1f) * 0.5f;
            point.intensity = 100.0f + j % 50;
            pose.body_cloud->push_back(point);
        }

        trajectory.push_back(pose);
    }

    return trajectory;
}

std::vector<pgos::KeyPoseWithCloud> AdvancedLoopDetectorComprehensiveTest::createLoopTrajectory(
    int num_poses, double radius) {

    std::vector<pgos::KeyPoseWithCloud> trajectory;

    for (int i = 0; i < num_poses; ++i) {
        double angle = 2.0 * M_PI * i / num_poses;

        pgos::KeyPoseWithCloud pose;
        pose.t_global = Eigen::Vector3d(radius * cos(angle), radius * sin(angle), 0);

        // 朝向圆心
        Eigen::Vector3d forward(-cos(angle), -sin(angle), 0);
        Eigen::Vector3d up(0, 0, 1);
        Eigen::Vector3d right = forward.cross(up);

        pose.r_global.col(0) = forward;
        pose.r_global.col(1) = right;
        pose.r_global.col(2) = up;

        pose.timestamp = i * 2.0; // 确保时间间隔足够

        // 创建与位置相关的点云
        pose.body_cloud = std::make_shared<CloudType>();
        for (int j = 0; j < 1000; ++j) {
            PointType point;
            point.x = (j % 30) * 0.1f - 1.5f;
            point.y = (j / 30) * 0.1f - 1.5f;
            point.z = abs(j % 10) * 0.1f;

            // 添加位置相关的特征
            point.intensity = 100.0f + (int)(angle * 10) % 100;
            pose.body_cloud->push_back(point);
        }

        trajectory.push_back(pose);
    }

    return trajectory;
}

TEST_F(AdvancedLoopDetectorComprehensiveTest, NoFalsePositivesLinear) {
    // 测试线性轨迹不应该产生回环
    auto trajectory = createLinearTrajectory(20, 2.0);

    for (size_t i = 10; i < trajectory.size(); ++i) {
        auto candidates = detector_->detectLoopCandidates(trajectory, i);

        // 线性轨迹不应该有回环
        EXPECT_TRUE(candidates.empty())
            << "False positive detected at pose " << i;
    }

    auto stats = detector_->getDetectionStats();
    EXPECT_EQ(stats.successful_detections, 0);
    EXPECT_GT(stats.total_queries, 0);
}

TEST_F(AdvancedLoopDetectorComprehensiveTest, DetectTrueLoops) {
    // 测试环形轨迹应该检测到回环
    auto trajectory = createLoopTrajectory(20, 5.0);

    size_t loop_detections = 0;

    // 从轨迹后半段开始检测
    for (size_t i = 15; i < trajectory.size(); ++i) {
        auto candidates = detector_->detectLoopCandidates(trajectory, i);

        for (const auto& candidate : candidates) {
            if (candidate.is_verified && candidate.combined_score >= config_.min_confidence_score) {
                loop_detections++;

                // 验证回环检测的合理性
                EXPECT_LT(candidate.candidate_idx, i - 10)
                    << "Loop detected too close in time";
                EXPECT_GT(candidate.combined_score, config_.min_confidence_score)
                    << "Loop confidence too low";

                // 验证变换矩阵的合理性
                Eigen::Vector3f translation = candidate.transform.block<3, 1>(0, 3);
                EXPECT_LT(translation.norm(), 5.0)
                    << "Loop closure transform too large";
            }
        }
    }

    EXPECT_GT(loop_detections, 0) << "No loops detected in circular trajectory";

    auto stats = detector_->getDetectionStats();
    EXPECT_GT(stats.successful_detections, 0);
    EXPECT_GT(stats.recall_rate, 0.5); // 至少50%召回率
}

TEST_F(AdvancedLoopDetectorComprehensiveTest, PerformanceBenchmark) {
    // 性能基准测试
    auto large_trajectory = createLinearTrajectory(1000, 1.0);

    auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 100; i < 110; ++i) { // 测试10次检测
        detector_->detectLoopCandidates(large_trajectory, i);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 平均每次检测应该在100ms内
    double avg_time = duration.count() / 10.0;
    EXPECT_LT(avg_time, 100.0) << "Loop detection too slow: " << avg_time << "ms";

    auto stats = detector_->getDetectionStats();
    EXPECT_LT(stats.average_detection_time_ms, 100.0);
}

TEST_F(AdvancedLoopDetectorComprehensiveTest, RobustnessToNoise) {
    // 测试对噪声的鲁棒性
    auto trajectory = createLoopTrajectory(15, 8.0);

    // 添加噪声到最后几帧
    for (size_t i = 12; i < trajectory.size(); ++i) {
        // 位置噪声
        trajectory[i].t_global += Eigen::Vector3d::Random() * 0.1;

        // 点云噪声
        for (auto& point : trajectory[i].body_cloud->points) {
            point.x += (rand() % 21 - 10) * 0.01f; // ±10cm噪声
            point.y += (rand() % 21 - 10) * 0.01f;
            point.z += (rand() % 11 - 5) * 0.01f;  // ±5cm噪声
        }
    }

    size_t robust_detections = 0;

    for (size_t i = 12; i < trajectory.size(); ++i) {
        auto candidates = detector_->detectLoopCandidates(trajectory, i);

        for (const auto& candidate : candidates) {
            if (candidate.is_verified && candidate.combined_score >= config_.min_confidence_score * 0.8) {
                robust_detections++;
            }
        }
    }

    // 即使有噪声，也应该能检测到一些回环
    EXPECT_GT(robust_detections, 0) << "Not robust to noise";
}

TEST_F(AdvancedLoopDetectorComprehensiveTest, ConfigurationSensitivity) {
    // 测试配置参数敏感性
    auto trajectory = createLoopTrajectory(12, 6.0);

    // 测试不同置信度阈值
    std::vector<double> confidence_thresholds = {0.5, 0.7, 0.9};

    for (double threshold : confidence_thresholds) {
        config_.min_confidence_score = threshold;
        detector_->updateConfig(config_);

        size_t detections = 0;
        for (size_t i = 8; i < trajectory.size(); ++i) {
            auto candidates = detector_->detectLoopCandidates(trajectory, i);
            detections += candidates.size();
        }

        if (threshold <= 0.7) {
            EXPECT_GT(detections, 0) << "No detections with threshold " << threshold;
        }
        // 较高阈值可能没有检测结果，这是正常的
    }
}

// 压力测试
TEST_F(AdvancedLoopDetectorComprehensiveTest, StressTest) {
    // 大规模轨迹压力测试
    auto large_trajectory = createLoopTrajectory(200, 20.0);

    size_t total_detections = 0;
    double total_time = 0.0;

    for (size_t i = 150; i < large_trajectory.size(); i += 5) { // 每5帧检测一次
        auto start = std::chrono::high_resolution_clock::now();
        auto candidates = detector_->detectLoopCandidates(large_trajectory, i);
        auto end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_time += duration.count() / 1000.0; // 转换为毫秒

        total_detections += candidates.size();
    }

    // 性能要求
    double avg_time = total_time / 10.0; // 10次检测
    EXPECT_LT(avg_time, 200.0) << "Average detection time too high under stress";

    // 功能要求
    EXPECT_GT(total_detections, 0) << "No detections in large trajectory";

    auto stats = detector_->getDetectionStats();
    EXPECT_LT(stats.false_positives, stats.successful_detections * 0.1)
        << "Too many false positives";
}
```

### HBA实时优化测试

```cpp
// 文件位置: ws_livox/src/hba/test/test_realtime_hba_comprehensive.cpp

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "hba/realtime_hba.h"
#include <thread>
#include <atomic>

class RealtimeHBAComprehensiveTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.target_frequency = 10.0;
        config_.max_window_size = 10;
        config_.num_optimization_threads = 2;
        config_.enable_adaptive_scheduling = true;

        realtime_hba_ = std::make_unique<hba::RealtimeHBA>(config_);
    }

    void TearDown() override {
        if (realtime_hba_->isRunning()) {
            realtime_hba_->stop();
        }
    }

    hba::RealtimeHBAConfig config_;
    std::unique_ptr<hba::RealtimeHBA> realtime_hba_;

    // 测试辅助函数
    hba::KeyPoseWithCloud createTestFrame(int frame_id, const Eigen::Vector3d& position);
    void generateTestTrajectory(int num_frames, double frame_interval = 0.1);
    void waitForOptimizations(int expected_optimizations, int timeout_ms = 5000);
};

hba::KeyPoseWithCloud RealtimeHBAComprehensiveTest::createTestFrame(
    int frame_id, const Eigen::Vector3d& position) {

    hba::KeyPoseWithCloud frame;
    frame.timestamp = frame_id * 0.1;
    frame.t_global = position;
    frame.r_global = Eigen::Matrix3d::Identity();

    // 创建测试点云
    frame.body_cloud = std::make_shared<CloudType>();
    for (int i = 0; i < 1000; ++i) {
        PointType point;
        point.x = (i % 20) * 0.05f - 0.5f;
        point.y = (i / 20) * 0.05f - 0.5f;
        point.z = abs(i % 10) * 0.02f;
        point.intensity = 100.0f + frame_id % 50;
        frame.body_cloud->push_back(point);
    }

    return frame;
}

void RealtimeHBAComprehensiveTest::generateTestTrajectory(int num_frames, double frame_interval) {
    ASSERT_TRUE(realtime_hba_->start()) << "Failed to start realtime HBA";

    for (int i = 0; i < num_frames; ++i) {
        Eigen::Vector3d position(i * 0.1, 0, 0); // 直线轨迹
        auto frame = createTestFrame(i, position);

        ASSERT_TRUE(realtime_hba_->addFrameAsync(frame)) << "Failed to add frame " << i;

        std::this_thread::sleep_for(std::chrono::duration<double>(frame_interval));
    }
}

void RealtimeHBAComprehensiveTest::waitForOptimizations(int expected_optimizations, int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
        auto stats = realtime_hba_->getPerformanceStats();
        if (stats.successful_optimizations >= static_cast<size_t>(expected_optimizations)) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    FAIL() << "Timeout waiting for optimizations";
}

TEST_F(RealtimeHBAComprehensiveTest, BasicFunctionality) {
    // 基本功能测试
    ASSERT_TRUE(realtime_hba_->start());
    EXPECT_TRUE(realtime_hba_->isRunning());

    // 添加一些测试帧
    for (int i = 0; i < 20; ++i) {
        auto frame = createTestFrame(i, Eigen::Vector3d(i * 0.1, 0, 0));
        EXPECT_TRUE(realtime_hba_->addFrameAsync(frame));
    }

    // 等待一些优化完成
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto stats = realtime_hba_->getPerformanceStats();
    EXPECT_GT(stats.total_frames, 15);
    EXPECT_GT(stats.successful_optimizations, 0);

    realtime_hba_->stop();
    EXPECT_FALSE(realtime_hba_->isRunning());
}

TEST_F(RealtimeHBAComprehensiveTest, PerformanceTargets) {
    // 性能目标测试
    generateTestTrajectory(100, 0.05); // 20Hz输入

    // 等待足够的优化
    waitForOptimizations(5, 10000);

    auto stats = realtime_hba_->getPerformanceStats();

    // 检查频率目标
    EXPECT_GT(stats.current_frequency, config_.target_frequency * 0.7)
        << "Frequency too low: " << stats.current_frequency;

    // 检查处理时间
    EXPECT_LT(stats.average_optimization_time_ms, 100.0)
        << "Average optimization time too high: " << stats.average_optimization_time_ms;

    // 检查成功率
    double success_rate = static_cast<double>(stats.successful_optimizations) /
                         (stats.successful_optimizations + stats.failed_optimizations);
    EXPECT_GT(success_rate, 0.8) << "Success rate too low: " << success_rate;
}

TEST_F(RealtimeHBAComprehensiveTest, AdaptiveScheduling) {
    // 自适应调度测试
    config_.enable_adaptive_scheduling = true;
    config_.max_optimization_time_ms = 50.0; // 较严格的时间限制
    realtime_hba_->updateConfig(config_);

    ASSERT_TRUE(realtime_hba_->start());

    // 第一阶段：正常负载
    for (int i = 0; i < 30; ++i) {
        auto frame = createTestFrame(i, Eigen::Vector3d(i * 0.1, 0, 0));
        realtime_hba_->addFrameAsync(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto normal_stats = realtime_hba_->getPerformanceStats();

    // 第二阶段：高负载
    for (int i = 30; i < 100; ++i) {
        auto frame = createTestFrame(i, Eigen::Vector3d(i * 0.1, 0, 0));
        realtime_hba_->addFrameAsync(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    auto high_load_stats = realtime_hba_->getPerformanceStats();

    // 自适应调度应该能够处理负载变化
    EXPECT_LT(high_load_stats.queue_length, 15)
        << "Queue not managed under high load";
    EXPECT_GT(high_load_stats.successful_optimizations, normal_stats.successful_optimizations)
        << "No additional optimizations under high load";
}

TEST_F(RealtimeHBAComprehensiveTest, ConcurrencyStressTest) {
    // 并发压力测试
    ASSERT_TRUE(realtime_hba_->start());

    std::atomic<int> frames_added{0};
    std::atomic<bool> stop_flag{false};

    // 多个线程同时添加帧
    std::vector<std::thread> producer_threads;

    for (int t = 0; t < 3; ++t) {
        producer_threads.emplace_back([this, &frames_added, &stop_flag, t]() {
            int frame_id = t * 1000;
            while (!stop_flag) {
                auto frame = createTestFrame(frame_id++,
                    Eigen::Vector3d(frame_id * 0.05, t * 2.0, 0));

                if (realtime_hba_->addFrameAsync(frame)) {
                    frames_added++;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }

    // 运行5秒
    std::this_thread::sleep_for(std::chrono::seconds(5));
    stop_flag = true;

    for (auto& thread : producer_threads) {
        thread.join();
    }

    // 等待处理完成
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto stats = realtime_hba_->getPerformanceStats();

    EXPECT_GT(frames_added.load(), 200) << "Not enough frames added";
    EXPECT_GT(stats.successful_optimizations, 10) << "Not enough optimizations";
    EXPECT_LT(stats.failed_optimizations, stats.successful_optimizations * 0.2)
        << "Too many failures under stress";
}

TEST_F(RealtimeHBAComprehensiveTest, MemoryStability) {
    // 内存稳定性测试
    ASSERT_TRUE(realtime_hba_->start());

    size_t initial_memory = getCurrentMemoryUsage();

    // 长时间运行
    for (int batch = 0; batch < 20; ++batch) {
        for (int i = 0; i < 50; ++i) {
            auto frame = createTestFrame(batch * 50 + i,
                Eigen::Vector3d(i * 0.1, batch * 5.0, 0));
            realtime_hba_->addFrameAsync(frame);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 每个批次后检查内存
        if (batch % 5 == 0) {
            size_t current_memory = getCurrentMemoryUsage();
            double memory_growth = static_cast<double>(current_memory - initial_memory) / initial_memory;

            EXPECT_LT(memory_growth, 2.0)
                << "Memory growth too high: " << memory_growth * 100 << "%";
        }
    }

    auto final_stats = realtime_hba_->getPerformanceStats();
    EXPECT_GT(final_stats.successful_optimizations, 50);
}

TEST_F(RealtimeHBAComprehensiveTest, QualityValidation) {
    // 优化质量验证
    ASSERT_TRUE(realtime_hba_->start());

    // 创建有累积误差的轨迹
    std::vector<Eigen::Vector3d> noisy_trajectory;
    Eigen::Vector3d position(0, 0, 0);

    for (int i = 0; i < 50; ++i) {
        // 添加累积误差
        Eigen::Vector3d noise = Eigen::Vector3d::Random() * 0.01;
        position += Eigen::Vector3d(0.1, 0, 0) + noise;
        noisy_trajectory.push_back(position);

        auto frame = createTestFrame(i, position);
        realtime_hba_->addFrameAsync(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待优化完成
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto optimized_poses = realtime_hba_->getOptimizedPoses();
    ASSERT_GT(optimized_poses.size(), 30);

    // 检查优化后的轨迹是否更平滑
    double original_roughness = 0.0;
    double optimized_roughness = 0.0;

    for (size_t i = 1; i < std::min(noisy_trajectory.size(), optimized_poses.size()) - 1; ++i) {
        // 计算原轨迹的粗糙度（二阶差分）
        Eigen::Vector3d orig_accel = noisy_trajectory[i+1] - 2*noisy_trajectory[i] + noisy_trajectory[i-1];
        original_roughness += orig_accel.norm();

        // 计算优化后轨迹的粗糙度
        Eigen::Vector3d opt_accel = optimized_poses[i+1].t - 2*optimized_poses[i].t + optimized_poses[i-1].t;
        optimized_roughness += opt_accel.norm();
    }

    EXPECT_LT(optimized_roughness, original_roughness * 0.8)
        << "Optimization did not improve trajectory smoothness";
}

// 辅助函数实现
size_t RealtimeHBAComprehensiveTest::getCurrentMemoryUsage() {
    std::ifstream status_file("/proc/self/status");
    std::string line;
    while (std::getline(status_file, line)) {
        if (line.substr(0, 6) == "VmRSS:") {
            std::istringstream iss(line);
            std::string label, value, unit;
            iss >> label >> value >> unit;
            return std::stoul(value) * 1024;
        }
    }
    return 0;
}
```

## 集成测试

### 系统级集成测试

```cpp
// 文件位置: test/integration/test_slam_system_integration.cpp

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>

class SLAMSystemIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);

        // 启动所有SLAM组件
        startSLAMSystem();

        // 等待系统初始化
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    void TearDown() override {
        stopSLAMSystem();
        rclcpp::shutdown();
    }

    void startSLAMSystem();
    void stopSLAMSystem();
    void playTestBag(const std::string& bag_path);
    void validateSystemOutput();

    std::vector<rclcpp::Node::SharedPtr> nodes_;
    std::map<std::string, std::vector<std::string>> topic_messages_;
};

void SLAMSystemIntegrationTest::startSLAMSystem() {
    // 这里会启动实际的ROS2节点
    // 实际实现中可能需要使用launch文件或直接创建节点
}

TEST_F(SLAMSystemIntegrationTest, EndToEndDataFlow) {
    // 端到端数据流测试
    playTestBag("test_data/integration_test.bag");

    // 验证各个组件的输出
    validateSystemOutput();

    // 检查关键话题是否有数据
    EXPECT_GT(topic_messages_["/fastlio2/lio_odom"].size(), 100);
    EXPECT_GT(topic_messages_["/pgo/loop_markers"].size(), 1);
    EXPECT_GT(topic_messages_["/hba/optimized_map"].size(), 10);
    EXPECT_GT(topic_messages_["/localizer/filtered_cloud"].size(), 100);
}

TEST_F(SLAMSystemIntegrationTest, SystemResilience) {
    // 系统弹性测试：模拟各种故障情况

    // 1. 数据中断测试
    playTestBag("test_data/data_interruption_test.bag");
    validateSystemOutput();

    // 2. 高频数据测试
    playTestBag("test_data/high_frequency_test.bag");
    validateSystemOutput();

    // 3. 噪声数据测试
    playTestBag("test_data/noisy_data_test.bag");
    validateSystemOutput();
}

TEST_F(SLAMSystemIntegrationTest, PerformanceUnderLoad) {
    // 负载下的性能测试
    auto start_time = std::chrono::steady_clock::now();

    playTestBag("test_data/performance_test.bag");

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    // 检查实时性能
    EXPECT_LT(duration.count(), 300); // 5分钟数据应该在5分钟内处理完

    validateSystemOutput();
}
```

### 性能集成测试脚本

```bash
#!/bin/bash
# 文件位置: test/integration/performance_integration_test.sh

echo "Starting SLAM System Performance Integration Test..."

# 设置测试环境
export ROS_DOMAIN_ID=42
export RCUTILS_LOGGING_LEVEL=WARN

cd ws_livox
source install/setup.bash

# 1. 启动完整SLAM系统
echo "Starting complete SLAM system..."

# FAST-LIO2
ros2 launch fastlio2 enhanced_visualization.launch.py \
    config_path:=$(pwd)/src/fastlio2/config/lio.yaml &
FASTLIO_PID=$!

sleep 3

# PGO with advanced loop detection
ros2 launch pgo pgo_launch.py \
    enable_advanced_loop_detection:=true &
PGO_PID=$!

sleep 2

# Realtime HBA
ros2 launch hba hba_launch.py \
    enable_realtime_mode:=true &
HBA_PID=$!

sleep 2

# Localizer with dynamic filter
ros2 launch localizer localizer_launch.py \
    enable_dynamic_filter:=true &
LOCALIZER_PID=$!

sleep 3

echo "All components started. Running performance tests..."

# 2. 性能监控
monitor_performance() {
    local test_name=$1
    local duration=$2

    echo "Monitoring $test_name for $duration seconds..."

    # 创建监控脚本
    cat > /tmp/monitor_${test_name}.py << EOF
import rclpy
from rclcpp import Node
from std_msgs.msg import String
import time
import json

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.stats = {}

        # 订阅各组件性能统计
        self.create_subscription(String, '/hba/performance_stats',
                               lambda msg: self.record_stats('hba', msg), 10)
        self.create_subscription(String, '/localizer/dynamic_filter_stats',
                               lambda msg: self.record_stats('localizer', msg), 10)

        self.start_time = time.time()

    def record_stats(self, component, msg):
        if component not in self.stats:
            self.stats[component] = []
        self.stats[component].append({
            'timestamp': time.time() - self.start_time,
            'data': msg.data
        })

    def save_results(self):
        with open('/tmp/performance_${test_name}.json', 'w') as f:
            json.dump(self.stats, f, indent=2)

def main():
    rclpy.init()
    monitor = PerformanceMonitor()

    try:
        rclpy.spin_for_timeout(monitor, timeout_sec=$duration)
    finally:
        monitor.save_results()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

    python3 /tmp/monitor_${test_name}.py &
    MONITOR_PID=$!

    sleep $duration
    kill $MONITOR_PID 2>/dev/null || true

    # 分析结果
    if [ -f "/tmp/performance_${test_name}.json" ]; then
        echo "Performance results for $test_name:"
        python3 -c "
import json
with open('/tmp/performance_${test_name}.json') as f:
    data = json.load(f)

for component, stats in data.items():
    print(f'{component}: {len(stats)} measurements')
    if stats:
        print(f'  Duration: {stats[-1][\"timestamp\"]:.1f}s')
"
    fi
}

# 3. 运行不同场景的性能测试

# 场景1：室内环境（动态对象多）
echo "Testing indoor scenario with dynamic objects..."
ros2 bag play ../test_data/indoor_dynamic_environment.bag --rate 1.0 &
BAG_PID=$!
monitor_performance "indoor" 120
kill $BAG_PID 2>/dev/null || true

sleep 5

# 场景2：室外环境（长距离回环）
echo "Testing outdoor scenario with long-distance loops..."
ros2 bag play ../test_data/outdoor_loop_closure.bag --rate 1.0 &
BAG_PID=$!
monitor_performance "outdoor" 300
kill $BAG_PID 2>/dev/null || true

sleep 5

# 场景3：高速场景（实时性测试）
echo "Testing high-speed scenario for real-time performance..."
ros2 bag play ../test_data/high_speed_trajectory.bag --rate 2.0 &
BAG_PID=$!
monitor_performance "high_speed" 180
kill $BAG_PID 2>/dev/null || true

# 4. 性能报告生成
echo "Generating performance report..."

cat > /tmp/generate_report.py << 'EOF'
import json
import os
from datetime import datetime

def analyze_performance_data(test_name):
    file_path = f'/tmp/performance_{test_name}.json'
    if not os.path.exists(file_path):
        return None

    with open(file_path) as f:
        data = json.load(f)

    analysis = {
        'test_name': test_name,
        'components': {}
    }

    for component, stats in data.items():
        if not stats:
            continue

        component_analysis = {
            'total_measurements': len(stats),
            'duration': stats[-1]['timestamp'] if stats else 0,
            'avg_frequency': len(stats) / stats[-1]['timestamp'] if stats and stats[-1]['timestamp'] > 0 else 0
        }

        # 解析HBA性能数据
        if component == 'hba':
            frequencies = []
            optimization_times = []

            for stat in stats:
                data_str = stat['data']
                if 'Freq=' in data_str:
                    freq = float(data_str.split('Freq=')[1].split('Hz')[0])
                    frequencies.append(freq)
                if 'AvgTime=' in data_str:
                    time_ms = float(data_str.split('AvgTime=')[1].split('ms')[0])
                    optimization_times.append(time_ms)

            if frequencies:
                component_analysis['avg_optimization_frequency'] = sum(frequencies) / len(frequencies)
                component_analysis['min_optimization_frequency'] = min(frequencies)

            if optimization_times:
                component_analysis['avg_optimization_time_ms'] = sum(optimization_times) / len(optimization_times)
                component_analysis['max_optimization_time_ms'] = max(optimization_times)

        analysis['components'][component] = component_analysis

    return analysis

def generate_report():
    report = {
        'test_timestamp': datetime.now().isoformat(),
        'scenarios': {}
    }

    for scenario in ['indoor', 'outdoor', 'high_speed']:
        analysis = analyze_performance_data(scenario)
        if analysis:
            report['scenarios'][scenario] = analysis

    # 生成报告文件
    with open('/tmp/slam_performance_report.json', 'w') as f:
        json.dump(report, f, indent=2)

    # 生成人类可读的报告
    with open('/tmp/slam_performance_report.txt', 'w') as f:
        f.write("SLAM System Performance Integration Test Report\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Test Time: {report['test_timestamp']}\n\n")

        for scenario_name, scenario_data in report['scenarios'].items():
            f.write(f"Scenario: {scenario_name.upper()}\n")
            f.write("-" * 30 + "\n")

            for component, comp_data in scenario_data['components'].items():
                f.write(f"  {component.upper()}:\n")
                f.write(f"    Duration: {comp_data['duration']:.1f}s\n")
                f.write(f"    Measurements: {comp_data['total_measurements']}\n")
                f.write(f"    Frequency: {comp_data['avg_frequency']:.1f}Hz\n")

                if 'avg_optimization_frequency' in comp_data:
                    f.write(f"    Optimization Freq: {comp_data['avg_optimization_frequency']:.1f}Hz\n")
                if 'avg_optimization_time_ms' in comp_data:
                    f.write(f"    Avg Optimization Time: {comp_data['avg_optimization_time_ms']:.1f}ms\n")

                f.write("\n")
            f.write("\n")

    print("Performance report generated:")
    print("  JSON: /tmp/slam_performance_report.json")
    print("  Text: /tmp/slam_performance_report.txt")

if __name__ == '__main__':
    generate_report()
EOF

python3 /tmp/generate_report.py

# 5. 清理
echo "Cleaning up..."
kill $FASTLIO_PID $PGO_PID $HBA_PID $LOCALIZER_PID 2>/dev/null || true

# 6. 显示结果
echo "Performance Integration Test Results:"
echo "====================================="
cat /tmp/slam_performance_report.txt

# 7. 验证性能指标
echo "Validating performance metrics..."

python3 -c "
import json
import sys

with open('/tmp/slam_performance_report.json') as f:
    report = json.load(f)

exit_code = 0

for scenario, data in report['scenarios'].items():
    print(f'Validating {scenario}...')

    # 检查HBA性能
    if 'hba' in data['components']:
        hba_data = data['components']['hba']

        if 'avg_optimization_frequency' in hba_data:
            freq = hba_data['avg_optimization_frequency']
            if freq < 5.0:
                print(f'  ❌ HBA frequency too low: {freq:.1f}Hz < 5.0Hz')
                exit_code = 1
            else:
                print(f'  ✅ HBA frequency OK: {freq:.1f}Hz')

        if 'avg_optimization_time_ms' in hba_data:
            time_ms = hba_data['avg_optimization_time_ms']
            if time_ms > 100.0:
                print(f'  ❌ HBA optimization time too high: {time_ms:.1f}ms > 100ms')
                exit_code = 1
            else:
                print(f'  ✅ HBA optimization time OK: {time_ms:.1f}ms')

sys.exit(exit_code)
"

VALIDATION_RESULT=$?

if [ $VALIDATION_RESULT -eq 0 ]; then
    echo "✅ All performance metrics passed!"
else
    echo "❌ Some performance metrics failed!"
fi

echo "Performance integration test completed."
exit $VALIDATION_RESULT
```

## 端到端测试

### 系统级端到端测试

```bash
#!/bin/bash
# 文件位置: test/e2e/end_to_end_system_test.sh

echo "Starting End-to-End SLAM System Test..."

# 测试配置
TEST_DATASETS_DIR="../test_data/e2e"
RESULTS_DIR="/tmp/e2e_test_results"
mkdir -p $RESULTS_DIR

cd ws_livox
source install/setup.bash

# 端到端测试函数
run_e2e_test() {
    local test_name=$1
    local dataset_path=$2
    local expected_performance=$3

    echo "Running E2E test: $test_name"
    echo "Dataset: $dataset_path"

    # 创建测试结果目录
    local result_dir="$RESULTS_DIR/$test_name"
    mkdir -p $result_dir

    # 启动完整SLAM系统
    echo "Starting SLAM system..."

    # 使用配置化的启动方式
    ros2 launch fastlio2 full_slam_system.launch.py \
        enable_dynamic_filter:=true \
        enable_advanced_pgo:=true \
        enable_realtime_hba:=true \
        output_dir:=$result_dir &
    SLAM_PID=$!

    sleep 5

    # 启动数据记录
    echo "Starting data recording..."
    ros2 bag record -o $result_dir/output_bag \
        /fastlio2/lio_odom \
        /pgo/loop_markers \
        /hba/optimized_map \
        /localizer/filtered_cloud \
        /slam/performance_metrics &
    RECORD_PID=$!

    sleep 2

    # 播放测试数据
    echo "Playing test dataset..."
    local start_time=$(date +%s)

    ros2 bag play $dataset_path --rate 1.0
    local play_result=$?

    local end_time=$(date +%s)
    local duration=$((end_time - start_time))

    sleep 5  # 等待处理完成

    # 停止记录和SLAM系统
    kill $RECORD_PID 2>/dev/null || true
    kill $SLAM_PID 2>/dev/null || true

    sleep 3

    # 分析结果
    echo "Analyzing results for $test_name..."

    local analysis_result=$(analyze_e2e_results $result_dir $expected_performance)

    echo "Test $test_name completed in ${duration}s"
    echo "Result: $analysis_result"

    return $play_result
}

# 结果分析函数
analyze_e2e_results() {
    local result_dir=$1
    local expected_performance=$2

    # 分析生成的地图质量
    python3 << EOF
import json
import numpy as np
from pathlib import Path

def analyze_results(result_dir, expected_perf):
    result_path = Path(result_dir)

    # 分析轨迹精度
    trajectory_file = result_path / "trajectory.txt"
    if trajectory_file.exists():
        # 计算轨迹统计
        print(f"Trajectory analysis completed")

    # 分析地图质量
    map_file = result_path / "final_map.pcd"
    if map_file.exists():
        print(f"Map quality analysis completed")

    # 分析性能指标
    performance_file = result_path / "performance_summary.json"
    if performance_file.exists():
        with open(performance_file) as f:
            perf_data = json.load(f)

        # 检查关键性能指标
        if perf_data.get('avg_processing_time_ms', 1000) < 100:
            print("✅ Real-time performance: PASS")
        else:
            print("❌ Real-time performance: FAIL")

        if perf_data.get('loop_closure_accuracy', 0) > 0.8:
            print("✅ Loop closure accuracy: PASS")
        else:
            print("❌ Loop closure accuracy: FAIL")

        if perf_data.get('dynamic_filter_effectiveness', 0) > 0.7:
            print("✅ Dynamic filtering: PASS")
        else:
            print("❌ Dynamic filtering: FAIL")

    return "ANALYSIS_COMPLETED"

print(analyze_results("$result_dir", "$expected_performance"))
EOF
}

# 测试场景定义
echo "Defining test scenarios..."

# 场景1：室内动态环境
echo "Test 1: Indoor Dynamic Environment"
run_e2e_test "indoor_dynamic" \
    "$TEST_DATASETS_DIR/indoor_with_people.bag" \
    "realtime_processing:100ms,dynamic_filter_ratio:0.2"

# 场景2：室外大尺度回环
echo "Test 2: Outdoor Large-scale Loop Closure"
run_e2e_test "outdoor_loops" \
    "$TEST_DATASETS_DIR/outdoor_campus_loop.bag" \
    "loop_closure_accuracy:0.85,trajectory_error:0.5m"

# 场景3：高速移动场景
echo "Test 3: High-speed Movement"
run_e2e_test "high_speed" \
    "$TEST_DATASETS_DIR/drone_flight_path.bag" \
    "processing_frequency:10Hz,optimization_delay:50ms"

# 场景4：复杂结构化环境
echo "Test 4: Complex Structured Environment"
run_e2e_test "structured_complex" \
    "$TEST_DATASETS_DIR/warehouse_navigation.bag" \
    "mapping_accuracy:0.1m,feature_matching:0.9"

# 生成综合报告
echo "Generating comprehensive E2E report..."

cat > $RESULTS_DIR/e2e_summary_report.py << 'EOF'
import json
import os
from pathlib import Path
from datetime import datetime

def generate_e2e_report():
    results_dir = Path("/tmp/e2e_test_results")

    report = {
        'test_timestamp': datetime.now().isoformat(),
        'test_scenarios': {},
        'overall_summary': {
            'total_tests': 0,
            'passed_tests': 0,
            'failed_tests': 0
        }
    }

    # 扫描测试结果目录
    for test_dir in results_dir.iterdir():
        if test_dir.is_dir() and test_dir.name != '.':
            test_name = test_dir.name

            # 分析每个测试的结果
            scenario_report = analyze_scenario(test_dir)
            report['test_scenarios'][test_name] = scenario_report

            report['overall_summary']['total_tests'] += 1
            if scenario_report['status'] == 'PASSED':
                report['overall_summary']['passed_tests'] += 1
            else:
                report['overall_summary']['failed_tests'] += 1

    # 保存报告
    with open(results_dir / 'e2e_comprehensive_report.json', 'w') as f:
        json.dump(report, f, indent=2)

    # 生成人类可读报告
    generate_readable_report(report, results_dir / 'e2e_comprehensive_report.txt')

    return report

def analyze_scenario(test_dir):
    scenario_report = {
        'test_name': test_dir.name,
        'status': 'UNKNOWN',
        'metrics': {},
        'issues': []
    }

    # 检查是否有输出文件
    output_bag = test_dir / 'output_bag'
    if not output_bag.exists():
        scenario_report['status'] = 'FAILED'
        scenario_report['issues'].append('No output bag recorded')
        return scenario_report

    # 分析性能数据
    performance_file = test_dir / 'performance_summary.json'
    if performance_file.exists():
        try:
            with open(performance_file) as f:
                perf_data = json.load(f)
            scenario_report['metrics'] = perf_data
        except:
            scenario_report['issues'].append('Failed to read performance data')

    # 基于指标判断测试状态
    if len(scenario_report['issues']) == 0:
        scenario_report['status'] = 'PASSED'
    else:
        scenario_report['status'] = 'FAILED'

    return scenario_report

def generate_readable_report(report, output_file):
    with open(output_file, 'w') as f:
        f.write("SLAM System End-to-End Test Report\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Test Timestamp: {report['test_timestamp']}\n")
        f.write(f"Total Tests: {report['overall_summary']['total_tests']}\n")
        f.write(f"Passed: {report['overall_summary']['passed_tests']}\n")
        f.write(f"Failed: {report['overall_summary']['failed_tests']}\n\n")

        for test_name, test_data in report['test_scenarios'].items():
            f.write(f"Test: {test_name}\n")
            f.write("-" * 30 + "\n")
            f.write(f"Status: {test_data['status']}\n")

            if test_data['metrics']:
                f.write("Metrics:\n")
                for metric, value in test_data['metrics'].items():
                    f.write(f"  {metric}: {value}\n")

            if test_data['issues']:
                f.write("Issues:\n")
                for issue in test_data['issues']:
                    f.write(f"  - {issue}\n")

            f.write("\n")

if __name__ == '__main__':
    report = generate_e2e_report()

    print(f"E2E Test Summary:")
    print(f"Total: {report['overall_summary']['total_tests']}")
    print(f"Passed: {report['overall_summary']['passed_tests']}")
    print(f"Failed: {report['overall_summary']['failed_tests']}")

    if report['overall_summary']['failed_tests'] > 0:
        print("\nFailed tests:")
        for test_name, test_data in report['test_scenarios'].items():
            if test_data['status'] == 'FAILED':
                print(f"  - {test_name}: {', '.join(test_data['issues'])}")
EOF

python3 $RESULTS_DIR/e2e_summary_report.py

# 显示最终结果
echo "End-to-End Test Results:"
echo "========================"
cat $RESULTS_DIR/e2e_comprehensive_report.txt

# 检查测试是否通过
FINAL_RESULT=$(python3 -c "
import json
with open('$RESULTS_DIR/e2e_comprehensive_report.json') as f:
    report = json.load(f)
if report['overall_summary']['failed_tests'] == 0:
    print('PASSED')
else:
    print('FAILED')
")

echo "Final E2E Test Result: $FINAL_RESULT"

if [ "$FINAL_RESULT" = "PASSED" ]; then
    echo "🎉 All end-to-end tests passed successfully!"
    exit 0
else
    echo "❌ Some end-to-end tests failed. Check the detailed report."
    exit 1
fi
```

## 持续集成测试

### CI/CD流水线配置

```yaml
# 文件位置: .github/workflows/slam_comprehensive_test.yml
name: SLAM System Comprehensive Testing

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  unit-tests:
    runs-on: ubuntu-22.04
    strategy:
      matrix:
        ros-distro: [humble]

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS 2
      uses: ros-tooling/setup-ros@v0.6
      with:
        required-ros-distributions: ${{ matrix.ros-distro }}

    - name: Install Dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y \
          libpcl-dev \
          libeigen3-dev \
          libopencv-dev \
          libyaml-cpp-dev

    - name: Build SLAM System
      run: |
        cd ws_livox
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

    - name: Run Unit Tests
      run: |
        cd ws_livox
        source install/setup.bash
        colcon test --packages-select interface localizer pgo hba
        colcon test-result --verbose

    - name: Upload Test Results
      uses: actions/upload-artifact@v3
      if: always()
      with:
        name: unit-test-results
        path: ws_livox/log/

  integration-tests:
    runs-on: ubuntu-22.04
    needs: unit-tests

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS 2
      uses: ros-tooling/setup-ros@v0.6
      with:
        required-ros-distributions: humble

    - name: Download Test Data
      run: |
        mkdir -p test_data
        # 下载或生成测试数据
        ./scripts/generate_test_data.sh

    - name: Build System
      run: |
        cd ws_livox
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

    - name: Run Integration Tests
      run: |
        cd ws_livox
        source install/setup.bash
        ../test/integration/performance_integration_test.sh

    - name: Upload Integration Results
      uses: actions/upload-artifact@v3
      with:
        name: integration-test-results
        path: /tmp/slam_performance_report.*

  performance-benchmarks:
    runs-on: ubuntu-22.04
    needs: integration-tests

    steps:
    - uses: actions/checkout@v3

    - name: Setup Performance Environment
      run: |
        # 配置性能测试环境
        sudo sysctl -w kernel.perf_event_paranoid=1
        sudo apt-get install -y linux-tools-generic

    - name: Run Performance Benchmarks
      run: |
        ./test/performance/run_benchmarks.sh

    - name: Generate Performance Report
      run: |
        python3 test/performance/generate_benchmark_report.py

    - name: Upload Performance Results
      uses: actions/upload-artifact@v3
      with:
        name: performance-benchmark-results
        path: test/performance/results/

  quality-assurance:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Code Quality Check
      run: |
        # 静态代码分析
        find . -name "*.cpp" -o -name "*.hpp" | xargs cppcheck --enable=all

        # 代码格式检查
        find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format --dry-run --Werror

    - name: Documentation Check
      run: |
        # 检查文档完整性
        python3 scripts/check_documentation.py

    - name: Security Scan
      run: |
        # 安全扫描
        bandit -r . -f json -o security_report.json || true

    - name: Upload QA Results
      uses: actions/upload-artifact@v3
      with:
        name: quality-assurance-results
        path: |
          security_report.json
          cppcheck_report.xml

  end-to-end-tests:
    runs-on: ubuntu-22.04
    needs: [unit-tests, integration-tests]
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'

    steps:
    - uses: actions/checkout@v3

    - name: Setup Full Test Environment
      run: |
        # 设置完整测试环境
        ./scripts/setup_e2e_environment.sh

    - name: Download E2E Test Data
      run: |
        # 下载大型端到端测试数据集
        ./scripts/download_e2e_datasets.sh

    - name: Run End-to-End Tests
      run: |
        timeout 1800 ./test/e2e/end_to_end_system_test.sh

    - name: Upload E2E Results
      uses: actions/upload-artifact@v3
      if: always()
      with:
        name: e2e-test-results
        path: /tmp/e2e_test_results/

  final-report:
    runs-on: ubuntu-22.04
    needs: [unit-tests, integration-tests, performance-benchmarks, quality-assurance]
    if: always()

    steps:
    - uses: actions/checkout@v3

    - name: Download All Artifacts
      uses: actions/download-artifact@v3

    - name: Generate Comprehensive Report
      run: |
        python3 scripts/generate_comprehensive_report.py \
          --unit-tests unit-test-results/ \
          --integration integration-test-results/ \
          --performance performance-benchmark-results/ \
          --quality quality-assurance-results/

    - name: Upload Final Report
      uses: actions/upload-artifact@v3
      with:
        name: comprehensive-test-report
        path: comprehensive_report.*

    - name: Comment on PR
      if: github.event_name == 'pull_request'
      uses: actions/github-script@v6
      with:
        script: |
          const fs = require('fs');
          const report = fs.readFileSync('comprehensive_report.md', 'utf8');

          github.rest.issues.createComment({
            issue_number: context.issue.number,
            owner: context.repo.owner,
            repo: context.repo.repo,
            body: report
          });
```

## 总结

本测试与验证指南提供了完整的三层测试架构：

### 测试覆盖范围
- **单元测试**: 覆盖核心算法和组件
- **集成测试**: 验证组件间协作和系统性能
- **端到端测试**: 验证完整系统在真实场景下的表现

### 关键验证指标
- **功能正确性**: 算法逻辑和输出正确性
- **性能指标**: 实时性、精度、资源使用
- **鲁棒性**: 噪声、异常数据、系统故障的处理
- **可扩展性**: 大规模数据和长时间运行的稳定性

### 质量保证
- **自动化测试**: CI/CD流水线确保代码质量
- **性能基准**: 持续监控系统性能退化
- **回归测试**: 确保新功能不影响现有功能
- **文档验证**: 确保文档与实际实现一致

通过执行这套完整的测试方案，可以确保三大功能增强（动态对象处理、PGO回环检测、HBA实时优化）的质量和可靠性，为生产环境部署提供信心保证。