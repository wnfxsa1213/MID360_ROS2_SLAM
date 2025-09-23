#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Eigen>
#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>
#include <future>
#include <string>
#include "commons.h"

namespace localizers {

/**
 * @brief 点云时间信息结构体
 * 用于存储单个点的历史轨迹和时间一致性信息
 */
struct PointTemporalInfo {
    std::vector<Eigen::Vector3f> history_positions;  ///< 历史位置序列
    std::vector<double> timestamps;                   ///< 对应的时间戳
    float stability_score;                            ///< 稳定性评分 [0, 1]
    bool is_dynamic;                                  ///< 是否为动态点

    /**
     * @brief 默认构造函数
     */
    PointTemporalInfo() : stability_score(1.0f), is_dynamic(false) {}

    /**
     * @brief 清空历史信息
     */
    void clear() {
        history_positions.clear();
        timestamps.clear();
        stability_score = 1.0f;
        is_dynamic = false;
    }

    /**
     * @brief 添加历史位置
     * @param position 3D位置
     * @param timestamp 时间戳
     */
    void addHistoryPosition(const Eigen::Vector3f& position, double timestamp) {
        history_positions.push_back(position);
        timestamps.push_back(timestamp);
    }
};

/**
 * @brief 动态对象过滤器配置参数
 */
struct DynamicFilterConfig {
    // 时间一致性参数
    float motion_threshold = 0.1f;        ///< 运动阈值(m)
    int history_size = 5;                 ///< 历史帧数
    float stability_threshold = 0.8f;     ///< 稳定性阈值
    double max_time_diff = 0.5;           ///< 最大时间差(s)

    // 几何特征参数
    float search_radius = 0.2f;           ///< 邻域搜索半径(m)
    int min_neighbors = 10;               ///< 最小邻居点数
    float normal_consistency_thresh = 0.8f; ///< 法向量一致性阈值
    float density_ratio_thresh = 0.5f;    ///< 密度比值阈值

    // 性能参数
    int downsample_ratio = 2;             ///< 降采样比例
    int max_points_per_frame = 50000;     ///< 每帧最大点数
    float voxel_size_base = 0.01f;        ///< 基础体素大小(m)

    /**
     * @brief 验证配置参数的有效性
     * @return true 如果配置有效
     */
    bool isValid() const {
        return motion_threshold > 0.0f &&
               history_size > 0 &&
               stability_threshold >= 0.0f && stability_threshold <= 1.0f &&
               max_time_diff > 0.0 &&
               search_radius > 0.0f &&
               min_neighbors > 0 &&
               normal_consistency_thresh >= 0.0f && normal_consistency_thresh <= 1.0f &&
               density_ratio_thresh >= 0.0f && density_ratio_thresh <= 1.0f &&
               downsample_ratio > 0 &&
               max_points_per_frame > 0 &&
               voxel_size_base > 0.0f;
    }
};

/**
 * @brief 过滤器统计信息
 */
struct FilterStats {
    size_t total_points = 0;              ///< 总点数
    size_t dynamic_points = 0;            ///< 动态点数
    size_t static_points = 0;             ///< 静态点数
    double filter_ratio = 0.0;            ///< 过滤比例 [0, 1]
    double processing_time_ms = 0.0;      ///< 处理时间(毫秒)
    size_t history_frames = 0;            ///< 当前历史帧数
    double memory_usage_mb = 0.0;         ///< 内存使用量(MB)

    /**
     * @brief 重置统计信息
     */
    void reset() {
        total_points = 0;
        dynamic_points = 0;
        static_points = 0;
        filter_ratio = 0.0;
        processing_time_ms = 0.0;
        history_frames = 0;
        memory_usage_mb = 0.0;
    }

    /**
     * @brief 更新过滤比例
     */
    void updateFilterRatio() {
        if (total_points > 0) {
            filter_ratio = static_cast<double>(dynamic_points) / total_points;
        } else {
            filter_ratio = 0.0;
        }
    }
};

/**
 * @brief 动态对象过滤器类
 * 
 * 实现基于时间一致性和几何特征的动态对象检测与过滤算法。
 * 主要功能：
 * - 多帧时间一致性分析
 * - 几何特征验证
 * - 实时点云过滤
 * - 性能统计与监控
 */
class DynamicObjectFilter {
public:
    /**
     * @brief 构造函数
     * @param config 过滤器配置参数
     */
    explicit DynamicObjectFilter(const DynamicFilterConfig& config);

    /**
     * @brief 析构函数
     */
    ~DynamicObjectFilter() = default;

    // 禁用拷贝构造和赋值
    DynamicObjectFilter(const DynamicObjectFilter&) = delete;
    DynamicObjectFilter& operator=(const DynamicObjectFilter&) = delete;

    // 允许移动构造和赋值
    DynamicObjectFilter(DynamicObjectFilter&&) = default;
    DynamicObjectFilter& operator=(DynamicObjectFilter&&) = default;

    /**
     * @brief 主要过滤接口 - 过滤动态对象
     * @param input_cloud 输入点云
     * @param timestamp 时间戳
     * @return 过滤后的静态点云
     */
    CloudType::Ptr filterDynamicObjects(const CloudType::Ptr& input_cloud,
                                       double timestamp);

    /**
     * @brief 异步过滤接口（实验性功能）
     * @param input_cloud 输入点云
     * @param timestamp 时间戳
     * @return 过滤后的静态点云的future对象
     */
    std::future<CloudType::Ptr> filterDynamicObjectsAsync(const CloudType::Ptr& input_cloud,
                                                         double timestamp);

    /**
     * @brief 更新配置参数
     * @param config 新的配置参数
     * @return true 如果更新成功
     */
    bool updateConfig(const DynamicFilterConfig& config);

    /**
     * @brief 获取当前配置
     * @return 当前配置参数
     */
    DynamicFilterConfig getConfig() const;

    /**
     * @brief 获取最近一次过滤的统计信息
     * @return 过滤统计信息
     */
    FilterStats getLastFilterStats() const;

    /**
     * @brief 重置统计信息和历史数据
     */
    void reset();

    /**
     * @brief 重置仅统计信息
     */
    void resetStats();

    /**
     * @brief 检查过滤器是否已初始化
     * @return true 如果已初始化
     */
    bool isInitialized() const;

    /**
     * @brief 获取当前内存使用量估计
     * @return 内存使用量(MB)
     */
    double getMemoryUsageMB() const;

    /**
     * @brief 获取最近一次过滤的动态点云（用于可视化调试）
     * @return 动态点云指针，如果没有动态点则返回空点云
     */
    CloudType::Ptr getLastDynamicPoints() const;

    /**
     * @brief 设置调试模式
     * @param enable 是否启用调试模式
     */
    void setDebugMode(bool enable);

private:
    // 配置和状态
    DynamicFilterConfig config_;           ///< 过滤器配置
    mutable std::mutex mutex_;             ///< 线程安全锁
    bool initialized_;                     ///< 初始化状态
    bool debug_mode_;                      ///< 调试模式

    // 历史数据管理
    std::deque<CloudType::Ptr> cloud_history_;           ///< 点云历史
    std::deque<double> timestamp_history_;               ///< 时间戳历史
    std::deque<std::vector<PointTemporalInfo>> temporal_info_history_; ///< 时间信息历史

    // 计算组件
    pcl::KdTreeFLANN<PointType> kdtree_;                ///< KD树用于邻域搜索
    pcl::VoxelGrid<PointType> voxel_filter_;            ///< 体素滤波器
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator_; ///< 法向量估计器
    pcl::search::KdTree<PointType>::Ptr search_tree_;   ///< 搜索树

    // 统计信息
    FilterStats last_stats_;               ///< 最近统计信息
    std::chrono::high_resolution_clock::time_point last_process_time_; ///< 上次处理时间
    CloudType::Ptr last_dynamic_cloud_;   ///< 最近一次过滤的动态点云（用于可视化）

    // ================ 核心算法接口 ================

    /**
     * @brief 更新历史数据
     * @param cloud 当前点云
     * @param timestamp 时间戳
     */
    void updateHistory(const CloudType::Ptr& cloud, double timestamp);

    /**
     * @brief 检测动态点
     * @param cloud 输入点云
     * @param timestamp 时间戳
     * @return 动态点掩码（1表示动态点）
     */
    std::vector<uint8_t> detectDynamicPoints(const CloudType::Ptr& cloud, double timestamp);

    // ================ 时间一致性分析 ================

    /**
     * @brief 计算点云的时间信息
     * @param cloud 当前点云
     * @param timestamp 时间戳
     * @return 每个点的时间信息
     */
    std::vector<PointTemporalInfo> computeTemporalInfo(const CloudType::Ptr& cloud,
                                                       double timestamp);

    /**
     * @brief 计算点的稳定性评分
     * @param info 点的时间信息
     * @return 稳定性评分 [0, 1]
     */
    float computeStabilityScore(const PointTemporalInfo& info) const;

    /**
     * @brief 分析运动模式
     * @param info 点的时间信息
     * @return 运动特征（速度、加速度等）
     */
    struct MotionFeatures {
        float velocity;           ///< 平均速度(m/s)
        float acceleration;       ///< 平均加速度(m/s²)
        float direction_consistency; ///< 方向一致性 [0, 1]
    };
    MotionFeatures analyzeMotionPattern(const PointTemporalInfo& info) const;

    // ================ 几何特征分析 ================

    /**
     * @brief 几何一致性检查
     * @param cloud 输入点云
     * @param dynamic_candidates 动态候选点
     * @return 最终动态点掩码
     */
    std::vector<uint8_t> geometricConsistencyCheck(const CloudType::Ptr& cloud,
                                                  const std::vector<uint8_t>& dynamic_candidates);

    /**
     * @brief 计算点云法向量
     * @param cloud 输入点云
     * @return 法向量点云
     */
    pcl::PointCloud<pcl::Normal>::Ptr computeNormals(const CloudType::Ptr& cloud);

    /**
     * @brief 分析局部密度特征
     * @param cloud 输入点云
     * @param point_idx 点索引
     * @return 密度特征
     */
    struct DensityFeatures {
        float local_density;      ///< 局部密度(点/m³)
        float density_variance;   ///< 密度方差
        float smoothness;         ///< 表面平滑度
    };
    DensityFeatures analyzeDensityFeatures(const CloudType::Ptr& cloud, 
                                          size_t point_idx) const;

    // ================ 工具函数 ================

    /**
     * @brief 降采样点云
     * @param cloud 输入点云
     * @return 降采样后的点云
     */
    CloudType::Ptr downsampleCloud(const CloudType::Ptr& cloud);

    /**
     * @brief 自适应降采样
     * @param cloud 输入点云
     * @param target_size 目标点数
     * @return 降采样后的点云
     */
    CloudType::Ptr adaptiveDownsample(const CloudType::Ptr& cloud, size_t target_size);

    /**
     * @brief 清理过期的历史数据
     * @param current_timestamp 当前时间戳
     */
    void cleanOldHistory(double current_timestamp);

    /**
     * @brief 在历史点云中查找对应点
     * @param current_cloud 当前点云
     * @param reference_cloud 参考点云
     * @return 对应点索引列表
     */
    std::vector<int> findCorrespondingPoints(const CloudType::Ptr& current_cloud,
                                            const CloudType::Ptr& reference_cloud) const;

    /**
     * @brief 根据掩码创建过滤后的点云
     * @param input_cloud 输入点云
     * @param dynamic_mask 动态点掩码
     * @return 过滤后的点云
     */
    CloudType::Ptr createFilteredCloud(const CloudType::Ptr& input_cloud,
                                      const std::vector<uint8_t>& dynamic_mask) const;

    // ================ 性能和调试 ================

    /**
     * @brief 更新统计信息
     * @param total_points 总点数
     * @param dynamic_points 动态点数
     * @param processing_time_ms 处理时间
     */
    void updateStatistics(size_t total_points, size_t dynamic_points, double processing_time_ms);

    /**
     * @brief 验证输入参数
     * @param cloud 输入点云
     * @param timestamp 时间戳
     * @return true 如果输入有效
     */
    bool validateInput(const CloudType::Ptr& cloud, double timestamp) const;

    /**
     * @brief 输出调试信息
     * @param message 调试消息
     */
    void debugLog(const std::string& message) const;

    /**
     * @brief 计算内存使用量
     * @return 内存使用量(字节)
     */
    size_t calculateMemoryUsage() const;
};

} // namespace localizers