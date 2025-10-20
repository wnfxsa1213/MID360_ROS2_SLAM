#pragma once
#include "commons.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <deque>
#include <list>
#include <limits>
#include <unordered_map>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
    double inlier_ratio = 0.0;
    double fitness = 0.0;
    double combined_score = 0.0;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_time_tresh = 60.0;
    double loop_score_tresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;
    // 高级几何验证与约束管理新增参数（有默认值，YAML可选覆盖）
    double min_inlier_ratio = 0.30;      // ICP后几何内点比例最小阈值
    double icp_max_distance = 2.0;       // ICP最大对应距离（提升鲁棒性）
    int max_candidates = 5;              // 每次检测最多验证的候选数量
    int min_index_separation = 10;       // 关键帧索引最小间隔，避免近邻伪回环
    double inlier_threshold = 0.30;      // 计算内点时的距离阈值(米)
    int loop_kdtree_update_batch = 10;   // 累计这么多新关键帧后再重建KD树
    size_t submap_cache_size = 32;       // 子图缓存最大容量
    int icp_max_iterations = 50;         // ICP最大迭代次数
    double icp_transformation_epsilon = 1e-6; // ICP位姿收敛阈值
    double icp_euclidean_fitness_epsilon = 1e-6; // ICP误差收敛阈值
    double isam_relinearize_threshold = 0.01; // iSAM2 relinearize 阈值
    int isam_relinearize_skip = 1;            // iSAM2 relinearize 间隔
    int loop_retry_max_attempts = 3;          // 回环重试最大次数
    double loop_retry_interval = 5.0;         // 重试时间间隔(秒)
    size_t loop_retry_capacity = 32;          // 重试候选缓存上限
    double loop_retry_score_relax = 1.5;      // 重试时得分阈值放宽倍数
    double loop_retry_inlier_relax = 0.7;     // 重试时内点比例放宽倍数
};

struct PoseUpdate
{
    M3D r;
    V3D t;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    bool smoothAndUpdate();

    void applyOptimizedPoses(const std::vector<size_t>& indices,
                             const std::vector<PoseUpdate>& poses);

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }
    struct LoopQualityMetrics {
        double inlier_ratio = 0.0;
        double fitness = 0.0;
        double combined_score = 0.0;
    };
    const std::deque<LoopQualityMetrics>& recentLoopMetrics() const { return m_recent_loop_metrics; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

private:
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    std::deque<std::pair<size_t,size_t>> m_recent_added_pairs; // 约束去重辅助
    static constexpr size_t kRecentPairsCapacity = 1024;
    static constexpr size_t kRecentLoopMetricsCapacity = 64;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;
    struct LoopEvaluationResult {
        enum class FailureReason {
            NONE,
            ICP_NOT_CONVERGED,
            FITNESS_THRESHOLD,
            INLIER_THRESHOLD
        };
        bool success = false;
        double combined_score = std::numeric_limits<double>::infinity();
        double fitness = std::numeric_limits<double>::infinity();
        double inlier_ratio = 0.0;
        LoopPair pair{};
        FailureReason failure_reason = FailureReason::NONE;
    };
    struct LoopRetryEntry {
        size_t target_id = 0;
        int attempts = 0;
        double last_attempt_time = 0.0;
        double last_recorded_score = std::numeric_limits<double>::infinity();
    };

    std::deque<LoopRetryEntry> m_loop_retry_queue;
    std::deque<LoopQualityMetrics> m_recent_loop_metrics;

    // 几何验证辅助
    bool isRecentPair(size_t a, size_t b) const;
    void recordRecentPair(size_t a, size_t b);
    double computeInlierRatio(const CloudType::Ptr& src_aligned,
                              const CloudType::Ptr& tgt,
                              double dist_thresh) const;
    void registerLoopRetryCandidate(int loop_idx,
                                    double current_time,
                                    const LoopEvaluationResult& eval);
    bool tryRetryCandidates(size_t cur_idx,
                            const CloudType::Ptr& source_cloud,
                            double current_time,
                            LoopPair& out_pair,
                            double& out_score);
    LoopEvaluationResult evaluateLoopCandidate(size_t cur_idx,
                                               int loop_idx,
                                               const CloudType::Ptr& source_cloud,
                                               double score_relax_factor,
                                               double inlier_relax_factor);
    void appendLoopMetrics(const LoopPair& pair);

    // 关键帧位姿KD树缓存
    void updateKeyPoseKdTree(size_t current_idx);
    void resetKeyPoseKdTree();

    struct SubmapCacheKey {
        int idx;
        int half_range;
        int resolution_mm;

        bool operator==(const SubmapCacheKey& other) const noexcept {
            return idx == other.idx &&
                   half_range == other.half_range &&
                   resolution_mm == other.resolution_mm;
        }
    };

    struct SubmapCacheKeyHasher {
        std::size_t operator()(const SubmapCacheKey& key) const noexcept {
            std::size_t h1 = std::hash<int>{}(key.idx);
            std::size_t h2 = std::hash<int>{}(key.half_range);
            std::size_t h3 = std::hash<int>{}(key.resolution_mm);
            std::size_t seed = h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
            return seed ^ (h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2));
        }
    };

    struct SubmapCacheEntry {
        CloudType::Ptr cloud;
        std::list<SubmapCacheKey>::iterator lru_it;
    };

    void invalidateSubmapCache();
    int quantizeResolution(double resolution) const;
    static int clampHalfRange(int half_range, size_t pose_count);

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_key_pose_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> m_key_pose_kdtree;
    std::vector<int> m_kdtree_index_map;
    size_t m_kdtree_populated_count = 0;
    size_t m_new_key_poses_since_rebuild = 0;
    bool m_kdtree_dirty = false;

    std::unordered_map<SubmapCacheKey, SubmapCacheEntry, SubmapCacheKeyHasher> m_submap_cache;
    std::list<SubmapCacheKey> m_submap_lru;
};
