#include "simple_pgo.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iterator>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = m_config.isam_relinearize_threshold;
    isam2_params.relinearizeSkip = m_config.isam_relinearize_skip;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(m_config.icp_max_iterations);
    m_icp.setMaxCorrespondenceDistance(m_config.icp_max_distance);
    m_icp.setTransformationEpsilon(m_config.icp_transformation_epsilon);
    m_icp.setEuclideanFitnessEpsilon(m_config.icp_euclidean_fitness_epsilon);
    m_icp.setRANSACIterations(0);

    m_key_pose_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    resetKeyPoseKdTree();
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    m_kdtree_dirty = true;
    m_new_key_poses_since_rebuild++;
    invalidateSubmapCache();
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int clamped_half = clampHalfRange(half_range, m_key_poses.size());
    int min_idx = std::max(0, idx - clamped_half);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + clamped_half);

    SubmapCacheKey key{idx, clamped_half, quantizeResolution(resolution)};

    if (m_config.submap_cache_size > 0)
    {
        auto cache_it = m_submap_cache.find(key);
        if (cache_it != m_submap_cache.end())
        {
            // LRU更新：将命中条目挪到链表头部
            m_submap_lru.splice(m_submap_lru.begin(), m_submap_lru, cache_it->second.lru_it);
            cache_it->second.lru_it = m_submap_lru.begin();
            return cache_it->second.cloud;
        }
    }

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {
        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }

    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }

    if (m_config.submap_cache_size > 0)
    {
        m_submap_lru.push_front(key);
        auto lru_it = m_submap_lru.begin();
        m_submap_cache.emplace(key, SubmapCacheEntry{ret, lru_it});

        while (m_submap_cache.size() > m_config.submap_cache_size)
        {
            const SubmapCacheKey& evict_key = m_submap_lru.back();
            m_submap_cache.erase(evict_key);
            m_submap_lru.pop_back();
        }
    }

    return ret;
}

SimplePGO::LoopEvaluationResult SimplePGO::evaluateLoopCandidate(size_t cur_idx,
                                                                 int loop_idx,
                                                                 const CloudType::Ptr& source_cloud,
                                                                 double score_relax_factor,
                                                                 double inlier_relax_factor)
{
    LoopEvaluationResult result;
    if (loop_idx < 0 || loop_idx >= static_cast<int>(m_key_poses.size())) {
        return result;
    }

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr aligned(new CloudType);

    m_icp.setMaxCorrespondenceDistance(m_config.icp_max_distance);
    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*aligned);

    if (!m_icp.hasConverged()) {
        result.failure_reason = LoopEvaluationResult::FailureReason::ICP_NOT_CONVERGED;
        return result;
    }

    result.fitness = m_icp.getFitnessScore();
    result.inlier_ratio = computeInlierRatio(aligned, target_cloud, m_config.inlier_threshold);
    result.combined_score = result.fitness / std::max(1e-3, result.inlier_ratio);

    double fitness_threshold = m_config.loop_score_tresh * std::max(1.0, score_relax_factor);
    if (result.fitness > fitness_threshold) {
        result.failure_reason = LoopEvaluationResult::FailureReason::FITNESS_THRESHOLD;
        return result;
    }

    double min_inlier = std::clamp(m_config.min_inlier_ratio * inlier_relax_factor, 0.0, 1.0);
    if (result.inlier_ratio < min_inlier) {
        result.failure_reason = LoopEvaluationResult::FailureReason::INLIER_THRESHOLD;
        return result;
    }

    LoopPair pair;
    pair.source_id = cur_idx;
    pair.target_id = static_cast<size_t>(loop_idx);

    const M4F tf = m_icp.getFinalTransformation();
    M3D r_refined = tf.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = tf.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global +
                    tf.block<3, 1>(0, 3).cast<double>();
    pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
    pair.inlier_ratio = result.inlier_ratio;
    pair.fitness = result.fitness;
    pair.combined_score = result.combined_score;
    pair.score = std::max(1e-4, result.combined_score);

    result.pair = pair;
    result.success = true;
    result.failure_reason = LoopEvaluationResult::FailureReason::NONE;
    return result;
}

void SimplePGO::registerLoopRetryCandidate(int loop_idx,
                                           double current_time,
                                           const LoopEvaluationResult& eval)
{
    if (m_config.loop_retry_max_attempts <= 0 || loop_idx < 0) {
        return;
    }
    if (eval.failure_reason == LoopEvaluationResult::FailureReason::ICP_NOT_CONVERGED ||
        eval.failure_reason == LoopEvaluationResult::FailureReason::NONE) {
        return;
    }

    const size_t target_id = static_cast<size_t>(loop_idx);
    auto existing = std::find_if(m_loop_retry_queue.begin(), m_loop_retry_queue.end(),
                                 [target_id](const LoopRetryEntry& entry) {
                                     return entry.target_id == target_id;
                                 });
    if (existing != m_loop_retry_queue.end()) {
        existing->last_attempt_time = current_time;
        existing->last_recorded_score = std::min(existing->last_recorded_score, eval.combined_score);
        return;
    }

    LoopRetryEntry entry;
    entry.target_id = target_id;
    entry.attempts = 1;
    entry.last_attempt_time = current_time;
    entry.last_recorded_score = eval.combined_score;
    m_loop_retry_queue.push_back(entry);

    while (m_loop_retry_queue.size() > m_config.loop_retry_capacity) {
        m_loop_retry_queue.pop_front();
    }
}

bool SimplePGO::tryRetryCandidates(size_t cur_idx,
                                   const CloudType::Ptr& source_cloud,
                                   double current_time,
                                   LoopPair& out_pair,
                                   double& out_score)
{
    if (m_loop_retry_queue.empty()) {
        return false;
    }

    for (auto it = m_loop_retry_queue.begin(); it != m_loop_retry_queue.end();) {
        if (it->target_id >= m_key_poses.size()) {
            it = m_loop_retry_queue.erase(it);
            continue;
        }

        if ((current_time - it->last_attempt_time) < m_config.loop_retry_interval) {
            ++it;
            continue;
        }

        auto eval = evaluateLoopCandidate(cur_idx,
                                          static_cast<int>(it->target_id),
                                          source_cloud,
                                          m_config.loop_retry_score_relax,
                                          m_config.loop_retry_inlier_relax);
        it->last_attempt_time = current_time;
        it->attempts = std::min(it->attempts + 1, m_config.loop_retry_max_attempts);

        if (eval.success) {
            out_pair = eval.pair;
            out_score = eval.combined_score;
            m_loop_retry_queue.erase(it);
            return true;
        }

        const bool give_up = eval.failure_reason == LoopEvaluationResult::FailureReason::ICP_NOT_CONVERGED ||
                             it->attempts >= m_config.loop_retry_max_attempts;
        if (give_up) {
            it = m_loop_retry_queue.erase(it);
        } else {
            it->last_recorded_score = std::min(it->last_recorded_score, eval.combined_score);
            ++it;
        }
    }

    return false;
}

void SimplePGO::appendLoopMetrics(const LoopPair& pair)
{
    LoopQualityMetrics metrics;
    metrics.inlier_ratio = pair.inlier_ratio;
    metrics.fitness = pair.fitness;
    metrics.combined_score = pair.combined_score;
    m_recent_loop_metrics.push_back(metrics);
    while (m_recent_loop_metrics.size() > kRecentLoopMetricsCapacity) {
        m_recent_loop_metrics.pop_front();
    }
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    updateKeyPoseKdTree(cur_idx);
    if (m_key_pose_cloud->empty())
        return;

    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = m_key_pose_kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return;

    // 收集候选（按时间间隔与索引间隔过滤）
    std::vector<int> candidate_indices;
    candidate_indices.reserve(ids.size());
    for (int kd_index : ids)
    {
        if (kd_index < 0 || static_cast<size_t>(kd_index) >= m_kdtree_index_map.size()) continue;
        candidate_indices.push_back(m_kdtree_index_map[kd_index]);
    }

    std::vector<int> cand_idxs;
    cand_idxs.reserve(candidate_indices.size());
    auto is_valid_candidate = [&](int idx) {
        if (idx < 0 || idx >= static_cast<int>(m_key_poses.size() - 1)) return false;
        if (static_cast<int>(cur_idx) - idx < m_config.min_index_separation) return false;
        if (std::abs(last_item.time - m_key_poses[idx].time) <= m_config.loop_time_tresh) return false;
        return true;
    };

    std::copy_if(candidate_indices.begin(),
                 candidate_indices.end(),
                 std::back_inserter(cand_idxs),
                 is_valid_candidate);

    if (m_config.max_candidates > 0 && cand_idxs.size() > static_cast<size_t>(m_config.max_candidates))
    {
        cand_idxs.resize(static_cast<size_t>(m_config.max_candidates));
    }

    if (cand_idxs.empty()) return;

    // 源与目标点云（源仅取当前帧/局部子图）
    CloudType::Ptr source_cloud = getSubMap(static_cast<int>(cur_idx), 0, m_config.submap_resolution);

    double best_score = std::numeric_limits<double>::infinity();
    bool has_best = false;
    LoopPair best_pair;
    const double current_time = last_item.time;

    for (int loop_idx : cand_idxs)
    {
        if (isRecentPair(loop_idx, cur_idx)) continue; // 去重

        auto eval = evaluateLoopCandidate(cur_idx, loop_idx, source_cloud, 1.0, 1.0);
        if (eval.success)
        {
            if (!has_best || eval.combined_score < best_score)
            {
                best_score = eval.combined_score;
                best_pair = eval.pair;
                has_best = true;
            }
            auto erase_it = std::remove_if(m_loop_retry_queue.begin(),
                                           m_loop_retry_queue.end(),
                                           [target = eval.pair.target_id](const LoopRetryEntry& entry) {
                                               return entry.target_id == target;
                                           });
            if (erase_it != m_loop_retry_queue.end()) {
                m_loop_retry_queue.erase(erase_it, m_loop_retry_queue.end());
            }
        }
        else
        {
            registerLoopRetryCandidate(loop_idx, current_time, eval);
        }
    }

    if (!has_best)
    {
        if (!tryRetryCandidates(cur_idx, source_cloud, current_time, best_pair, best_score))
        {
            return;
        }
        has_best = true;
    }

    if (!has_best) return;

    m_cache_pairs.push_back(best_pair);
    m_history_pairs.emplace_back(best_pair.target_id, best_pair.source_id);
    recordRecentPair(best_pair.target_id, best_pair.source_id);
}

bool SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            appendLoopMetrics(pair);
            gtsam::Vector6 variances;
            variances.head<3>() = gtsam::Vector3::Ones() * (pair.score * 0.5);
            variances.tail<3>() = gtsam::Vector3::Ones() * (pair.score * 2.0);
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(variances)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
    m_kdtree_dirty = true;
    resetKeyPoseKdTree();
    invalidateSubmapCache();
    return has_loop;
}

bool SimplePGO::isRecentPair(size_t a, size_t b) const
{
    // 检查是否已存在相同顺序或反向的近似配对，避免重复添加
    for (const auto& p : m_recent_added_pairs)
    {
        if ((p.first == a && p.second == b) || (p.first == b && p.second == a))
            return true;
    }
    return false;
}

void SimplePGO::recordRecentPair(size_t a, size_t b)
{
    m_recent_added_pairs.emplace_back(a, b);
    if (m_recent_added_pairs.size() > kRecentPairsCapacity)
    {
        size_t shrink_target = kRecentPairsCapacity / 2;
        shrink_target = std::max<size_t>(shrink_target, 1);
        while (m_recent_added_pairs.size() > shrink_target)
        {
            m_recent_added_pairs.pop_front();
        }
    }
}

double SimplePGO::computeInlierRatio(const CloudType::Ptr& src_aligned,
                                     const CloudType::Ptr& tgt,
                                     double dist_thresh) const
{
    if (!src_aligned || !tgt || src_aligned->empty() || tgt->empty()) return 0.0;
    pcl::KdTreeFLANN<PointType> kdt;
    kdt.setInputCloud(tgt);
    int inliers = 0;
    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist(1);
    for (const auto& pt : src_aligned->points)
    {
        if (kdt.nearestKSearch(pt, 1, nn_idx, nn_dist) > 0)
        {
            if (nn_dist[0] <= dist_thresh * dist_thresh) inliers++;
        }
    }
    return static_cast<double>(inliers) / static_cast<double>(src_aligned->size());
}

void SimplePGO::updateKeyPoseKdTree(size_t current_idx)
{
    if (m_key_poses.size() <= 1)
    {
        resetKeyPoseKdTree();
        return;
    }

    size_t target_count = m_key_poses.size() - 1; // 不包含当前帧
    if (m_kdtree_populated_count > target_count)
    {
        resetKeyPoseKdTree();
    }

    bool need_rebuild = m_kdtree_dirty || m_key_pose_cloud->empty() || (m_kdtree_populated_count == 0);
    if (!need_rebuild)
    {
        if (m_new_key_poses_since_rebuild >= static_cast<size_t>(m_config.loop_kdtree_update_batch))
        {
            need_rebuild = true;
        }
        else if (current_idx >= static_cast<size_t>(m_config.min_index_separation))
        {
            size_t required_index = current_idx - static_cast<size_t>(m_config.min_index_separation);
            if (m_kdtree_populated_count <= required_index)
            {
                need_rebuild = true;
            }
        }
    }

    if (!need_rebuild)
    {
        return;
    }

    m_key_pose_cloud->clear();
    m_kdtree_index_map.clear();
    m_key_pose_cloud->reserve(target_count);
    m_kdtree_index_map.reserve(target_count);

    for (size_t i = 0; i < target_count; ++i)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        m_key_pose_cloud->push_back(pt);
        m_kdtree_index_map.push_back(static_cast<int>(i));
    }

    m_key_pose_kdtree.setInputCloud(m_key_pose_cloud);
    m_kdtree_populated_count = target_count;
    m_new_key_poses_since_rebuild = 0;
    m_kdtree_dirty = false;
}

void SimplePGO::resetKeyPoseKdTree()
{
    if (!m_key_pose_cloud)
    {
        m_key_pose_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }
    m_key_pose_cloud->clear();
    m_kdtree_index_map.clear();
    m_key_pose_kdtree.setInputCloud(m_key_pose_cloud);
    m_kdtree_populated_count = 0;
    m_new_key_poses_since_rebuild = 0;
    m_kdtree_dirty = false;
}

void SimplePGO::invalidateSubmapCache()
{
    m_submap_cache.clear();
    m_submap_lru.clear();
}

int SimplePGO::quantizeResolution(double resolution) const
{
    double res = std::max(0.0, resolution);
    return static_cast<int>(std::round(res * 1000.0));
}

int SimplePGO::clampHalfRange(int half_range, size_t pose_count)
{
    if (half_range <= 0 || pose_count <= 1)
    {
        return 0;
    }
    int max_allowed = static_cast<int>(pose_count) - 1;
    if (max_allowed < 0)
    {
        return 0;
    }
    return std::min(half_range, max_allowed);
}

void SimplePGO::applyOptimizedPoses(const std::vector<size_t>& indices, const std::vector<PoseUpdate>& poses)
{
    if (indices.size() != poses.size())
    {
        return;
    }

    for (size_t i = 0; i < indices.size(); ++i)
    {
        size_t idx = indices[i];
        if (idx >= m_key_poses.size())
        {
            continue;
        }
        m_key_poses[idx].r_global = poses[i].r;
        m_key_poses[idx].t_global = poses[i].t;
    }

    if (!m_key_poses.empty())
    {
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        m_r_offset = last_item.r_global * last_item.r_local.transpose();
        m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
    }

    m_kdtree_dirty = true;
    resetKeyPoseKdTree();
    invalidateSubmapCache();
}
