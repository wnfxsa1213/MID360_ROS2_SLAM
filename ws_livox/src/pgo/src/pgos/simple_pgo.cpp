#include "simple_pgo.h"
#include <limits>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(m_config.icp_max_distance);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);
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
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

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
    return ret;
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < m_key_poses.size() - 1; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return;

    // 收集候选（按时间间隔与索引间隔过滤）
    std::vector<int> cand_idxs;
    cand_idxs.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
        int idx = ids[i];
        if (idx < 0 || idx >= static_cast<int>(m_key_poses.size()-1)) continue;
        if (static_cast<int>(cur_idx) - idx < m_config.min_index_separation) continue; // 索引间隔
        if (std::abs(last_item.time - m_key_poses[idx].time) <= m_config.loop_time_tresh) continue; // 时间隔离
        cand_idxs.push_back(idx);
        if (static_cast<int>(cand_idxs.size()) >= m_config.max_candidates) break;
    }

    if (cand_idxs.empty()) return;

    // 源与目标点云（源仅取当前帧/局部子图）
    CloudType::Ptr source_cloud = getSubMap(static_cast<int>(cur_idx), 0, m_config.submap_resolution);

    double best_score = std::numeric_limits<double>::infinity();
    int best_idx = -1;
    M4F best_tf = M4F::Identity();
    double best_inlier_ratio = 0.0;

    for (int loop_idx : cand_idxs)
    {
        if (isRecentPair(loop_idx, cur_idx)) continue; // 去重

        CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
        CloudType::Ptr aligned(new CloudType);

        m_icp.setMaxCorrespondenceDistance(m_config.icp_max_distance);
        m_icp.setInputSource(source_cloud);
        m_icp.setInputTarget(target_cloud);
        m_icp.align(*aligned);

        if (!m_icp.hasConverged()) continue;

        double fitness = m_icp.getFitnessScore();
        if (fitness > m_config.loop_score_tresh) continue; // 粗滤

        // 计算内点比例（几何验证）
        double inlier_ratio = computeInlierRatio(aligned, target_cloud, m_config.inlier_threshold);
        if (inlier_ratio < m_config.min_inlier_ratio) continue;

        // 组合评分：更小更好（用于噪声建模）
        double combined = fitness / std::max(1e-3, inlier_ratio);
        if (combined < best_score)
        {
            best_score = combined;
            best_idx = loop_idx;
            best_tf = m_icp.getFinalTransformation();
            best_inlier_ratio = inlier_ratio;
        }
    }

    if (best_idx == -1) return;

    // 构造并缓存最优回环对
    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = static_cast<size_t>(best_idx);
    one_pair.score = std::max(1e-4, best_score); // 用于方差缩放
    M3D r_refined = best_tf.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = best_tf.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + best_tf.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[best_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[best_idx].r_global.transpose() * (t_refined - m_key_poses[best_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
    recordRecentPair(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
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
