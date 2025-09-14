#include "lidar_processor.h"

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    
    // 初始化优化的地图管理结构
    m_global_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_global_ikdtree->set_downsample_param(m_config.map_resolution * 2.0); // 全局地图使用2倍分辨率
    m_cached_global_map.reset(new CloudType);
    m_cache_timestamp = std::chrono::steady_clock::now() - std::chrono::milliseconds(CACHE_VALID_MS + 1);
    
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    trimCloudMap();
    m_kf->update();
    incrCloudMap();
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
            }
        }
    }

    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }
    if (effect_feat_num < 1)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_wi);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * norm_p.intensity;
    }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}

CloudType::Ptr LidarProcessor::getGlobalMap()
{
    CloudType::Ptr global_map(new CloudType);
    
    // 检查ikd-tree状态
    if (!m_ikdtree || !m_ikdtree->Root_Node) {
        return global_map;  // 返回空点云
    }
    
    // 使用ikd-tree的flatten方法获取所有点
    typename KD_TREE<PointType>::PointVector all_points;
    m_ikdtree->flatten(m_ikdtree->Root_Node, all_points, delete_point_storage_set::NOT_RECORD);
    
    // 预分配内存以提升性能
    global_map->points.reserve(all_points.size());
    
    // 将PointVector转换为CloudType，应用质量滤波
    for (const auto& point : all_points) {
        // 简单的质量滤波：排除异常点
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            float distance = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if (distance > 0.1 && distance < 80.0) {  // 合理的距离范围
                global_map->points.push_back(point);
            }
        }
    }
    
    global_map->width = global_map->points.size();
    global_map->height = 1;
    global_map->is_dense = true;
    
    return global_map;
}

// 优化的全局地图获取方法 - 使用缓存和分层策略
CloudType::Ptr LidarProcessor::getOptimizedGlobalMap(float resolution_scale)
{
    // 检查缓存是否有效
    if (isCacheValid() && resolution_scale == 1.0) {
        return m_cached_global_map;
    }
    
    CloudType::Ptr optimized_map(new CloudType);
    
    // 检查ikd-tree状态
    if (!m_ikdtree || !m_ikdtree->Root_Node) {
        return optimized_map;
    }
    
    // 如果树的大小发生显著变化或缓存失效，重新构建
    size_t current_tree_size = m_ikdtree->size();
    bool need_rebuild = (current_tree_size - m_last_tree_size) > (m_last_tree_size * 0.1) || !isCacheValid();
    
    if (need_rebuild && resolution_scale == 1.0) {
        updateGlobalMapCache();
        return m_cached_global_map;
    }
    
    // 对于非标准分辨率请求，实时生成
    typename KD_TREE<PointType>::PointVector all_points;
    m_ikdtree->flatten(m_ikdtree->Root_Node, all_points, delete_point_storage_set::NOT_RECORD);
    
    // 预分配内存
    optimized_map->points.reserve(all_points.size() / (resolution_scale * resolution_scale));
    
    // 智能采样策略 - 根据距离和密度动态调整
    const State &state = m_kf->x();
    Eigen::Vector3d current_pos = state.t_wi + state.r_wi * state.t_il;
    
    float adaptive_threshold = 0.1 * resolution_scale;
    size_t skip_counter = 0;
    size_t skip_ratio = static_cast<size_t>(resolution_scale * resolution_scale);
    
    for (const auto& point : all_points) {
        // 智能质量滤波
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        
        float distance_to_sensor = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        float distance_to_robot = sqrt(pow(point.x - current_pos.x(), 2) + 
                                      pow(point.y - current_pos.y(), 2) + 
                                      pow(point.z - current_pos.z(), 2));
        
        // 距离滤波 + 动态采样
        if (distance_to_sensor > 0.1 && distance_to_sensor < 80.0) {
            // 根据距离机器人的远近调整采样率
            size_t dynamic_skip = distance_to_robot > 50.0 ? skip_ratio * 2 : skip_ratio;
            
            if (skip_counter++ % dynamic_skip == 0) {
                optimized_map->points.push_back(point);
            }
        }
    }
    
    optimized_map->width = optimized_map->points.size();
    optimized_map->height = 1;
    optimized_map->is_dense = true;
    
    return optimized_map;
}

// 更新全局地图缓存
void LidarProcessor::updateGlobalMapCache()
{
    if (!m_ikdtree || !m_ikdtree->Root_Node) {
        return;
    }
    
    typename KD_TREE<PointType>::PointVector all_points;
    m_ikdtree->flatten(m_ikdtree->Root_Node, all_points, delete_point_storage_set::NOT_RECORD);
    
    // 更新缓存
    m_cached_global_map->clear();
    m_cached_global_map->points.reserve(all_points.size());
    
    // 应用质量滤波和智能采样
    for (const auto& point : all_points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            float distance = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if (distance > 0.1 && distance < 80.0) {
                m_cached_global_map->points.push_back(point);
            }
        }
    }
    
    m_cached_global_map->width = m_cached_global_map->points.size();
    m_cached_global_map->height = 1;
    m_cached_global_map->is_dense = true;
    
    // 更新缓存时间戳和树大小记录
    m_cache_timestamp = std::chrono::steady_clock::now();
    m_last_tree_size = m_ikdtree->size();
}

// 检查缓存是否有效
bool LidarProcessor::isCacheValid() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_cache_timestamp);
    return duration.count() < CACHE_VALID_MS && !m_cached_global_map->empty();
}

// 优化地图内存使用
void LidarProcessor::optimizeMapMemory()
{
    if (!m_ikdtree || !m_ikdtree->Root_Node) {
        return;
    }
    
    // 获取当前机器人位置
    const State &state = m_kf->x();
    Eigen::Vector3d robot_pos = state.t_wi + state.r_wi * state.t_il;
    
    // 定义内存清理区域：距离机器人超过100米的旧数据
    BoxPointType cleanup_box;
    float cleanup_radius = 100.0;
    
    // 创建清理区域 (示例：清理远离机器人的区域)
    std::vector<BoxPointType> boxes_to_remove;
    
    // 这里可以根据具体需求添加更复杂的清理策略
    // 比如基于时间戳、访问频率等
    
    // 执行清理
    if (!boxes_to_remove.empty()) {
        m_ikdtree->Delete_Point_Boxes(boxes_to_remove);
        
        // 清理后重建缓存
        m_cache_timestamp = std::chrono::steady_clock::now() - std::chrono::milliseconds(CACHE_VALID_MS + 1);
    }
}

// 获取地图内存使用情况
size_t LidarProcessor::getMapMemoryUsage() const
{
    size_t memory_usage = 0;
    
    if (m_ikdtree) {
        memory_usage += m_ikdtree->size() * sizeof(PointType);
    }
    
    if (m_cached_global_map) {
        memory_usage += m_cached_global_map->size() * sizeof(PointType);
    }
    
    return memory_usage;
}

// 点云降采样方法
CloudType::Ptr LidarProcessor::downsampleMap(CloudType::Ptr input, float leaf_size) const
{
    if (!input || input->empty()) {
        return CloudType::Ptr(new CloudType);
    }
    
    // 安全的体素化：检查体素尺寸和点云范围
    Eigen::Vector4f min_pt, max_pt;
    
    // 手动计算点云边界
    if (input->points.empty()) {
        return CloudType::Ptr(new CloudType);
    }
    
    min_pt[0] = max_pt[0] = input->points[0].x;
    min_pt[1] = max_pt[1] = input->points[0].y;
    min_pt[2] = max_pt[2] = input->points[0].z;
    
    for (const auto& point : input->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            min_pt[0] = std::min(min_pt[0], point.x);
            max_pt[0] = std::max(max_pt[0], point.x);
            min_pt[1] = std::min(min_pt[1], point.y);
            max_pt[1] = std::max(max_pt[1], point.y);
            min_pt[2] = std::min(min_pt[2], point.z);
            max_pt[2] = std::max(max_pt[2], point.z);
        }
    }
    
    double range_x = max_pt[0] - min_pt[0];
    double range_y = max_pt[1] - min_pt[1];
    double range_z = max_pt[2] - min_pt[2];
    double max_range = std::max({range_x, range_y, range_z});
    
    // 防止整数溢出：确保体素数量不超过安全阈值
    double safe_leaf_size = leaf_size;
    const int32_t MAX_VOXELS_PER_DIM = 10000;  // 安全阈值
    
    if (max_range / safe_leaf_size > MAX_VOXELS_PER_DIM) {
        safe_leaf_size = max_range / MAX_VOXELS_PER_DIM;
        std::cout << "[WARN] Voxel size too small, adjusted from " 
                  << leaf_size << " to " << safe_leaf_size << std::endl;
    }
    
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(safe_leaf_size, safe_leaf_size, safe_leaf_size);
    
    CloudType::Ptr downsampled(new CloudType);
    voxel_grid.filter(*downsampled);
    
    return downsampled;
}