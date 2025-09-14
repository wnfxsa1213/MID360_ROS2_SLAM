#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }
    
    // 获取全局累积地图点云
    CloudType::Ptr getGlobalMap();
    
    // 优化的地图管理接口
    CloudType::Ptr getOptimizedGlobalMap(float resolution_scale = 1.0);
    void optimizeMapMemory();
    size_t getMapMemoryUsage() const;

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;
    
    // 优化的地图管理数据结构
    std::shared_ptr<KD_TREE<PointType>> m_global_ikdtree;  // 全局低分辨率地图
    CloudType::Ptr m_cached_global_map;                     // 缓存的全局地图
    std::chrono::steady_clock::time_point m_cache_timestamp; // 缓存时间戳
    static constexpr int CACHE_VALID_MS = 1000;             // 缓存有效时间1秒
    size_t m_last_tree_size = 0;                           // 上次树的大小
    
    // 优化方法
    void updateGlobalMapCache();
    bool isCacheValid() const;
    CloudType::Ptr downsampleMap(CloudType::Ptr input, float leaf_size) const;
};
