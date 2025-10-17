#include "dynamic_object_filter.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <execution>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace localizers {

// ================ 构造函数和初始化 ================

DynamicObjectFilter::DynamicObjectFilter(const DynamicFilterConfig& config)
    : config_(config), initialized_(false), debug_mode_(false) {
    
    if (!config_.isValid()) {
        throw std::invalid_argument("无效的动态过滤器配置参数");
    }
    
    // 初始化PCL组件
    search_tree_ = std::make_shared<pcl::search::KdTree<PointType>>();
    normal_estimator_.setSearchMethod(search_tree_);
    normal_estimator_.setKSearch(config_.min_neighbors);
    
    // 配置体素滤波器
    voxel_filter_.setLeafSize(config_.voxel_size_base, 
                             config_.voxel_size_base, 
                             config_.voxel_size_base);
    
    // 初始化统计信息
    last_stats_.reset();
    last_process_time_ = std::chrono::high_resolution_clock::now();

    // 初始化动态点云存储
    last_dynamic_cloud_ = std::make_shared<CloudType>();
    
    initialized_ = true;
    
    if (debug_mode_) {
        debugLog("动态对象过滤器初始化完成");
    }
}

// ================ 主要过滤接口 ================

CloudType::Ptr DynamicObjectFilter::filterDynamicObjects(const CloudType::Ptr& input_cloud,
                                                        double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto start_time = std::chrono::high_resolution_clock::now();
  if (debug_mode_) {
    std::ostringstream ss;
    ss << "[filter] start, input_size=" << (input_cloud ? input_cloud->size() : 0)
       << ", ts=" << std::fixed << std::setprecision(6) << timestamp;
    debugLog(ss.str());
  }
    
    // 输入验证
    if (!validateInput(input_cloud, timestamp)) {
        if (debug_mode_) {
            debugLog("输入验证失败");
        }
        return std::make_shared<CloudType>();
    }
    
  try {
    // 1. 预处理 - 降采样
    auto t0 = std::chrono::high_resolution_clock::now();
    CloudType::Ptr processed_cloud = downsampleCloud(input_cloud);
    auto t1 = std::chrono::high_resolution_clock::now();
    double t_down_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (debug_mode_) {
      std::ostringstream ss;
      ss << "[filter] downsampled_size=" << (processed_cloud ? processed_cloud->size() : 0)
         << ", time_ms=" << std::fixed << std::setprecision(2) << t_down_ms;
      debugLog(ss.str());
    }
    
    if (processed_cloud->empty()) {
      if (debug_mode_) {
        debugLog("降采样后点云为空");
      }
      return std::make_shared<CloudType>();
    }
    
    // 2. 更新历史数据
    t0 = std::chrono::high_resolution_clock::now();
    updateHistory(processed_cloud, timestamp);
    t1 = std::chrono::high_resolution_clock::now();
    double t_hist_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (debug_mode_) {
      std::ostringstream ss;
      ss << "[filter] history_frames=" << cloud_history_.size()
         << ", time_ms=" << std::fixed << std::setprecision(2) << t_hist_ms;
      debugLog(ss.str());
    }
    
    // 3. 检测动态点
    t0 = std::chrono::high_resolution_clock::now();
    std::vector<uint8_t> dynamic_mask = detectDynamicPoints(processed_cloud, timestamp);
    t1 = std::chrono::high_resolution_clock::now();
    double t_det_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (debug_mode_) {
      size_t dyn_cnt = std::count(dynamic_mask.begin(), dynamic_mask.end(), 1);
      std::ostringstream ss;
      ss << "[filter] detect_dynamic done, dyn_cnt=" << dyn_cnt
         << "/" << dynamic_mask.size()
         << ", time_ms=" << std::fixed << std::setprecision(2) << t_det_ms;
      debugLog(ss.str());
    }
    
    // 4. 创建过滤后的点云和动态点云
    t0 = std::chrono::high_resolution_clock::now();
    CloudType::Ptr filtered_cloud = createFilteredCloud(processed_cloud, dynamic_mask);
    t1 = std::chrono::high_resolution_clock::now();
    double t_make_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (debug_mode_) {
      std::ostringstream ss;
      ss << "[filter] build_filtered size=" << (filtered_cloud ? filtered_cloud->size() : 0)
         << ", time_ms=" << std::fixed << std::setprecision(2) << t_make_ms;
      debugLog(ss.str());
    }

        // 创建动态点云用于可视化调试
        last_dynamic_cloud_->clear();
        for (size_t i = 0; i < processed_cloud->size() && i < dynamic_mask.size(); ++i) {
            if (dynamic_mask[i] == 1) {  // 动态点
                last_dynamic_cloud_->push_back(processed_cloud->points[i]);
            }
        }
        last_dynamic_cloud_->header = processed_cloud->header;
        
        // 5. 更新统计信息
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double processing_time_ms = duration.count() / 1000.0;
        
        size_t dynamic_count = std::count(dynamic_mask.begin(), dynamic_mask.end(), 1);
        updateStatistics(processed_cloud->size(), dynamic_count, processing_time_ms);
        
    if (debug_mode_) {
      std::ostringstream ss;
      ss << "[filter] done, in=" << input_cloud->size()
         << ", out=" << filtered_cloud->size()
         << ", total_ms=" << std::fixed << std::setprecision(2) << processing_time_ms;
      debugLog(ss.str());
    }
        
        return filtered_cloud;
        
    } catch (const std::exception& e) {
        std::string err_msg = "过滤过程中出现异常: " + std::string(e.what());
        if (debug_mode_) {
            debugLog(err_msg);
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("dynamic_object_filter"), err_msg);
        }
        // 降级策略：返回原始点云，避免直接丢帧导致稀疏地图
        return input_cloud;
    }
}

std::future<CloudType::Ptr> DynamicObjectFilter::filterDynamicObjectsAsync(
    const CloudType::Ptr& input_cloud, double timestamp) {
    
    return std::async(std::launch::async, [this, input_cloud, timestamp]() {
        return filterDynamicObjects(input_cloud, timestamp);
    });
}

// ================ 动态点检测核心算法 ================

std::vector<uint8_t> DynamicObjectFilter::detectDynamicPoints(const CloudType::Ptr& cloud,
                                                             double timestamp) {
  std::vector<uint8_t> dynamic_mask(cloud->size(), 0);
  
  if (cloud_history_.size() < 2) {
    // 历史数据不足，保守策略：认为所有点都是静态的
    if (debug_mode_) {
      debugLog("[detect] history_frames<2, skip dynamic detection (all static)");
    }
    return dynamic_mask;
  }
  
  // 1. 计算时间一致性信息
  auto t0 = std::chrono::high_resolution_clock::now();
  std::vector<PointTemporalInfo> temporal_info = computeTemporalInfo(cloud, timestamp);
  auto t1 = std::chrono::high_resolution_clock::now();
  double t_temporal_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (debug_mode_) {
    std::ostringstream ss;
    ss << "[detect] temporal info built, points=" << cloud->size()
       << ", frames=" << cloud_history_.size()
       << ", time_ms=" << std::fixed << std::setprecision(2) << t_temporal_ms;
    debugLog(ss.str());
  }
    
    // 2. 基于时间一致性的初步筛选
    std::vector<uint8_t> temporal_candidates(cloud->size(), 0);
    
  for (size_t idx = 0; idx < temporal_info.size(); ++idx) {
    float stability = computeStabilityScore(temporal_info[idx]);
    if (stability < config_.stability_threshold) {
      temporal_candidates[idx] = 1;
    }
  }
  if (debug_mode_) {
    size_t cand = std::count(temporal_candidates.begin(), temporal_candidates.end(), 1);
    std::ostringstream ss;
    ss << "[detect] temporal candidates=" << cand;
    debugLog(ss.str());
  }
    
    // 3. 几何一致性验证
  t0 = std::chrono::high_resolution_clock::now();
  dynamic_mask = geometricConsistencyCheck(cloud, temporal_candidates);
  t1 = std::chrono::high_resolution_clock::now();
  double t_geom_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (debug_mode_) {
    size_t dyn = std::count(dynamic_mask.begin(), dynamic_mask.end(), 1);
    std::ostringstream ss;
    ss << "[detect] geometric checked, dynamic=" << dyn
       << ", time_ms=" << std::fixed << std::setprecision(2) << t_geom_ms;
    debugLog(ss.str());
  }
    
    return dynamic_mask;
}

// ================ 时间一致性分析算法 ================

std::vector<PointTemporalInfo> DynamicObjectFilter::computeTemporalInfo(
    const CloudType::Ptr& cloud, double timestamp) {
  
  std::vector<PointTemporalInfo> temporal_info(cloud->size());
  
  if (cloud_history_.empty()) {
    if (debug_mode_) {
      debugLog("[temporal] empty history");
    }
    return temporal_info;
  }
    
    // 建立当前点云的KD树
    kdtree_.setInputCloud(cloud);
    
    // 对于每个历史帧，寻找对应点并计算运动信息
  for (size_t hist_idx = 0; hist_idx < cloud_history_.size(); ++hist_idx) {
    const auto& hist_cloud = cloud_history_[hist_idx];
    double hist_timestamp = timestamp_history_[hist_idx];
    
    // 时间差检查
    if (std::abs(timestamp - hist_timestamp) > config_.max_time_diff) {
      if (debug_mode_) {
        debugLog("[temporal] skip history frame due to time_diff");
      }
      continue;
    }
        
        // 寻找对应点
        std::vector<int> correspondence = findCorrespondingPoints(cloud, hist_cloud);
        
        // 更新时间信息
        for (size_t i = 0; i < cloud->size() && i < correspondence.size(); ++i) {
            if (correspondence[i] >= 0 && correspondence[i] < static_cast<int>(hist_cloud->size())) {
                const auto& hist_point = hist_cloud->points[correspondence[i]];
                Eigen::Vector3f hist_pos(hist_point.x, hist_point.y, hist_point.z);
                
                temporal_info[i].addHistoryPosition(hist_pos, hist_timestamp);
            }
        }
    }
    
  return temporal_info;
}

float DynamicObjectFilter::computeStabilityScore(const PointTemporalInfo& info) const {
    if (info.history_positions.size() < 2) {
        return 1.0f; // 没有足够的历史信息，假设是静态的
    }
    
    // 计算位置变化的统计量
    std::vector<float> displacements;
    displacements.reserve(info.history_positions.size() - 1);
    
    for (size_t i = 1; i < info.history_positions.size(); ++i) {
        float displacement = (info.history_positions[i] - info.history_positions[i-1]).norm();
        displacements.push_back(displacement);
    }
    
    // 计算平均位移和标准差
    float mean_displacement = std::accumulate(displacements.begin(), displacements.end(), 0.0f) 
                             / displacements.size();
    
    float variance = 0.0f;
    for (float disp : displacements) {
        variance += (disp - mean_displacement) * (disp - mean_displacement);
    }
    variance /= displacements.size();
    float std_deviation = std::sqrt(variance);
    
    // 分析运动模式
    MotionFeatures motion = analyzeMotionPattern(info);
    
    // 综合评分计算
    float displacement_score = std::exp(-mean_displacement / config_.motion_threshold);
    float consistency_score = motion.direction_consistency;
    float smoothness_score = std::exp(-std_deviation / config_.motion_threshold);
    
    // 加权组合
    float stability_score = 0.4f * displacement_score + 
                           0.3f * consistency_score + 
                           0.3f * smoothness_score;
    
    return std::clamp(stability_score, 0.0f, 1.0f);
}

DynamicObjectFilter::MotionFeatures DynamicObjectFilter::analyzeMotionPattern(
    const PointTemporalInfo& info) const {
    
    MotionFeatures features{0.0f, 0.0f, 1.0f};
    
    if (info.history_positions.size() < 3 || info.timestamps.size() < 3) {
        return features;
    }
    
    std::vector<Eigen::Vector3f> velocities;
    std::vector<float> speeds;
    
    // 计算速度向量
    for (size_t i = 1; i < info.history_positions.size(); ++i) {
        double dt = info.timestamps[i] - info.timestamps[i-1];
        if (dt > 1e-6) { // 避免除零
            Eigen::Vector3f velocity = (info.history_positions[i] - info.history_positions[i-1]) / dt;
            velocities.push_back(velocity);
            speeds.push_back(velocity.norm());
        }
    }
    
    if (velocities.empty()) {
        return features;
    }
    
    // 平均速度
    features.velocity = std::accumulate(speeds.begin(), speeds.end(), 0.0f) / speeds.size();
    
    // 计算加速度
    std::vector<float> accelerations;
    for (size_t i = 1; i < velocities.size(); ++i) {
        double dt = info.timestamps[i+1] - info.timestamps[i];
        if (dt > 1e-6) {
            float acceleration = (velocities[i] - velocities[i-1]).norm() / dt;
            accelerations.push_back(acceleration);
        }
    }
    
    if (!accelerations.empty()) {
        features.acceleration = std::accumulate(accelerations.begin(), accelerations.end(), 0.0f) 
                               / accelerations.size();
    }
    
    // 方向一致性
    if (velocities.size() >= 2) {
        float total_consistency = 0.0f;
        int valid_pairs = 0;
        
        for (size_t i = 1; i < velocities.size(); ++i) {
            float v1_norm = velocities[i-1].norm();
            float v2_norm = velocities[i].norm();
            
            if (v1_norm > 1e-6 && v2_norm > 1e-6) {
                float dot_product = velocities[i-1].dot(velocities[i]);
                float cosine_similarity = dot_product / (v1_norm * v2_norm);
                total_consistency += std::clamp(cosine_similarity, -1.0f, 1.0f);
                valid_pairs++;
            }
        }
        
        if (valid_pairs > 0) {
            features.direction_consistency = (total_consistency / valid_pairs + 1.0f) / 2.0f;
        }
    }
    
    return features;
}

// ================ 几何特征验证算法 ================

std::vector<uint8_t> DynamicObjectFilter::geometricConsistencyCheck(
    const CloudType::Ptr& cloud, const std::vector<uint8_t>& dynamic_candidates) {

    std::vector<uint8_t> final_mask = dynamic_candidates;
    
    if (cloud->empty()) {
        return final_mask;
    }
    
    // 计算法向量
    auto normals = computeNormals(cloud);
    if (!normals || normals->size() != cloud->size()) {
        if (debug_mode_) {
            debugLog("法向量计算失败，跳过几何验证");
        }
        return final_mask;
    }
    
    // 使用局部KD树（避免在并行中访问共享KD树导致未定义行为）
    pcl::KdTreeFLANN<PointType> local_kdtree;
    local_kdtree.setInputCloud(cloud);
    
    // 顺序处理每个候选动态点，确保稳定性。必要时可改为分块并行并加互斥保护。
    for (size_t idx = 0; idx < final_mask.size(); ++idx) {
        if (!dynamic_candidates[idx]) {
            continue; // 不是候选点，跳过
        }

        // 邻域搜索
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;

        int found_neighbors = local_kdtree.radiusSearch(
            cloud->points[idx], config_.search_radius,
            neighbor_indices, neighbor_distances);

        if (found_neighbors < config_.min_neighbors) {
            final_mask[idx] = 0; // 邻居太少，可能是噪点，认为是静态
            continue;
        }

        // 分析密度特征
        DensityFeatures density_features = analyzeDensityFeatures(cloud, idx);

        // 法向量一致性检查
        const auto& current_normal = normals->points[idx];
        float normal_consistency_sum = 0.0f;
        int valid_normal_count = 0;

        for (int neighbor_idx : neighbor_indices) {
            if (neighbor_idx != static_cast<int>(idx) && 
                neighbor_idx < static_cast<int>(normals->size())) {

                const auto& neighbor_normal = normals->points[neighbor_idx];

                // 检查法向量有效性
                if (std::isfinite(current_normal.normal_x) && 
                    std::isfinite(current_normal.normal_y) &&
                    std::isfinite(current_normal.normal_z) &&
                    std::isfinite(neighbor_normal.normal_x) &&
                    std::isfinite(neighbor_normal.normal_y) &&
                    std::isfinite(neighbor_normal.normal_z)) {

                    Eigen::Vector3f n1(current_normal.normal_x, 
                                       current_normal.normal_y, 
                                       current_normal.normal_z);
                    Eigen::Vector3f n2(neighbor_normal.normal_x, 
                                       neighbor_normal.normal_y, 
                                       neighbor_normal.normal_z);

                    float dot_product = std::abs(n1.dot(n2));
                    normal_consistency_sum += dot_product;
                    valid_normal_count++;
                }
            }
        }

        // 几何一致性决策
        bool is_geometrically_consistent = true;

        if (valid_normal_count > 0) {
            float avg_normal_consistency = normal_consistency_sum / valid_normal_count;
            if (avg_normal_consistency < config_.normal_consistency_thresh) {
                is_geometrically_consistent = false;
            }
        }

        // 密度一致性检查
        if (density_features.density_variance > config_.density_ratio_thresh) {
            is_geometrically_consistent = false;
        }

        // 最终决策：只有时间和几何都认为是动态的点才标记为动态
        final_mask[idx] = (dynamic_candidates[idx] && !is_geometrically_consistent) ? 1 : 0;
    }

    return final_mask;
}

pcl::PointCloud<pcl::Normal>::Ptr DynamicObjectFilter::computeNormals(
    const CloudType::Ptr& cloud) {
    
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    
    if (cloud->empty()) {
        return normals;
    }
    
    try {
        normal_estimator_.setInputCloud(cloud);
        normal_estimator_.compute(*normals);
    } catch (const std::exception& e) {
        if (debug_mode_) {
            debugLog("法向量计算异常: " + std::string(e.what()));
        }
        return std::make_shared<pcl::PointCloud<pcl::Normal>>();
    }
    
    return normals;
}

DynamicObjectFilter::DensityFeatures DynamicObjectFilter::analyzeDensityFeatures(
    const CloudType::Ptr& cloud, size_t point_idx) const {
    
    DensityFeatures features{0.0f, 0.0f, 0.0f};
    
    if (point_idx >= cloud->size()) {
        return features;
    }
    
    // 邻域搜索
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_distances;
    
    int found_neighbors = kdtree_.radiusSearch(
        cloud->points[point_idx], config_.search_radius,
        neighbor_indices, neighbor_distances);
    
    if (found_neighbors < 2) {
        return features;
    }
    
    // 计算局部密度
    float sphere_volume = (4.0f / 3.0f) * M_PI * 
                         std::pow(config_.search_radius, 3);
    features.local_density = found_neighbors / sphere_volume;
    
    // 计算距离方差（密度均匀性的指标）
    float mean_distance = std::accumulate(neighbor_distances.begin(), 
                                        neighbor_distances.end(), 0.0f) 
                         / neighbor_distances.size();
    
    float distance_variance = 0.0f;
    for (float dist : neighbor_distances) {
        distance_variance += (dist - mean_distance) * (dist - mean_distance);
    }
    distance_variance /= neighbor_distances.size();
    features.density_variance = distance_variance;
    
    // 表面平滑度（基于邻居点的分布）
    if (found_neighbors >= 3) {
        Eigen::Vector3f center_point(cloud->points[point_idx].x,
                                    cloud->points[point_idx].y,
                                    cloud->points[point_idx].z);
        
        float total_deviation = 0.0f;
        for (int neighbor_idx : neighbor_indices) {
            if (neighbor_idx != static_cast<int>(point_idx)) {
                Eigen::Vector3f neighbor_point(cloud->points[neighbor_idx].x,
                                              cloud->points[neighbor_idx].y,
                                              cloud->points[neighbor_idx].z);
                
                float distance_to_center = (neighbor_point - center_point).norm();
                total_deviation += std::abs(distance_to_center - mean_distance);
            }
        }
        features.smoothness = 1.0f / (1.0f + total_deviation / found_neighbors);
    }
    
    return features;
}

// ================ 工具函数实现 ================

void DynamicObjectFilter::updateHistory(const CloudType::Ptr& cloud, double timestamp) {
    // 添加新的点云和时间戳
    cloud_history_.push_back(cloud);
    timestamp_history_.push_back(timestamp);
    
    // 保持历史大小限制
    while (cloud_history_.size() > static_cast<size_t>(config_.history_size)) {
        cloud_history_.pop_front();
        timestamp_history_.pop_front();
        if (!temporal_info_history_.empty()) {
            temporal_info_history_.pop_front();
        }
    }
    
    // 清理过期数据
    cleanOldHistory(timestamp);
}

CloudType::Ptr DynamicObjectFilter::downsampleCloud(const CloudType::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return std::make_shared<CloudType>();
    }
    
    // 如果点云太大，使用自适应降采样
    if (cloud->size() > static_cast<size_t>(config_.max_points_per_frame)) {
        return adaptiveDownsample(cloud, config_.max_points_per_frame);
    }
    
    // 使用体素滤波进行降采样
    CloudType::Ptr downsampled = std::make_shared<CloudType>();
    
    try {
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*downsampled);
    } catch (const std::exception& e) {
        if (debug_mode_) {
            debugLog("体素滤波失败: " + std::string(e.what()));
        }
        return cloud; // 返回原点云
    }
    
    return downsampled;
}

CloudType::Ptr DynamicObjectFilter::adaptiveDownsample(const CloudType::Ptr& cloud, 
                                                      size_t target_size) {
    if (!cloud || cloud->empty() || cloud->size() <= target_size) {
        return cloud;
    }
    
    // 计算降采样比例
    float downsample_ratio = static_cast<float>(target_size) / cloud->size();
    
    CloudType::Ptr downsampled = std::make_shared<CloudType>();
    downsampled->reserve(target_size);
    
    // 均匀采样
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (static_cast<float>(i) * downsample_ratio >= downsampled->size()) {
            downsampled->push_back(cloud->points[i]);
            if (downsampled->size() >= target_size) {
                break;
            }
        }
    }
    
    downsampled->width = downsampled->size();
    downsampled->height = 1;
    downsampled->is_dense = cloud->is_dense;
    
    return downsampled;
}

void DynamicObjectFilter::cleanOldHistory(double current_timestamp) {
    while (!timestamp_history_.empty() && 
           (current_timestamp - timestamp_history_.front()) > config_.max_time_diff * 2) {
        cloud_history_.pop_front();
        timestamp_history_.pop_front();
        if (!temporal_info_history_.empty()) {
            temporal_info_history_.pop_front();
        }
    }
}

std::vector<int> DynamicObjectFilter::findCorrespondingPoints(
    const CloudType::Ptr& current_cloud, const CloudType::Ptr& reference_cloud) const {
    
    std::vector<int> correspondence(current_cloud->size(), -1);
    
    if (!reference_cloud || reference_cloud->empty()) {
        return correspondence;
    }
    
    // 建立参考点云的KD树
    pcl::KdTreeFLANN<PointType> ref_kdtree;
    ref_kdtree.setInputCloud(reference_cloud);
    
    // 对每个当前点找最近邻
    for (size_t i = 0; i < current_cloud->size(); ++i) {
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);
        
        if (ref_kdtree.nearestKSearch(current_cloud->points[i], 1, 
                                     nearest_indices, nearest_distances) > 0) {
            // 距离阈值检查
            if (nearest_distances[0] < config_.search_radius * config_.search_radius) {
                correspondence[i] = nearest_indices[0];
            }
        }
    }
    
    return correspondence;
}

CloudType::Ptr DynamicObjectFilter::createFilteredCloud(
    const CloudType::Ptr& input_cloud, const std::vector<uint8_t>& dynamic_mask) const {
    
    CloudType::Ptr filtered_cloud = std::make_shared<CloudType>();
    
    if (!input_cloud || input_cloud->empty() || dynamic_mask.size() != input_cloud->size()) {
        return filtered_cloud;
    }
    
    // 预计算静态点数量
    size_t static_count = std::count(dynamic_mask.begin(), dynamic_mask.end(), 0);
    filtered_cloud->reserve(static_count);
    
    // 复制静态点
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        if (!dynamic_mask[i]) {
            filtered_cloud->push_back(input_cloud->points[i]);
        }
    }
    
    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = input_cloud->is_dense;
    
    return filtered_cloud;
}

// ================ 配置和状态管理 ================

bool DynamicObjectFilter::updateConfig(const DynamicFilterConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!config.isValid()) {
        if (debug_mode_) {
            debugLog("配置参数无效");
        }
        return false;
    }
    
    config_ = config;
    
    // 更新PCL组件配置
    normal_estimator_.setKSearch(config_.min_neighbors);
    voxel_filter_.setLeafSize(config_.voxel_size_base, 
                             config_.voxel_size_base, 
                             config_.voxel_size_base);
    
    // 清理历史数据以适应新配置
    while (cloud_history_.size() > static_cast<size_t>(config_.history_size)) {
        cloud_history_.pop_front();
        timestamp_history_.pop_front();
        if (!temporal_info_history_.empty()) {
            temporal_info_history_.pop_front();
        }
    }
    
    if (debug_mode_) {
        debugLog("配置更新成功");
    }
    
    return true;
}

DynamicFilterConfig DynamicObjectFilter::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

FilterStats DynamicObjectFilter::getLastFilterStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_stats_;
}

void DynamicObjectFilter::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    cloud_history_.clear();
    timestamp_history_.clear();
    temporal_info_history_.clear();
    last_stats_.reset();
    
    if (debug_mode_) {
        debugLog("过滤器重置完成");
    }
}

void DynamicObjectFilter::resetStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_stats_.reset();
}

bool DynamicObjectFilter::isInitialized() const {
    return initialized_;
}

double DynamicObjectFilter::getMemoryUsageMB() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calculateMemoryUsage() / (1024.0 * 1024.0);
}

CloudType::Ptr DynamicObjectFilter::getLastDynamicPoints() const {
    std::lock_guard<std::mutex> lock(mutex_);

    // 返回最近一次过滤的动态点云的深拷贝
    CloudType::Ptr dynamic_cloud = std::make_shared<CloudType>();
    if (last_dynamic_cloud_ && !last_dynamic_cloud_->empty()) {
        *dynamic_cloud = *last_dynamic_cloud_;
    }

    return dynamic_cloud;
}

void DynamicObjectFilter::setDebugMode(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    debug_mode_ = enable;
}

// ================ 私有辅助函数 ================

void DynamicObjectFilter::updateStatistics(size_t total_points, size_t dynamic_points, 
                                           double processing_time_ms) {
    // 注意：filterDynamicObjects 已持有 mutex_，此处不能再次加锁
    // 不可调用 getMemoryUsageMB()（内部会再次尝试加锁导致死锁）
    last_stats_.total_points = total_points;
    last_stats_.dynamic_points = dynamic_points;
    last_stats_.static_points = total_points - dynamic_points;
    last_stats_.processing_time_ms = processing_time_ms;
    last_stats_.history_frames = cloud_history_.size();
    last_stats_.memory_usage_mb = calculateMemoryUsage() / (1024.0 * 1024.0);
    last_stats_.updateFilterRatio();
}

bool DynamicObjectFilter::validateInput(const CloudType::Ptr& cloud, double timestamp) const {
    if (!cloud) {
        if (debug_mode_) {
            debugLog("输入点云为空指针");
        }
        return false;
    }
    
    if (cloud->empty()) {
        if (debug_mode_) {
            debugLog("输入点云为空");
        }
        return false;
    }
    
    if (timestamp < 0) {
        if (debug_mode_) {
            debugLog("时间戳无效: " + std::to_string(timestamp));
        }
        return false;
    }
    
    // 检查时间戳的合理性（不能太旧或太新）
    if (!timestamp_history_.empty()) {
        double last_timestamp = timestamp_history_.back();
        if (timestamp < last_timestamp - config_.max_time_diff) {
            if (debug_mode_) {
                debugLog("时间戳过旧: " + std::to_string(timestamp) + 
                        " vs " + std::to_string(last_timestamp));
            }
            return false;
        }
    }
    
    return true;
}

void DynamicObjectFilter::debugLog(const std::string& message) const {
    if (debug_mode_) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);
        
        std::cout << "[DynamicObjectFilter] " 
                  << std::put_time(&tm, "%H:%M:%S") << " - " 
                  << message << std::endl;
    }
}

size_t DynamicObjectFilter::calculateMemoryUsage() const {
    size_t total_memory = 0;
    
    // 计算点云历史的内存使用
    for (const auto& cloud : cloud_history_) {
        if (cloud) {
            total_memory += cloud->size() * sizeof(PointType);
        }
    }
    
    // 计算时间戳历史的内存使用
    total_memory += timestamp_history_.size() * sizeof(double);
    
    // 计算时间信息历史的内存使用
    for (const auto& info_vec : temporal_info_history_) {
        for (const auto& info : info_vec) {
            total_memory += info.history_positions.size() * sizeof(Eigen::Vector3f);
            total_memory += info.timestamps.size() * sizeof(double);
        }
    }
    
    // 添加其他组件的估计内存使用
    total_memory += sizeof(DynamicObjectFilter);
    
    return total_memory;
}

} // namespace localizers
