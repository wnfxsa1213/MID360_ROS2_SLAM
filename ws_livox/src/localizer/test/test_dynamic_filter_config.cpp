#include <gtest/gtest.h>
#include "localizers/dynamic_object_filter.h"

using localizers::DynamicFilterConfig;
using localizers::DynamicObjectFilter;

TEST(DynamicFilterConfigTest, SanitizeClampsInvalidValues)
{
    DynamicFilterConfig cfg;
    cfg.motion_threshold = -0.5f;
    cfg.history_size = 0;
    cfg.stability_threshold = 1.5f;
    cfg.normal_consistency_thresh = -1.0f;
    cfg.density_ratio_thresh = 5.0f;
    cfg.uncertain_smoothness_thresh = 0.0f;
    cfg.search_radius = -0.2f;
    cfg.min_neighbors = 0;
    cfg.downsample_ratio = 0;
    cfg.max_points_per_frame = -1;
    cfg.voxel_size_base = 0.0f;

    auto issues = DynamicObjectFilter::sanitizeConfig(cfg);
    EXPECT_FALSE(issues.empty());
    EXPECT_GT(cfg.motion_threshold, 0.0f);
    EXPECT_GT(cfg.history_size, 0);
    EXPECT_GE(cfg.stability_threshold, 0.0f);
    EXPECT_LE(cfg.stability_threshold, 1.0f);
    EXPECT_GE(cfg.normal_consistency_thresh, 0.0f);
    EXPECT_LE(cfg.normal_consistency_thresh, 1.0f);
    EXPECT_GE(cfg.density_ratio_thresh, 0.0f);
    EXPECT_LE(cfg.density_ratio_thresh, 1.0f);
    EXPECT_GT(cfg.search_radius, 0.0f);
    EXPECT_GT(cfg.min_neighbors, 0);
    EXPECT_GT(cfg.downsample_ratio, 0);
    EXPECT_GT(cfg.max_points_per_frame, 0);
    EXPECT_GT(cfg.voxel_size_base, 0.0f);
}

TEST(DynamicFilterConfigTest, SanitizeKeepsValidConfiguration)
{
    DynamicFilterConfig cfg;
    cfg.motion_threshold = 0.2f;
    cfg.history_size = 10;
    auto issues = DynamicObjectFilter::sanitizeConfig(cfg);
    EXPECT_TRUE(issues.empty());
    EXPECT_TRUE(cfg.isValid());
}

