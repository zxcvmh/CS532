-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0
-- Configured for NVIDIA JetBot CS532 + LDROBOT D500 LiDAR (2D Distributed SLAM)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",          -- REP-105: Cartographer publishes odom -> base_link when provide_odom_frame = true
  odom_frame = "odom",
  provide_odom_frame = true,              -- Cartographer computes continuous laser odometry without wheel slip
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,                   -- Differential JetBot wheels slip on smooth surfaces; scan odometry is superior
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Local SLAM: Real-Time Correlative Scan Matcher (CSM) + Ceres Scan Matcher
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.5
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Online Correlative Scan Matcher Parameters (Olsen 2009 / Hess et al. 2016)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres Submap Fine-Tuning
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0 -- Heavy penalty against aggressive rotation slips

-- Submap Lifecycle (Optimized for 2m x 1m Arena & Indoor Rooms)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 -- 35 scans (~3.5 seconds at 10Hz) per submap lifecycle
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- Global SLAM: Branch-and-Bound Loop Closure & Pose Graph Optimization
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

return options
