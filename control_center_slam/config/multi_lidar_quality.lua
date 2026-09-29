-- Keep an upstream-compatible base configuration for emergency rollback.
include "multi_lidar_optimized.lua"

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

-- Protect existing occupied cells from distant missed returns. The private
-- inserter keeps normal clearing for unknown/already-free cells, so this
-- does not slow exploration. Obstacles still clear after repeated evidence.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.distance_weighted_free_space = {
  near_range = 1.5,
  far_range = 4.0,
  far_miss_probability = 0.49,
}

-- Raw vendor snapshots have no trustworthy per-beam acquisition times.
-- Tracking continues during turns; only irreversible grid insertion is gated.
-- At low motion, wait 0.2 s to flush the rolling scan captured during motion.
TRAJECTORY_BUILDER_2D.insertion_motion_guard = {
  enabled = true,
  max_linear_speed = 0.22,
  max_angular_speed = 0.10,
  settle_seconds = 0.20,
  max_odometry_age = 0.20,
}
-- Repeated corridors: trust the recent odometry/pose-graph prediction instead
-- of searching five metres away for a visually similar neighbouring corridor.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
POSE_GRAPH.constraint_builder.min_score = 0.68
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.78
POSE_GRAPH.optimization_problem.huber_scale = 1.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 5e3
-- Incremental sessions are explicitly anchored by the operator's validated
-- initial pose. Do not silently recover across the entire old submap.
-- This does not disable the navigation module's deliberate global relocation.
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.log_matches = false
-- Full optimized trajectories grow over time; 2 Hz is sufficient for the
-- history overlay. Robot TF remains at the independent 50 Hz pose rate.
options.trajectory_publish_period_sec = 0.5
return options
