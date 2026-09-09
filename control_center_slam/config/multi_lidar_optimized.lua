include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- The vendor already owns odom -> base_link. Publishing a second copy
  -- makes TF alternate between wheel odometry and scan-matched odometry.
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- Keep both native angular grids. The mapping-only scan adapter removes
  -- untrustworthy per-ray times from the vendor's resampled snapshots.
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 2e-2,
  trajectory_publish_period_sec = 5e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8

-- The 0.5 degree lidars resolve roughly 2.6 cm at 3 m. A 3 cm probability
-- grid preserves useful wall detail without the memory/CPU cost of 2 cm.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47

-- RangeDataCollator emits the two LaserScan streams separately. Accumulating
-- two range-data batches combines one front/rear observation before matching;
-- using 1 caused alternating single-lidar insertion and excessive node growth.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- Indoor measurements show the useful p99 range below 12 m. Rejecting longer
-- sparse returns reduces glass/multipath streaks that can bias scan matching.
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.06
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 12.0

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.10
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 300
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 12.0

-- Wheel odometry is still the initial guess, but the previous +/-20 degree
-- window and weak delta penalty allowed scan matching to walk in corridors.
-- This window still covers several frames of realistic omni-wheel slip.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 35.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 35
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 2

-- Preserve fine motion while avoiding a new noisy graph node every two seconds
-- when the robot is stationary for a long task.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.4)

-- Optimize frequently and accept plausible loop closures that the old 0.65
-- threshold discarded. Scores remain conservative enough for repeated rooms.
-- Keep the active trajectory well optimized so non-blocking map snapshots
-- are useful without calling FinishTrajectory.
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.7
POSE_GRAPH.constraint_builder.max_constraint_distance = 8.0
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
-- Keep normal operation quiet. Enable temporarily only when collecting
-- loop-closure diagnostics during a controlled mapping run.
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

POSE_GRAPH.optimization_problem.huber_scale = 1e1
-- The vendor odometry message publishes covariance=-1 (unknown). Keep it as a
-- smooth local prior but let high-score lidar loop closures remove wheel-slip
-- accumulation from a long map instead of forcing the map to follow the wheels.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 2e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 5e3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 80
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 8
POSE_GRAPH.global_constraint_search_after_n_seconds = 10
POSE_GRAPH.max_num_final_iterations = 300
POSE_GRAPH.log_residual_histograms = false

return options
