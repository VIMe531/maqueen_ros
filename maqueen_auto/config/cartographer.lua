include "map_builder.lua"
include "trajectory_builder.lua"

MAP_BUILDER.use_trajectory_builder_2d = true

-- trajectory_builder_2d の設定は trajectory_builder.lua 内の TRAJECTORY_BUILDER_2D にまとめて記述
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  num_subdivisions_per_laser_scan = 1,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_tracked_pose = true,

  lookup_transform_timeout_sec = 0.2,
  pose_publish_period_sec = 5e-3,
  submap_publish_period_sec = 0.3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

return options

