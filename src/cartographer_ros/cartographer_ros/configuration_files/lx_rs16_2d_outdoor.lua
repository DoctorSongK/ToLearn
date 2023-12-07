-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "imu_link",              -- 将所有传感器数据转换到这个坐标系下，选这个imu_link的原因，是因为imu的频率特别高，就可以将所有的数据转换到其下
                                            -- 如果应用频率较低的雷达或者其他传感器，那么就是将所有传感器坐标转换至这个低频传感器上，高频至低频的计算量较大，算法会检测并报出warn,一般情况下imu频率最高
  published_frame = "footprint",            -- tf: map -> footprint，carto tf树中 指向的最后一个
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
  --use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿

  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 1,                      -- 是否使用单线激光数据 0的话就是不使用单线雷达 1的话就订阅scan,2的话订阅scan1、scan2 
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据，和上面num_laser_scans一样
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1,carto数据包中的雷达频率都可以达到1500hz
  num_point_clouds = 0,                     -- 是否使用点云数据，同时可理解为是否使用多线激光雷达 || 需要注意这里可以将num_point_clouds置为1，num_laser_scans为0，则在这里用多线雷达构建2D地图
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  --cartographer采样控制器
  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率，如果是1的话就是来一个数据就要一个，为0.5的话是每来两个数据采用 
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 100.  --100
TRAJECTORY_BUILDER_2D.min_z = 0.2 --暴力滤除地面点   看到这里拉
--TRAJECTORY_BUILDER_2D.max_z = 1.4
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 160.  
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
