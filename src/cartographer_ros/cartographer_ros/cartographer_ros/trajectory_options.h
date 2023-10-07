/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer_ros {

/**
 * @brief 一条轨迹的基础参数配置
 * @param trajectory_builder_options lua中轨迹参数
 * @param tracking_frame 将所有传感器数据转换到这个坐标系下
 * @param published_frame tf: map -> footprint，carto tf树中 指向的最后一个
 * @param odom_frame 里程计坐标系名字
 * @param provide_odom_frame 是否提供里程计tf，若为true,则tf树为map->odom->"published_frame"
 * @param use_odometry 是否应用里程计，若应用则需提供odom的tf(通过看代码似乎不大需要)
 * @param use_nav_sat 是否使用gps
 * @param use_landmarks 是否使用landmarks
 * @param publish_frame_projected_to_2d 是否将里程计坐标系投影到平面上
 * @param num_laser_scans 是否使用单线激光雷达，0为不使用，1为程序订阅scan, 2为程序订阅scan1 scan2
 * @param num_multi_echo_laser_scans 是否使用multi_echo_laser_scans数据，和上面num_laser_scans一样
 * @param num_subdivisions_per_laser_scan 1帧数据被分成几次处理,一般为1,carto数据包中的雷达频率都可以达到1500hz
 * @param num_point_clouds 是否使用点云数据，同时可理解为是否使用多线激光雷达 || 需要注意这里可以将num_point_clouds置为1，num_laser_scans为0，则在这里用多线雷达构建2D地图
 * @param rangefinder_sampling_ratio 传感器数据的采样频率，如果是1的话就是来一个数据就要一个，为0.5的话是每来两个数据采用
 * @param odometry_sampling_ratio 如上所述
 * @param fixed_frame_pose_sampling_ratio 如上所述
 * @param imu_sampling_ratio 如上所述
 * @param landmarks_sampling_ratio 如上所述
 */
struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  std::string tracking_frame;
  std::string published_frame;
  std::string odom_frame;
  bool provide_odom_frame;
  bool use_odometry;
  bool use_nav_sat;
  bool use_landmarks;
  bool publish_frame_projected_to_2d;
  int num_laser_scans;
  int num_multi_echo_laser_scans;
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;
  double landmarks_sampling_ratio;
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
