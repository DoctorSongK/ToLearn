/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// core: 位姿外插器部分，可用作点云数据去畸变、点云配准初值提供、重力方向校准等基础功能
// 该部分代码共有两处实例化部分：
// 其一为cartographer_ros:node中，用于ros层端的递归工作，发布SLAM的Trajectory States，旨在人机交互发布current pose信息；
// 其二为cartographer:LocalTrajectoryBuilder2D中，主要负责前端点云数据去畸变和点云配准初值提供。
// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
// 保持poses一定持续时间, 以估计线速度和角速度
// 使用速度预测运动. 使用IMU和/或里程计数据（如果有）来改善预测
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // c++11: override 关键字告诉编译器, 该函数应覆盖基类中的函数.
  // 如果该函数实际上没有覆盖任何函数, 则会导致编译器错误
  // 如果没加这个关键字 也没什么严重的error 只是少了编译器检查的安全性

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  // 返回上次添加姿势的时间，如果尚未添加姿势，则返回time::min()
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  // 保存一定时间内的pose
  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;                                    // 经扫描匹配后携带时间的位姿队列 
  // 线速度有2种计算途径
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();      // 通过点云匹配的位姿获取的线速度
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();     // 通过点云匹配的位姿获取的角速度

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;                                      // imu数据缓存队列

  // c++11: std::unique_ptr 是独享被管理对象指针所有权的智能指针
  // 它无法复制到其他 unique_ptr, 也无法通过值传递到函数,也无法用于需要副本的任何标准模板库 (STL) 算法
  // 只能通过 std::move() 来移动unique_ptr
  // std::make_unique 是 C++14 才有的特性

  // imu_tracker_更新频率将与激光匹配频率一致（每当前端CSM计算姿态时，该点云匹配结果的姿态（timepose）会被用于统一更新/对齐以上三个ImuTracker）
  // odometry_imu_tracker_更新频率与ODOM频率一致（该部分除了CSM输入时更新，还在odom data被传入时更新）
  // extrapolation_imu_tracker_更新频率与前端调用PoseExtrapolator::ExtrapolatePose()频率一致（由上述函数调用更新）

  // 下述3个实例的目的是为了在不同关键数据传入时，根据不同的timestamp进行的imu tracker的更新不会相互耦合影响。
  std::unique_ptr<ImuTracker> imu_tracker_;                                   // 扫描匹配（add_pose）用：  1、保持最新的CSM匹配结果的姿态；2、跟踪维护CSM当前激光帧frame姿态的旋转信息
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;                          // odom用：odom线速度计算用  1、保持最新ODOM位姿的递归姿态；2、跟踪维护当前里程计frame下的姿态旋转量信息
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;                     // imu用: IMU计算角速度用    1、保持最新推算位姿的递归姿态；2、跟踪维护当前推算位姿frame下的姿态旋转量信息
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;                            // 里程计数据缓存队列
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();   // 通过轮式里程计获取的线速度
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();  // 通过轮式里程计获取的角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
