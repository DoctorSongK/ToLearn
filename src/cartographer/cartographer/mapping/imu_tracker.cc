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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {


// core: imu_tracker（主要计算机器人姿态，不参与位置计算）利用重力朝向始终一致特性，通过advance计算预测姿态，通过AddImuLinearAccelerationObservation获取当前重力朝向计算与转换重力朝向夹角完成姿态校准
// 如果看不大明白了可以参考一下https://www.cnblogs.com/gary-guo/p/16608604.html
/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object
 * 
 * @param[in] imu_gravity_time_constant 这个值在2d与3d情况下都为10
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()), // 初始方向角
      gravity_vector_(Eigen::Vector3d::UnitZ()),    // 重力方向初始化为[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/**
 * @brief 使用上一时刻角速度预测出time时刻的姿态与重力方向
 * 
 * @param[in] time 要预测的时刻
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  // 上一时刻的角速度乘以时间,得到当前时刻相对于上一时刻的预测的姿态变化量,再转换成四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // 使用上一时刻的姿态 orientation_ 乘以姿态变化量, 得到当前时刻的预测出的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // 根据预测出的姿态变化量,预测旋转后的线性加速度的值
  // 下列步骤的作用：
  // 四元数和向量相乘表示这个向量按照四元数进行旋转之后得到新向量；向量v绕四元数q旋转：q * v * conjugate(q)；
  // QUES: 当四元数与向量相乘时，向量是位于四元数所属坐标系，要不感觉解释不通？？？？？
  // NOTE: conjugate为四元数共轭操作，实际意义为表示旋转轴不变，旋转角相反
  // 四元数在多此运算之后，会存在累积误差，需定期归一化处理
  

  // NOTE: 为帮助理解，现在这边计算的gravity_vector重力向量为通过角速度积分计算得到的姿态值，那么这样的值也就存在误差
  // NOTE: 此处可看做是imu_tracker计算姿态的测量部分，主要是重力向量的测量
  // 重力向量是朝上的，机器人姿态当发生顺时针变换时，为保证逆时针
  gravity_vector_ = rotation.conjugate() * gravity_vector_;  // 注意：实际IMU输出的z轴线加速度是朝上的，重力向下压，可以看做弹簧给了一个向上的力
  // 更新时间
  time_ = time;
}

/**
 * @brief 更新线性加速度的值,并根据重力的方向对上一时刻的姿态进行校准
 * 
 * @param[in] imu_linear_acceleration imu的线加速度的大小
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 指数滑动平均法 exponential moving average
 
  // Step: 1 求delta_t, delta_t初始时刻为infinity, 之后为time_-last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;

  // Step: 2 求alpha, alpha=1-e^(-delta_t/10)
  // delta_t越大, alpha越大
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // Step: 3 将之前的线加速度与当前传入的线加速度进行融合, 这里采用指数滑动平均法

  // 指数来确定权重, 因为有噪声的存在, 时间差越大, 当前的线性加速度的权重越大
  // 这里的gravity_vector_改成线性加速度更清晰一些
  
  // 一阶低通融合方式：
  // NOTE: 为帮助理解，这里的gravity_vector_是经过线加速度（重力的另一表现，加速度也是因为有力才存在的）校准的
  // 当pose_extrapolator采用add_pose加入CSM结果时，此时alpha会较大。
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
      
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // Step: 4 求得 线性加速度的值 与 由上一时刻姿态求出的线性加速度 间的旋转量
  // 计算出经校正后重力向量与机器人z轴的夹角，并通过下方完成补偿
  // NOTE: 此处的orientation_是当前tracking_frame在local frame下的姿态，在这里我们只关注重力方向的值，即z轴上的值
  // 由于tracking_frame的姿态在重力方向上的变化量与重力向量的方向是相反的（比如小车向上仰a角度，那么重力向量是向反方向旋转a度，故取逆）
  // 为帮助理解这里的Eigen::Vector3d::UnitZ()也是tracking_frame坐标系下的向量，此时通过变换将该向量转换至与重力垂直方向；
  // 通过计算所述转换重力与实际重力的差值，完成姿态补偿
  // NOTE: 可以将该部分当做imu_tracker用重力校准姿态的测量部分
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  // Step: 5 使用这个旋转量来校准当前的姿态（补偿）
  orientation_ = (orientation_ * rotation).normalized();

  // NOTE: glog CHECK_GT: 第一个参数要大于第二个参数
  // 如果线性加速度与姿态均计算完全正确,那这二者的乘积应该是 0 0 1
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
