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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

/**
 * @brief 构造函数,
 * 完成active_submaps、motion_filter、暴力匹配、CSM等对象的初始化
 *
 * @param[in] options
 * @param[in] expected_range_sensor_ids 所有range类型的话题
 */
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
    // note: active_submaps_在初始化时就对range_data_inserter_插入器完成初始化
      active_submaps_(options.submaps_options()),         
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/**
 * @brief 先进行点云的旋转与z方向的滤波, 然后再进行体素滤波减少数据量
 *
 * @param[in] transform_to_gravity_aligned_frame 将点云变换到原点处,
 * 且姿态为0的坐标变换
 * @param[in] range_data 传入的点云
 * @return sensor::RangeData 处理后的点云 拷贝
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // Step: 5 将原点位于机器人当前位姿处的点云 转成
  // 原点位于local坐标系原点处的点云, 再进行z轴上的过滤
  const sensor::RangeData cropped = sensor::CropRangeData(
      sensor::TransformRangeData(range_data,
                                 transform_to_gravity_aligned_frame),
      options_.min_z(), options_.max_z());  // param: min_z max_z
  // Step: 6 对点云进行体素滤波，这里的origin为（0，0）
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(
          cropped.returns,
          options_.voxel_filter_size()),  // param: voxel_filter_size
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

/**
 * @brief 进行扫描匹配
 *
 * @param[in] time 点云的时间
 * @param[in] pose_prediction 先验位姿
 * @param[in] filtered_gravity_aligned_point_cloud 匹配用的点云
 * @return std::unique_ptr<transform::Rigid2d> 匹配后的二维位姿
 */
// core: 前端扫描匹配部分
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  // note:
  // 当第一帧数据过来时不存在子图信息，故不参与扫描匹配，直接返回pose_extrapolater输出预测位姿
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 使用active_submaps_的第一个子图进行匹配
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // 根据参数决定是否 使用correlative_scan_matching对先验位姿进行校准
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;

  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  // 一些度量
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回ceres计算后的位姿
  return pose_observation;
}

/**
 * @brief 处理点云数据, 进行扫描匹配, 将点云写成地图
 *
 * @param[in] sensor_id 点云数据对应的话题名称
 * @param[in] unsynchronized_data 传入的点云数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
 * 匹配后的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // Step: 1 进行多个雷达点云数据的时间同步, 点云的坐标是相对于tracking_frame的
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 如果不用imu, 就在雷达这初始化位姿推测器
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  // 计算第一个点的时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 只有在extrapolator_初始化时, GetLastPoseTime()是common::Time::min()
  // QUES:
  // 看代码意思是说第一帧雷达数据应该是没法用，若数据正常time_first_point应该不会比LastPose大；
  // 所以是不是正常雷达都从第二帧开始实际应用

  // QUES:
  // 如果双雷达产生的点云已经滞后了，这边也会被返回，但是这样的返回码是不是有些错误
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f>
      range_data_poses;  // tracking frame 在 local slam 坐标系下的位姿
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;

  // 预测得到每一个时间点的位姿
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    // 如果该时间比上次预测位姿的时间还要早,说明这个点的时间戳往回走了, 就报错
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      // 一个循环只报一次错
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }

    // Step: 2 预测出 每个点的时间戳时刻, tracking frame 在 local slam
    // 坐标系下的位姿
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 对每个数据点进行处理
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取在tracking frame 下点的坐标
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    // 将点云的origins坐标转到 local slam 坐标系下 Tori2local = Ttrack2local *
    // Tori2track
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);

    // core: 运动畸变剔除，借助ExtrapolatePose函数计算预测位姿，完成当前时间点下激光点位置计算
    // Step: 3 运动畸变的去除, 将相对于tracking_frame的hit坐标 转成
    // local坐标系下的坐标
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);

    // 计算这个点的距离, 这里用的是去畸变之后的点的距离
    const Eigen::Vector3f delta =
        hit_in_local.position -
        origin_in_local;  // 这里的减号可以认为hit于origin的坐标
    const float range = delta.norm();  // 求2范数

    // param: min_range max_range
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        // 在这里可以看到, returns里保存的是local
        // slam下的去畸变之后的点的坐标（在最大最小范围内，则认为点云一定打在了障碍物上认为其为return，否则为miss）
        // 而实际应用的雷达还是hit_in_local的值来完成其他操作
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // Step: 4 超过max_range时的处理: 用一个距离进行替代, 并放入misses里
        hit_in_local.position =
            origin_in_local +
            // param: missing_data_ray_length, 是个比例, 不是距离
            // QUES:
            // 其实有点不明白为什么要做这个比例？！如果点云中存在inf，则这个点就是0；但是其他情况感觉有点混乱
            // 现阶段理解：可以认为既然已经在大距离照射距离内无障碍物，那么近距离内肯定也没有障碍物，同样的如果点云中存在inf,则认为这个值是
            // 不可靠的，所以距离为0也可以说通
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }  // end for

  // 有一帧有效的数据了
  ++num_accumulated_;

  // param: num_accumulated_range_data 几帧有效的点云数据进行一次扫描匹配
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 计算2次有效点云数据的的时间差
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // 重置变量
    num_accumulated_ = 0;

    // 获取机器人当前姿态
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));

    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    // 以最后一个点的时间戳估计出的坐标为这帧数据的原点
    // QUES: 为什么把range_data_poses.back()<tracking frame 在 local slam
    // 坐标系下的位姿>作为这一帧点云原点
    accumulated_range_data_.origin = range_data_poses.back().translation();

    return AddAccumulatedRangeData(
        time,
        // 将点云变换到local原点处, 且姿态为0
        TransformToGravityAlignedFrameAndFilter(
            // note: 这里的T*T-1并不是等于E，要细看一下inverse公式
            // QUES:
            // 坐标转换实际意义代表的是啥？到底是点云的重力校正，还是说转换到了原点？？？
            // gravity_alignment可以理解为Tgravity->local
            // range_data_poses可以理解为Thit->local
            // 这样下式计算就是激光点云的位姿仅仅加上了重力姿态的调整。
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }

  return nullptr;
}

/**
 * @brief 进行扫描匹配, 将点云写入地图
 *
 * @param[in] time 点云的时间戳
 * @param[in] gravity_aligned_range_data 原点位于local坐标系原点处的点云
 * @param[in] gravity_alignment 机器人当前姿态
 * @param[in] sensor_duration 2帧点云数据的时间差
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  // 如果处理完点云之后数据为空, 就报错. 使用单线雷达时不要设置min_z
  // note: 若报"Dropped empty horizontal range
  // data."错误是因为min_z参数设定，导致没有雷达数据
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 进行位姿的预测, 先验位姿
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // QUES: 没明白为什么要做这个旋转？？？
  // 将三维位姿先旋转到姿态为0, 再取xy坐标将三维位姿转成二维位姿
  // NOTE: 需注意坐标转换过程，这边把机器人预测姿态也转换至0
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // Step: 7 对 returns点云 进行自适应体素滤波，返回的点云的数据类型是PointCloud
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 扫描匹配, 进行点云与submap的匹配
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);

  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // 将二维坐标旋转回之前的姿态
  // QUES: 没明白为什么又做了次重力校正？？？
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 校准位姿估计器
  extrapolator_->AddPose(time, pose_estimate);

  // Step: 8 将 原点位于local坐标系原点处的点云 变换成
  // 原点位于匹配后的位姿处的点云
  // QUES:这里还是不确定地图中点是怎么更新的，他有一个更新标志，难到只更新一次吗？
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));

  // 将校正后的雷达数据写入submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  // 计算耗时
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  // 计算cpu耗时
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回结果
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

/**
 * @brief 将处理后的雷达数据写入submap
 *
 * @param[in] time 点云的时间
 * @param[in] range_data_in_local 校正后的点云
 * @param[in] filtered_gravity_aligned_point_cloud 自适应体素滤波后的点云
 * @param[in] pose_estimate 扫描匹配后的三维位姿
 * @param[in] gravity_alignment
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 如果移动距离过小, 或者时间过短, 不进行地图的更新
  // core: 运动滤波器的使用：建图时使用运动滤波器完成点云插入
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 1、将经坐标转换后的激光点云数据写入到submap中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  // 2、将原始滤波后点云用于生成InsertionResult格式的数据进行返回
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,  // 这里存的是体素滤波后的原点处点云,不是校准后的点云
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 将IMU数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// 将里程计数据加入到Extrapolator中
// core: 传感器数据流向step7->在local_trajectory_builder中加入至其所属的pose_extrapolator中
// NOTE: 可以看到如果订阅到IMU数据则姿势外插器直接被初始化；如果没有订阅到IMU数据的话，姿势外插器会在增加激光数据后被初始化
// 简单理解：cartographer传感器搭配 IMU+(odom-可有可没有)+lidar 或者Lidar
// 或者Lidar+odom
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 如果Extrapolator没有初始化就进行初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) {
    return;
  }

  // NOTE: use_imu_based为true就会报错,2d是不允许为true的，3d可以；
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(
          options_.pose_extrapolator_options()
              .constant_velocity()
              .pose_queue_duration()),  // 0.001s
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());  // 10
  // core: 添加初始位姿, PoseExtrapolator建图是完成初始化
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
