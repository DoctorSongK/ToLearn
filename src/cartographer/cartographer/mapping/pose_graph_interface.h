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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <chrono>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  
  // 包含了子图的id, 节点的id, 节点j相对于子图i的坐标变换, 以及节点是在子图内还是子图外的标志
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;        // 平移权重 
      double rotation_weight;           // 旋转权重
    };

    SubmapId submap_id;  /// @brief  子图索引'i' in the paper.
    NodeId node_id;      /// @brief  节点索引'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    /// @brief 节点索引“j”之于submap索引“i”的相对位姿
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    /// @brief intra 在子图内部; inter 在子图外部 
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  // 路标节点
  struct LandmarkNode {
    // landmark数据是相对于tracking_frame的相对坐标变换
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;
      double translation_weight;
      double rotation_weight;
    };
    // 同一时刻可能会观测到多个landmark数据
    std::vector<LandmarkObservation> landmark_observations;
    // 这帧数据对应的tracking_frame在global坐标系下的位姿
    absl::optional<transform::Rigid3d> global_landmark_pose;
    bool frozen = false;
  };

  struct SubmapPose {
    int version;
    transform::Rigid3d pose;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  // tag: TrajectoryData
  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    absl::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };

  // 轨迹状态：active、finished、frozen、deleted
  enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  /**
   * @brief 执行图优化计算
   * 
   */
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  // 返回所有的submap数据
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'. Returns 'nullptr' for the 'submap' member if the submap does
  // not exist (anymore).
  // 返回指定id的submap数据
  virtual SubmapData GetSubmapData(const SubmapId& submap_id) const = 0;

  // Returns the global poses for all submaps.
  // 返回所有submaps的global pose
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  // 得到正在优化的轨迹（包含整条轨迹上所有节点的前端匹配信息、global_pose）
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // Returns the current optimized trajectory poses.
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  // Returns the states of trajectories.
  // 返回当前轨迹状态（开始 结束 冻结 删除）
  virtual std::map<int, TrajectoryState> GetTrajectoryStates() const = 0;

  // Returns the current optimized landmark poses.
  // 返回当前被优化的landmark位姿
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
      const = 0;

  // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
  /**
   * @brief Set the Landmark Pose object
   * 
   * @param[in] landmark_id 给定landmark位姿
   * @param[in] global_pose landmark在全局坐标下的位姿
   * @param[in] frozen 
   */
  virtual void SetLandmarkPose(const std::string& landmark_id,
                               const transform::Rigid3d& global_pose,
                               const bool frozen = false) = 0;

  // Deletes a trajectory asynchronously.
  // 删除轨迹
  virtual void DeleteTrajectory(int trajectory_id) = 0;

  // Checks if the given trajectory is finished.
  /**
   * @brief 检查给定id的轨迹状态是否为FINISHED
   * 
   * @param[in] trajectory_id 
   * @return true 
   * @return false 
   */
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  /**
   * @brief 检查给定id的轨迹状态是否为FROZEN
   * 
   * @param[in] trajectory_id 
   * @return true 
   * @return false 
   */
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  // Returns the trajectory data.
  virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

  // Returns the collection of constraints.
  // 返回所有的连接约束
  virtual std::vector<Constraint> constraints() const = 0;

  // Serializes the constraints and trajectories. If
  // 'include_unfinished_submaps' is set to 'true', unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included, otherwise not.
  virtual proto::PoseGraph ToProto(bool include_unfinished_submaps) const = 0;

  // Sets the callback function that is invoked whenever the global optimization
  // problem is solved.
  // 设置每当全局优化时调用的回调函数
  virtual void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
