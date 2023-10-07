/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/optional.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

/**
 * @brief 添加排队传感器信息的抽象类
 * 
 */
class CollatorInterface {
 public:
  // note: CollatorInterface::Callback 2个参数
  using Callback =
      std::function<void(const std::string&, std::unique_ptr<Data>)>;

  CollatorInterface() {}
  virtual ~CollatorInterface() {}
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  /// @brief 添加轨迹以为生成排序的传感器输出。为每个整理的传感器数据调用“回调”
  virtual void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) = 0;

  // Marks 'trajectory_id' as finished.
  /// @brief 标记该trajectory_id已经结束啦
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
  // in time order.
  /**
   * @brief 为该轨迹id添加校正数据
   *        为要整理的“trajectory_id”添加“data”数据”必须包含有效的传感器数据。
   *        必须按时间顺序添加具有匹配“data.Sensor_id_”的传感器数据包。
   * @param[in] trajectory_id 
   * @param[in] data 
   */
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  /// @brief 调度所有排队的传感器数据包；且只能调用一次，在Flush后则无法调用AddSensorData
  virtual void Flush() = 0;

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before CollatorInterface is
  // unblocked. Returns 'nullopt' for implementations that do not wait for a
  // particular trajectory.
  virtual absl::optional<int> GetBlockingTrajectoryId() const = 0;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
