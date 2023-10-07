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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

// core: 多激光雷达数据时间同步程序,有个不明白的地方就是两个雷达时，多余的那一段雷达数据是怎么处理的？？？
/**
 * @brief 多个雷达数据的时间同步
 * 
 * @param[in] sensor_id 雷达数据的话题
 * @param[in] timed_point_cloud_data 雷达数据
 * @return sensor::TimedPointCloudOriginData 根据时间处理之后的数据
 */

/**
 * 如果采用一个激光雷达，此程序不做过多处理，若频率过高，可能会报drop earlier points；
 * 下面仅分析两个雷达及以上的时间同步情况。
 * 实际流程分析：
 * step1: 当第一帧激光雷达数据传入时，不做任何处理，传入id_to_pending_data_容器中；直至再次加入数据时id_to_pending_data_容器
 * 中达到指定类型、数量传感器数据要求则进行CropAndMerge处理；或者id_to_pending_data_已包含当前lidar topic数据则进行CropAndMerge处理，后将新数据添加至容器中
 * step2:
 *  情况一，当第二帧数据较新时，则将与第二帧数据重合的地方合并至第一帧数据中，并将每个激光点的时间戳进行更新；而第二帧较新的数据作为新数据保存在容器中，并于下一帧数据到来时才进行融合输出（此时情况与给定第三种情况相似）
 *        |····················|                          |###|
 *            |····················|           -->                |····················|
 *        |++++++++++++++++++++|###|                      |###|
 *  情况二，当第二帧数据较老时，则将与第二帧数据重合的地方合并至第一帧数据中，并将每个激光点的时间戳进行更新；而第二帧较老的数据直接抛出，此时id_to_pending_data_为空
 *                    |····················|              |····················|  
 *             |····················|          -->        
 *             |------|++++++++++++++++++++|              不满足融合条件
 *  情况三，当两帧数据没有交集时，则是逐个处理
 *        |····················|                                  |####################|
 *                                |····················|   -->                           |····················|
 *        |++++++++++++++++++++|  |####################|          |++++++++++++++++++++| |####################|
 * QUES: 问题分析：
 * 1、个人感觉，当发生情况1的时候，将剩余数据作为新数据放到里面去匹配，本身就存在滞后性，对建图定位的利好性较差；
 * 2、当不同频率的雷达进行融合时，感觉情况1会更厉害（待测试）
*/
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) { // 第一次拷贝
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

  // NOTE: 从sensor_bridge传过来的数据的intensities为空
  timed_point_cloud_data.intensities.resize(
      timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);

  // TODO(gaschler): These two cases can probably be one.
  // 如果同话题的点云, 还有没处理的, 就先弄同步没处理的点云, 将当前点云保存
  if (id_to_pending_data_.count(sensor_id) != 0) {
    // current_end_为上一次时间同步的结束时间
    // current_start_为本次时间同步的开始时间
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    // 本次时间同步的结束时间为这帧点云数据的结束时间
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    // 保存当前点云
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    return result;
  }

  // 先将当前点云添加到 等待时间同步的map中
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));

  // 等到range数据的话题都到来之后再进行处理
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }

  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  // 找到所有传感器数据中最早的时间戳(点云最后一个点的时间)
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  // current_end_是本次时间同步的结束时间
  // 是待时间同步map中的 所有点云中最早的时间戳
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

// 对时间段内的数据进行截取与合并, 返回时间同步后的点云
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  // 遍历所有的传感器话题
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    // 获取数据的引用
    sensor::TimedPointCloudData& data = it->second;
    const sensor::TimedPointCloud& ranges = it->second.ranges;
    const std::vector<float>& intensities = it->second.intensities;

    // 找到点云中 最后一个时间戳小于current_start_的点的索引
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }

    // 找到点云中 最后一个时间戳小于等于current_end_的点的索引
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }

    // 丢弃点云中时间比起始时间早的点, 每执行一下CropAndMerge()打印一次log
    // QUES: 在日常程序应用中，总会碰到这句话，也就说明雷达的time_increment和输出时间戳似乎不是很准确，导致前后两帧雷达数据总会有重叠
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      // 获取下个点云的index, 即当前vector的个数
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);  // 插入原点坐标

      // 获取此传感器时间与集合时间戳的误差, 
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));

      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());
      // reserve() 在预留空间改变时, 会将之前的数据拷贝到新的内存中
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      
      // 填充数据,即将多帧重合数据融合至一帧（指定时间戳序列）中
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        // 针对每个点时间戳进行修正, 让最后一个点的时间为0
        point.point_time.time += time_correction; 
        result.ranges.push_back(point);
      } // end for
    } // end if

    // Drop buffered points until overlap_end.
    // 如果点云每个点都用亦或者前半部分点云超时仅用后半部分, 则可将这个数据进行删除
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } 
    // 如果一个点都没用, 就先放这, 看下一个数据
    else if (overlap_end == ranges.begin()) {
      ++it;
    } 
    // 如果整帧点云仅用了前半部分，则将该位置处整帧点云替换为剩余后半部分，以便在下一次AddRangeData的时候使用
    else {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      // 将用了的点删除, 这里的赋值是拷贝
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  } // end for

  // 对各传感器的点云 按照每个点的时间从小到大进行排序
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
