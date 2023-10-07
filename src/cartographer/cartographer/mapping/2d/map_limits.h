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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.

/**
 * note: 地图坐标系可视化展示
 * ros的地图坐标系    cartographer的地图坐标系     cartographer地图的像素坐标系 
 * 
 * ^ y                            ^ x              0------> x
 * |                              |                |
 * |                              |                |
 * 0 ------> x           y <------0                y       
 * 
 * ros的地图坐标系: 左下角为原点, 向右为x正方向, 向上为y正方向, 角度以x轴正向为0度, 逆时针为正
 * cartographer的地图坐标系: 坐标系右下角为原点, 向上为x正方向, 向左为y正方向
 *             角度正方向以x轴正向为0度, 逆时针为正
 * cartographer地图的像素坐标系: 左上角为原点, 向右为x正方向, 向下为y正方向
 */
class MapLimits {
 public:
  /**
   * @brief 构造函数
   * 
   * @param[in] resolution 地图分辨率
   * @param[in] max 左上角的坐标为地图坐标系的最大值
   * @param[in] cell_limits 地图x方向与y方向的格子数
   */
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  /// @brief 返回地图分辨率值
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  /// @brief 返回地图左上角坐标, 左上角坐标为整个cartographer地图坐标系坐标的最大值（均为正值)
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  /// @brief 返回地图中x、y方向上的最大栅格个数
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  /// @brief 计算物理坐标点的像素索引
  // NOTE: 个人认为下面地图坐标系转换像素坐标系、像素坐标系转换地图坐标系的方式中±0.5的含义
  /*
  *            像素坐标系
  *           0--------->x
  *           |                      0  1
  *           |                     _____
  *           |                    |__|__|       解释：通过左图可看，四舍五入的方式是将竖杠左右半边的
  *           |                ->  |  |  | 当做一个栅格，但是对于第一个栅格来讲就只有1/4；在减去0.5个栅格后，则表示竖杠右侧为一个栅格
  *           |                    
  *           y
  */

  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    // 这里要注意Array2i的[0]，即CellIndex.x是由来自点云坐标的y，相应地CellIndex.y来自点云坐标x；
    // 因为像素坐标系和地图坐标系的行列表示方式不同，像素坐标系的行表示为y，地图坐标系的行表示为x
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // Returns the center of the cell at 'cell_index'.
  /// @brief 根据像素索引算物理坐标，是上述转换的反向计算,同样要在地图坐标系中展示，所以又要反过来
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const {
    return {max_.x() - resolution() * (cell_index[1] + 0.5),
            max_.y() - resolution() * (cell_index[0] + 0.5)};
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  /// @brief 判断给定像素索引是否在栅格地图内部
  bool Contains(const Eigen::Array2i& cell_index) const {
    // 这里的all应该可以理解为这个array数组的所有成员均为true
    // QUES: 从GetCellIndex可得出，cell_index[0]是y方向，但此时比较的是num_x_cells，实际测试时，num_x_cells == num_y_cells，也就是正方形
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;
  Eigen::Vector2d max_;    // cartographer地图坐标系左上角（可以理解为像素坐标系原点坐标）为坐标系的坐标的最大值，QUES: 这里是子图坐标系还是全局坐标系，个人理解为全局坐标系
  CellLimits cell_limits_;
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
