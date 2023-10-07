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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/grid_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

enum class GridType { PROBABILITY_GRID, TSDF };

// 注意Grid2D里面存储的全是空闲概率
class Grid2D : public GridInterface {
 public:
 // param: correspondence_cost 栅格为free的概率
  Grid2D(const MapLimits& limits, float min_correspondence_cost,
         float max_correspondence_cost,
         ValueConversionTables* conversion_tables);
  explicit Grid2D(const proto::Grid2D& proto,
                  ValueConversionTables* conversion_tables);

  // Returns the limits of this Grid2D.
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate();

  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
    if (!limits().Contains(cell_index)) return max_correspondence_cost_;
    return (*value_to_correspondence_cost_table_)
        [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
  }

  virtual GridType GetGridType() const = 0;

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  // 指定的栅格是否被更新过
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
               kUnknownCorrespondenceValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f& point);

  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

  virtual proto::Grid2D ToProto() const;

  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 // 保护成员仅可被类内及其派生类内部访问，不可外部访问
 protected:
  void GrowLimits(const Eigen::Vector2f& point,
                  const std::vector<std::vector<uint16>*>& grids,
                  const std::vector<uint16>& grids_unknown_cell_values);

  // 返回不可以修改的栅格地图数组的引用
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  const std::vector<int>& update_indices() const { return update_indices_; }
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  // 返回可以修改的栅格地图数组的指针
  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }

  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  // 二维像素坐标转为一维索引坐标
  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

 private:
  MapLimits limits_;  // 地图大小边界, 包括x和y最大值, 分辨率, x和y方向栅格数

  /// @brief 地图栅格存储值, 存储的是free的概率转成uint16后的[0, 32767]范围内的值, 0代表未知
  std::vector<uint16> correspondence_cost_cells_; 
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  std::vector<int> update_indices_;               // 记录已经更新过的索引 QUES:暂不清楚应用作用

  // Bounding box of known cells to efficiently compute cropping limits.

  /**
   * Eigen::AlignedBox<scalar,scalar>
   * #include <Eigen/Geometry>
   * 这个类代表由边界构成的区域，第一个scalar代表边界的数据类型（例如int,float,double等），第二个scalar代表边界的维度（通常为1，2，3）
   * 1）min() 返回边界中最小的角点
   * 2）max() 返回边界中最大的角点
   * 3）diagnol() 返回区域对角线的长度，即边界中最小和最大的角点距离
   * 4）extend(const MatrixBase<T> &point): 扩展区域的边界，使得该区域包含输入的点
   * 5）translate() 以a_t为参数进行平移。参数a_t类型必须要Eigen::Vector2f
   * 6) size() 返回区域宽度.x()、高度.y()。返回值类型Eigen::Vector2f,因为min\max可能含小数
   */
  Eigen::AlignedBox2i known_cells_box_;           // 栅格的bounding box, 存的是像素坐标，猜测这里是将整个地图尺寸由该容器表示
  // 将[0, 1~32767] 映射到 [0.9, 0.1~0.9] 的转换表
  const std::vector<float>* value_to_correspondence_cost_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
