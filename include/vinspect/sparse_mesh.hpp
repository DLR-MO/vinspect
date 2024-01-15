// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef VINSPECT__SPARSE_MESH_H_
#define VINSPECT__SPARSE_MESH_H_
#include <omp.h>

#include <map>
#include <set>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/TriangleMeshIO.h"
#include "vinspect/inspection.hpp"
#include "vinspect/utils.hpp"

namespace vinspect
{

// todo the name "sparsemesh" could be replaced by something else
class SparseMesh
{
public:
  explicit SparseMesh(Inspection & inspection);
  const open3d::geometry::TriangleMesh & createMesh(
    const float point_size, const bool use_own_color, const int mean_min_max,
    const int value_type_index = 0);
  void resetMesh();

private:
  void setMergeColors(const int mean_min_max);
  void normalizeValues(
    const std::vector<std::vector<double>> * value_array, const uint64_t value_array_length);

  Inspection & inspection_;
  open3d::geometry::TriangleMesh mesh_;
  std::function<double(double, double, int)> merge_values_;
  int meshed_mean_min_max_;
  float meshed_point_radius_;
  bool meshed_use_own_color_;
  int meshed_value_type_index_;
  uint64_t meshed_sparse_data_count_;
  std::vector<double> normalized_value_array_;
  double meshed_min_value_;
  double meshed_max_value_;
  std::vector<uint64_t> appended_ids_;
  std::vector<uint64_t> point_values_;
  // Key: The placement order of the appended points: First gets 0, second gets 1 etc.
  // Value: ID of the point
  std::map<uint64_t, uint64_t> id_to_appended_index_;
  // Key: ID of a Point
  // Value: How many other points where merged into this point
  std::map<uint64_t, uint64_t> merged_points_at_id_;
};
}  // namespace vinspect

#endif  // VINSPECT__SPARSE_MESH_H_
