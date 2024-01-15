// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/sparse_mesh.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace vinspect
{
#define NUM_THREADS 8

/**
 * Is called by add_vertices to set the RGB colors values that correspond to the measurement_values.
 *
 * @param measurement_value the measured value from the sensor as a single float32 value
 * @returns numpy array containing three float values between 0 and 1
 */
Eigen::Vector3d getColorValue(double measurement_value)
{
  double hue_low_score = 360.0;
  double hue_high_score = 240.0;
  double hue = hue_low_score + (hue_high_score - hue_low_score) * measurement_value;
  Eigen::Vector3d rgb_color_value = hsv2rgb(hue, 1.0, 1.0);

  return rgb_color_value;
}

/**
 * Is called by add_vertices to create a list of coordinates that represent the given point + 8 new points surrounding
 * it. The resulting shape is also rotated with the rotation of the original point.
 *
 * @param point original point from dataset first three entry are the coordinates, last four entries is the quaternion
 * @returns list of corners, each represented by [x,y,z]
 */
std::array<Eigen::Vector3d, 9> createOctagon(
  const std::array<double, 3> & point, const std::array<double, 4> & ori, double point_radius)
{
  // todo this is a bit hacky. without this points are overlapping.
  // with this, the size is not perfectly correct.
  point_radius = point_radius / 2;
  double x = point[0];
  double y = point[1];
  double z = point[2];
  Eigen::Vector3d mid_point = Eigen::Vector3d(x, y, z);
  Eigen::Quaterniond quat(ori[0], ori[1], ori[2], ori[3]);

  // Calculating homogenous affine matrix
  Eigen::Isometry3d T;
  T.setIdentity();
  T.rotate(quat.matrix());
  T.translation() = mid_point;

  std::array<Eigen::Vector3d, 8> points = {
    Eigen::Vector3d(x - point_radius * 0.5, y + point_radius, z),
    Eigen::Vector3d(x + point_radius * 0.5, y + point_radius, z),
    Eigen::Vector3d(x + point_radius, y + point_radius * 0.5, z),
    Eigen::Vector3d(x + point_radius, y - point_radius * 0.5, z),
    Eigen::Vector3d(x + point_radius * 0.5, y - point_radius, z),
    Eigen::Vector3d(x - point_radius * 0.5, y - point_radius, z),
    Eigen::Vector3d(x - point_radius, y - point_radius * 0.5, z),
    Eigen::Vector3d(x - point_radius, y + point_radius * 0.5, z)};

  std::array<Eigen::Vector3d, 9> corners;
  corners[0] = (mid_point);

  for (int i = 0; i < 8; i++) {
    Eigen::Vector3d point = points[i];
    Eigen::Vector3d point_vector = point - mid_point;
    Eigen::Isometry3d p = Eigen::Isometry3d();
    p.setIdentity();
    p.translation() = point_vector;
    Eigen::Vector3d transformed_point = (T * p).translation();
    corners[i + 1] = transformed_point;
  }

  return corners;
}

/**
 * Is called by add_vertices to normalize the data array to a scale of 0 to 1.
 * @param value_array numpy array filled with single values. These are measured by the sensor.
 * @returns returns a numpy array of the normalized values.
 */
void SparseMesh::normalizeValues(
  const std::vector<std::vector<double>> * value_array, const uint64_t value_array_length)
{
  // extend vector so that we can directly access the entries in parallel
  normalized_value_array_.resize(value_array_length);
  // only parallelize if we need to process a large number of entries to avoid unnecessary
  // threading overhead
#pragma omp parallel for num_threads( \
  NUM_THREADS) if (value_array_length - meshed_sparse_data_count_ < 1000)
  for (uint64_t i = meshed_sparse_data_count_; i < value_array_length; ++i) {
    // normalize to a number between 0 and 1.
    // The min and max values can be set to allow cutoffs to both sides.
    normalized_value_array_[i] = std::max(
      0.0, std::min(
        1.0, (value_array->at(i)[meshed_value_type_index_] - meshed_min_value_) /
        (meshed_max_value_ - meshed_min_value_)));
  }
}

std::vector<uint64_t> commonMembers(
  std::vector<uint64_t> v1, std::vector<uint64_t> v2)
{
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());

  std::vector<uint64_t> v_intersection;
  std::set_intersection(
    v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v_intersection));
  return v_intersection;
}

SparseMesh::SparseMesh(Inspection & inspection)
: inspection_(inspection)
{
  meshed_mean_min_max_ = -1;
  meshed_point_radius_ = -1.0;
  meshed_use_own_color_ = false;
  meshed_value_type_index_ = -1;
  resetMesh();
}

void SparseMesh::resetMesh()
{
  mesh_.Clear();
  meshed_sparse_data_count_ = 0;
  normalized_value_array_.clear();
  meshed_min_value_ = std::numeric_limits<double>::max();
  meshed_max_value_ = std::numeric_limits<double>::min();
  appended_ids_.clear();
  point_values_.clear();
  mesh_.vertex_colors_.clear();
  id_to_appended_index_.clear();
  merged_points_at_id_.clear();
}

void SparseMesh::setMergeColors(const int mean_min_max)
{
  if (mean_min_max == 0) {
    // mean
    merge_values_ = [](double current_value, double new_value, int number_of_merged_points) {
        return (new_value + current_value * number_of_merged_points) /
               (number_of_merged_points + 1);
      };
  } else if (mean_min_max == 1) {
    // min
    merge_values_ = [](double current_value, double new_value, int number_of_merged_points) {
        return std::min(new_value, current_value);
      };
  } else if (mean_min_max == 2) {
    // max
    merge_values_ = [](double current_value, double new_value, int number_of_merged_points) {
        return std::max(new_value, current_value);
      };
  } else {
    throw std::runtime_error("mean_min_max must be 0, 1 or 2");
  }
}

/**
 *  Is called by sparse_mesh to create new vertices on and around all the points of the dataset and color them
 *  corresponding to their value.
 *
 * @param mesh open3D mesh object that corresponds to the real object the data was gathered from.
 * @param inspection vinspect.inspection data class containing all the necessary data
 * @param point_radius size of the created dots(octagon).
 * @param use_own_color flag to decide wether to use intric color or generate own color scale.
 * @returns returns open3D mesh object.
 */
const open3d::geometry::TriangleMesh & SparseMesh::createMesh(
  const float point_radius, const bool use_own_color, const int mean_min_max,
  const int value_type_index)
{
  // Check if the merging method has changed. If so, we need to completly regenerate the mesh.
  if (
    point_radius != meshed_point_radius_ || use_own_color != meshed_use_own_color_ ||
    value_type_index != meshed_value_type_index_)
  {
    meshed_point_radius_ = point_radius;
    meshed_use_own_color_ = use_own_color;
    meshed_value_type_index_ = value_type_index;
    resetMesh();
  }
  if (mean_min_max != meshed_mean_min_max_) {
    setMergeColors(mean_min_max);
    meshed_mean_min_max_ = mean_min_max;
    resetMesh();
  }

  const uint64_t sparse_data_count = inspection_.getSparseDataCount();
  const std::vector<std::array<double, 3>> & positions = inspection_.getSparsePosition();
  const std::vector<std::array<double, 4>> & orientation = inspection_.getSparseOrientation();
  const std::vector<std::vector<double>> & all_values = inspection_.getSparseValue();
  const std::vector<Eigen::Vector3d> & own_color = inspection_.getSparseUserColor();

  if (!use_own_color) {
    // we ony need these normalized values if we compute the color.
    // these are just the type of values we want to show
    const double new_values_min = inspection_.getSparseMin()[value_type_index];
    const double new_values_max = inspection_.getSparseMax()[value_type_index];
    // when the min and max values change, we need to regenerate the mesh
    if (new_values_min != meshed_min_value_ || new_values_max != meshed_max_value_) {
      resetMesh();
      meshed_min_value_ = new_values_min;
      meshed_max_value_ = new_values_max;
    }
    normalizeValues(&all_values, sparse_data_count);
  }

  for (uint64_t id = meshed_sparse_data_count_; id < sparse_data_count; ++id) {
    std::vector<uint64_t> area_ids =
      inspection_.getSparseMeasurementsInRadius(positions[id], point_radius);
    std::vector<uint64_t> appended_neighbors = commonMembers(area_ids, appended_ids_);
    // Create a new point if there is none in this area or if we use custom colors,
    // since these can not be merged
    if (use_own_color || appended_neighbors.size() == 0) {
      int old_vert_count = mesh_.vertices_.size();
      std::array<Eigen::Vector3d, 9> corners =
        createOctagon(positions[id], orientation[id], point_radius);
      mesh_.vertices_.insert(mesh_.vertices_.end(), std::begin(corners), std::end(corners));
      // We always have 9 vertices, so add the value 9 times
      if (use_own_color) {
        mesh_.vertex_colors_.insert(mesh_.vertex_colors_.end(), 9, own_color[id]);
      } else {
        mesh_.vertex_colors_.insert(
          mesh_.vertex_colors_.end(), 9, getColorValue(normalized_value_array_[id]));
        // remember the original value, if we need to recompute the color later
        point_values_.push_back(normalized_value_array_[id]);
      }

      int new_vert_count = mesh_.vertices_.size();

      // creating new triangles from middle point (old_vert_count) to every new point
      for (int i = 0; (old_vert_count + 3 + i) <= new_vert_count; i++) {
        mesh_.triangles_.push_back(
          {old_vert_count, old_vert_count + i + 2, old_vert_count + i + 1});
      }
      mesh_.triangles_.push_back({old_vert_count, old_vert_count + 1, old_vert_count + 8});
      appended_ids_.push_back(id);
      id_to_appended_index_[id] = appended_ids_.size() - 1;
      merged_points_at_id_[id] = 1;
    } else {
      for (uint64_t neighbor_id : appended_neighbors) {
        // neighbor_id is a point that is already appended
        uint64_t number_of_merged_points = merged_points_at_id_[neighbor_id];

        double new_id_value = normalized_value_array_[id];
        double current_value = point_values_[id_to_appended_index_[neighbor_id]];
        double new_value = merge_values_(current_value, new_id_value, number_of_merged_points);
        point_values_[id_to_appended_index_[neighbor_id]] = new_value;
        Eigen::Vector3d new_color = getColorValue(new_value);
        merged_points_at_id_[neighbor_id] = merged_points_at_id_[neighbor_id] + 1;

        int indx_in_vertex_colors = (id_to_appended_index_[neighbor_id]) * 9;
        for (int i = 0; i < 9; i++) {
          mesh_.vertex_colors_[indx_in_vertex_colors + i] = new_color;
        }
      }
    }
  }
  meshed_sparse_data_count_ = sparse_data_count;

  mesh_.ComputeVertexNormals();
  return mesh_;
}
}  // namespace vinspect
