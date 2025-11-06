// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef VINSPECT__UTILS_H_
#define VINSPECT__UTILS_H_

#include <math.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/TriangleMeshIO.h"

#include <opencv2/opencv.hpp>

#include "vinspect/pose_tree/octree.h"
#include "dense.pb.h"

#include <unsupported/Eigen/EulerAngles>

namespace vinspect
{

open3d::geometry::TriangleMesh meshFromPath(const std::string & mesh_file_path);

float euclideanDistance(
  const double x1, const double y1, const double z1, const double x2, const double y2,
  const double z2);

std::vector<std::string> splitStringArray(
  const std::string str, const std::string delimiter_start = "[",
  const std::string delimiter_end = "]");

Eigen::Vector3d hsv2rgb(const double h, const double s, const double v);

bool isPointInSpace(
  const OrthoTree::BoundingBox3D * inspection_space, const std::array<double, 3> position);

  /**
   * Extracts the pose from an 4D extrisic matrix
   * @param extrinsic_matrix extrinsic matrix
   * @return pose of with orientation as euler angles
   */
std::array<double, 6> transformMatrixToPose(const Eigen::Matrix4d & extrinsic_matrix);
std::array<double, 6> quatToEulerPose(const std::array<double, 7> quat_pose);
std::array<double, 7> eulerToQuatPose(const std::array<double, 6> euler_pose);

Eigen::Matrix4d matrixFromFlatProtoArray(
  const google::protobuf::RepeatedField<double> & flat_array);

  /**
   * Serializes a filled protobuf object.
   * @param i
   * @param sensor_id sensor id of the entry
   * @param color_img color images to be saved
   * @param depth_img depth images to be saved
   * @param extrinsic_optical_matrix the extrinsic calibration of the camera
   * @param extrinsic_world_matrix the location of the camera in the world
   */
std::string serializedStructForDenseEntry(
  int i,
  int sensor_id,
  const cv::Mat & color_img,
  const cv::Mat & depth_img,
  double depth_trunc,
  const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world
);

inline Eigen::Vector3d cast(const std::array<double, 3> & arr)
{
  return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

inline std::array<double, 3> cast(const Eigen::Vector3d & arr)
{
  return std::array<double, 3>({arr.x(), arr.y(), arr.z()});
}

  // Function template for converting std::vector to std::array
template<typename T, std::size_t N>
std::array<T, N> vec2array(const std::vector<T> & vec)
{
  assert(vec.size() == N);
    // Create an array of appropriate type and size
  std::array<T, N> arr;
    // Use memcpy to copy data from vector to array
  std::memcpy(arr.data(), vec.data(), N * sizeof(T));
  return arr;
}

open3d::core::Device selectDevice();

}  // namespace vinspect
#endif  // VINSPECT__UTILS_H_
