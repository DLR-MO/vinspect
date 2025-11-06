// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/utils.hpp"

namespace vinspect
{

open3d::geometry::TriangleMesh meshFromPath(const std::string & mesh_file_path)
{
  open3d::geometry::TriangleMesh mesh;
  if (mesh_file_path.empty()) {
    return mesh;
  }else if (!open3d::io::ReadTriangleMesh(mesh_file_path, mesh)) {
    throw std::runtime_error("Cannot read mesh file: " + mesh_file_path);
  }
  return mesh;
}

float euclideanDistance(
  const double x1, const double y1, const double z1, const double x2, const double y2,
  const double z2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

std::vector<std::string> splitStringArray(
  const std::string str, const std::string delimiter_start, const std::string delimiter_end)
{
  std::vector<std::string> result;
  std::stringstream ss(str.substr(str.find(delimiter_start) + 1, str.find(delimiter_end)));
  // hacky way to get rid of the ]
  std::string str2 = ss.str();
  str2.pop_back();
  ss = std::stringstream(str2);
  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    result.push_back(substr);
  }
  return result;
}

Eigen::Vector3d hsv2rgb(const double h, const double s, const double v)
{
  double hh, p, q, t, ff, r, g, b;
  int64_t i;

  hh = h;
  if (hh >= 360.0) {hh = 0.0;}
  hh /= 60.0;
  i = (int64_t)hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));

  switch (i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
    default:
      r = v;
      g = p;
      b = q;
      break;
  }
  return Eigen::Vector3d(r, g, b);
}

bool isPointInSpace(
  const OrthoTree::BoundingBox3D * inspection_space, const std::array<double, 3> position)
{
  return inspection_space->Min[0] <= position[0];
}

std::array<double, 6> transformMatrixToPose(const Eigen::Matrix4d & extrinsic_matrix)
{
  std::array<double, 6> pose;
  Eigen::Vector3d translation = extrinsic_matrix.block<3, 1>(0, 3);

  Eigen::Matrix3d rotationMatrix = extrinsic_matrix.block<3, 3>(0, 0);
  Eigen::EulerAnglesXYZd euler_angles(rotationMatrix);

  pose[0] = translation[0];
  pose[1] = translation[1];
  pose[2] = translation[2];

  pose[3] = euler_angles.alpha();
  pose[4] = euler_angles.beta();
  pose[5] = euler_angles.gamma();

  return pose;
}


std::array<double, 6> quatToEulerPose(const std::array<double, 7> quat_pose)
{
  std::array<double, 6> euler_pose;

  Eigen::Quaterniond quat(quat_pose[6], quat_pose[3], quat_pose[4], quat_pose[5]);
  Eigen::EulerAnglesXYZd euler_angles(quat.toRotationMatrix());

  euler_pose[0] = quat_pose[0];
  euler_pose[1] = quat_pose[1];
  euler_pose[2] = quat_pose[2];

  euler_pose[3] = euler_angles.alpha();
  euler_pose[4] = euler_angles.beta();
  euler_pose[5] = euler_angles.gamma();

  return euler_pose;
}

std::array<double, 7> eulerToQuatPose(const std::array<double, 6> euler_pose)
{
  std::array<double, 7> quat_pose;

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(euler_pose[3], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(
    euler_pose[4],
    Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_pose[5], Eigen::Vector3d::UnitZ());

  quat_pose[0] = euler_pose[0];
  quat_pose[1] = euler_pose[1];
  quat_pose[2] = euler_pose[2];

  quat_pose[3] = q.x();
  quat_pose[4] = q.y();
  quat_pose[5] = q.z();
  quat_pose[6] = q.w();

  return quat_pose;
}

std::string serializedStructForDenseEntry(
  int i,
  int sensor_id,
  const cv::Mat & color_img,
  const cv::Mat & depth_img,
  double depth_trunc,
  const Eigen::Matrix4d & extrinsic_optical, 
  const Eigen::Matrix4d & extrinsic_world
)
{
  //Fill the Protobuf object

  Dense denseEntry;
  denseEntry.set_entry_nr(i);
  denseEntry.set_sensor_id(sensor_id);
  denseEntry.set_depth_trunc(depth_trunc);
  denseEntry.set_depth_dtype(depth_img.type());

  auto * extrinsic_optical_matrix_field = denseEntry.mutable_extrinsic_optical_matrix();
  extrinsic_optical_matrix_field->Add(extrinsic_optical.reshaped().begin(), extrinsic_optical.reshaped().end());

  auto * extrinsic_world_field = denseEntry.mutable_extrinsic_world_matrix();
  extrinsic_world_field->Add(extrinsic_world.reshaped().begin(), extrinsic_world.reshaped().end());

  denseEntry.set_color_image(color_img.data, color_img.cols * color_img.rows * color_img.elemSize());
  denseEntry.set_depth_image(depth_img.data, depth_img.cols * depth_img.rows * depth_img.elemSize());

  return denseEntry.SerializeAsString();
}

Eigen::Matrix4d matrixFromFlatProtoArray(const google::protobuf::RepeatedField<double>& flat_array){
  if (flat_array.size() != 16) {
        throw std::invalid_argument("Extrinsic optical matrix must have exactly 16 elements");
  }
  Eigen::Matrix4d matrix;
  for (int i = 0; i < 16; ++i) {
      int row = i / 4;
      int col = i % 4;
      matrix(row, col) = flat_array[i];
  }
  return matrix;
}

open3d::core::Device selectDevice()
{
  std::vector<open3d::core::Device> cuda_devices = open3d::core::Device::GetAvailableCUDADevices();
  if (cuda_devices.size() > 0) {
    return cuda_devices[0];
  } else {
    return open3d::core::Device("CPU:0");
  }
}

}  // namespace vinspect
