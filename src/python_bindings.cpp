// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef python_bindings_HPP
#define python_bindings_HPP
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vinspect/sparse_mesh.hpp>
#include <vinspect/inspection.hpp>
#include <vinspect/sensors.hpp>
#include <vinspect/utils.hpp>

#include <vector>

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/geometry/RGBDImage.h"
#include "open3d/visualization/utility/Draw.h"

#include <opencv2/opencv.hpp>

namespace py = pybind11;
namespace vinspect
{
// this is just a function for testing purposes
void showColoredMesh(Inspection & inspection, double point_radius)
{
  open3d::geometry::TriangleMesh input_mesh = open3d::geometry::TriangleMesh();
  SparseMesh sparse_mesh = SparseMesh(inspection);
  open3d::geometry::TriangleMesh mesh = sparse_mesh.createMesh(point_radius, false, 0, 0);
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
    std::make_shared<open3d::geometry::TriangleMesh>(mesh);
  open3d::visualization::DrawGeometries({mesh_ptr});
}

// this is just a function for testing purposes
void computeColoredMesh(Inspection & inspection, double point_radius)
{
  open3d::geometry::TriangleMesh input_mesh = open3d::geometry::TriangleMesh();
  SparseMesh sparse_mesh = SparseMesh(inspection);
  open3d::geometry::TriangleMesh mesh = sparse_mesh.createMesh(point_radius, false, 0, 0);
}

void addImagePy(
  Inspection & inspection,
  const py::array_t<uint8_t> & color_image, std::string color_encoding,
  const py::array_t<uint16_t> & depth_image, std::string depth_encoding, float depth_scale,
  float depth_trunc, const int sensor_id,
  const Eigen::Matrix4d & extrinsic_optical, const Eigen::Matrix4d & extrinsic_world)
{
  int color_rows = color_image.shape(0);
  int color_cols = color_image.shape(1);
  int color_channels = color_image.shape(2);
  int color_stride = color_image.strides(0);
  int color_bytes_per_channel;
  int color_encoding_int;
  if (color_encoding == "8U") {
    color_encoding_int = CV_8U;
    color_bytes_per_channel = 1;
  } else {
    std::cout << "Color image encoding " << color_encoding << " can not be processed" << std::endl;
    exit(1);
  }
  cv::Mat color_mat(color_rows, color_cols, CV_MAKETYPE(color_encoding_int, color_channels),
    const_cast<uint8_t *>(color_image.data()), color_stride);
  open3d::geometry::Image color_img;
  color_img.Prepare(color_cols, color_rows, color_channels, color_bytes_per_channel);
  memcpy(color_img.data_.data(), color_mat.data, color_img.data_.size());

  int depth_rows = depth_image.shape(0);
  int depth_cols = depth_image.shape(1);
  int depth_channels = 1; //always the case for depth images
  int depth_stride = 0;
  int depth_bytes_per_channel;
  int depth_encoding_int;
  if (depth_encoding == "16U") {
    depth_encoding_int = CV_16U;
    depth_bytes_per_channel = 2;
  } else {
    std::cout << "Depth image encoding " << depth_encoding << " can not be processed" << std::endl;
    exit(1);
  }
  cv::Mat depth_mat(depth_rows, depth_cols, CV_MAKETYPE(depth_encoding_int, depth_channels),
    const_cast<uint16_t *>(depth_image.data()), depth_stride);
  open3d::geometry::Image depth_img;
  depth_img.Prepare(depth_cols, depth_rows, depth_channels, depth_bytes_per_channel);
  memcpy(depth_img.data_.data(), depth_mat.data, depth_img.data_.size());

  std::shared_ptr<open3d::geometry::RGBDImage> rgbd =
    open3d::geometry::RGBDImage::CreateFromColorAndDepth(color_img, depth_img, depth_scale,
      depth_trunc, false);
  inspection.addImage(*rgbd.get(), sensor_id, extrinsic_optical, extrinsic_world);
}


PYBIND11_MODULE(vinspect_py, m)
{
  m.doc() = "Vinspect Python bindings";

  py::class_<Inspection, std::unique_ptr<Inspection>>(m, "Inspection")
  .def(
    py::init<
      std::vector<SparseSensor>, 
      std::vector<DenseSensor>,
      std::string, 
      std::string,
      std::array<double, 3>, 
      std::array<double, 3>, 
      std::array<double, 6>, 
      std::array<double, 6>>(),
    py::arg("sparse_sensors"), 
    py::arg("dense_sensors"), 
    py::arg("reference_mesh_file_path") = "",
    py::arg("save_path") = "",
    py::arg("inspection_space_3d_min") = std::array<double, 3>(),
    py::arg("inspection_space_3d_max") = std::array<double, 3>(),
    py::arg("inspection_space_6d_min") = std::array<double, 6>(),
    py::arg("inspection_space_6d_max") = std::array<double, 6>()
  )
  .def(py::init<std::string>(), py::arg("save_path") = "")
  .def("add_sparse_measurement", &Inspection::addSparseMeasurement)
  .def("get_closest_sparse_measurement", &Inspection::getClosestSparseMeasurement)
  .def("get_integrated_images_count", &Inspection::getIntegratedImagesCount)
  .def("get_sparse_measurements_in_radius", &Inspection::getSparseMeasurementsInRadius)
  .def("get_sparse_data_count", &Inspection::getSparseDataCount)
  .def("get_sparse_position", &Inspection::getSparsePosition)
  .def("get_sparse_orientation", &Inspection::getSparseOrientation)
  .def("get_sparse_value", &Inspection::getSparseValue)
  .def("get_sparse_user_color", &Inspection::getSparseUserColor)
  .def("get_sparse_timestamp", &Inspection::getSparseTimestamp)
  .def("add_image", &Inspection::addImage)
  .def("get_mesh", &Inspection::getMesh)
  .def("reinitialize_TSDF", &Inspection::reinitializeTSDF)
  .def("save_dense_reconstruction", &Inspection::saveDenseReconstruction)
  .def("set_intrinsic", &Inspection::setIntrinsic)
  .def("set_intrinsic2", &Inspection::setIntrinsic2);

  py::class_<ValueInfo, std::unique_ptr<ValueInfo>>(m, "ValueInfo")
  .def(
    py::init<std::string, std::string>(),
    py::arg("name"), py::arg("unit")
  )
  .def_readwrite("name", &ValueInfo::name)
  .def_readwrite("unit", &ValueInfo::unit);

  py::class_<SparseSensor, std::unique_ptr<SparseSensor>>(m, "SparseSensor")
  .def(
    py::init<int, std::vector<ValueInfo>>(),
    py::arg("id"), py::arg("value_infos")
  )
  .def("get_value_infos", &SparseSensor::getValueInfos)
  .def("get_num_values", &SparseSensor::numValues);

  py::class_<DenseSensor, std::unique_ptr<DenseSensor>>(m, "DenseSensor")
  .def(
    py::init<int, unsigned int, unsigned int>(),
    py::arg("id"), py::arg("width"), py::arg("height")
  )
  .def("num_pixels", &DenseSensor::numPixels)
  .def("get_width", &DenseSensor::getWidth)
  .def("get_height", &DenseSensor::getHeight)
  .def("get_resolution", &DenseSensor::getResolution);

  m.def("show_colored_mesh", &showColoredMesh);
  m.def("compute_colored_mesh", &computeColoredMesh);
  m.def("add_image_py", &addImagePy);
}
}  // namespace vinspect
#endif
