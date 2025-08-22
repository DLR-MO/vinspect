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
void showColoredMesh(Inspection inspection, double point_radius)
{
  open3d::geometry::TriangleMesh input_mesh = open3d::geometry::TriangleMesh();
  SparseMesh sparse_mesh = SparseMesh(inspection);
  open3d::geometry::TriangleMesh mesh = sparse_mesh.createMesh(point_radius, false, 0, 0);
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
    std::make_shared<open3d::geometry::TriangleMesh>(mesh);
  open3d::visualization::DrawGeometries({mesh_ptr});
}

// this is just a function for testing purposes
void computeColoredMesh(Inspection inspection, double point_radius)
{
  open3d::geometry::TriangleMesh input_mesh = open3d::geometry::TriangleMesh();
  SparseMesh sparse_mesh = SparseMesh(inspection);
  open3d::geometry::TriangleMesh mesh = sparse_mesh.createMesh(point_radius, false, 0, 0);
}

template<typename T>
open3d::geometry::Image imagePyToCpp(const py::array_t<T> image, std::string encoding, bool is_depth_img)
{
  //TODO make sure that there are no copy opertions happening here
  int rows = image.shape(0);
  int cols = image.shape(1);
  int channels;
  int stride;
  if (is_depth_img) {
    channels = 1;
    stride = 0;
  } else {
    channels = image.shape(2);
    stride = image.strides(0);
  }
  int bytes_per_channel;
  int color_encoding_int;
  if (encoding == "8U") {
    bytes_per_channel = 1;
    color_encoding_int = CV_8U;
  } else if (encoding == "16U") {
    bytes_per_channel = 2;
    color_encoding_int = CV_16U;
  } else if (encoding == "32F") {
    bytes_per_channel = 4;
    color_encoding_int = CV_32F;
  } else {
    std::cout << "Image encoding " << encoding << " can not be processed" << std::endl;
    exit(1);
  }
  cv::Mat cv_mat = cv::Mat(rows, cols, CV_MAKETYPE(color_encoding_int, channels), const_cast<T * >(image.data()), stride);
  open3d::geometry::Image img;
  img.Prepare(cols, rows, channels, bytes_per_channel);
  memcpy(img.data_.data(), cv_mat.data, img.data_.size());
  return img;
}

void integrateImage8_16(
  Inspection * inspection, const py::array_t<uint8_t> & color_image, std::string color_encoding, 
  const py::array_t<uint16_t> & depth_image, std::string depth_encoding, float depth_scale,
  float depth_trunc, const int sensor_id, const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world)
{
  open3d::geometry::Image color_img = imagePyToCpp<uint8_t>(color_image, color_encoding, false);
  open3d::geometry::Image depth_img = imagePyToCpp<uint16_t>(depth_image, depth_encoding, true);
  inspection->integrateImage(color_img, depth_img, depth_scale, depth_trunc, sensor_id, extrinsic_optical, extrinsic_world);
}

void integrateImage8_float(
  Inspection * inspection, const py::array_t<uint8_t> & color_image, std::string color_encoding, 
  const py::array_t<float> & depth_image, std::string depth_encoding, float depth_scale,
  float depth_trunc, const int sensor_id, const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world)
{
  open3d::geometry::Image color_img = imagePyToCpp<uint8_t>(color_image, color_encoding, false);
  open3d::geometry::Image depth_img = imagePyToCpp<float>(depth_image, depth_encoding, true);

  inspection->integrateImage(color_img, depth_img, depth_scale, depth_trunc, sensor_id, extrinsic_optical, extrinsic_world);
}


PYBIND11_MODULE(vinspect_py, m)
{
  m.doc() = "Vinspect Python bindings";

  py::class_<Inspection>(m, "Inspection")
  .def(
    py::init<
      std::vector<std::string>, std::vector<std::string>, std::vector<std::string>,
      std::vector<std::string>, std::string, std::tuple<int, int>, std::string,
      std::array<double, 3>, std::array<double, 3>, std::array<double, 6>, std::array<double, 6>,
      std::vector<double>, std::vector<double>>(),
    py::arg("sensor_types_names"), py::arg("sparse_types") = std::vector<std::string>(),
    py::arg("sparse_units") = std::vector<std::string>(),
    py::arg("joint_names") = std::vector<std::string>(), py::arg("mesh_file_path") = "",
    py::arg("dense_sensor_resolution") = std::tuple<int, int>(), py::arg("save_path") = "",
    py::arg("inspection_space_3d_min") = std::array<double, 3>(),
    py::arg("inspection_space_3d_max") = std::array<double, 3>(),
    py::arg("inspection_space_6d_min") = std::array<double, 6>(),
    py::arg("inspection_space_6d_max") = std::array<double, 6>(),
    py::arg("sparse_min_values") = std::vector<double>(),
    py::arg("sparse_max_values") = std::vector<double>())
  .def("add_sparse_measurement", &Inspection::addSparseMeasurement)
  .def("finish", &Inspection::finish)
  .def("get_closest_sparse_measurement", &Inspection::getClosestSparseMeasurement)
  .def("get_integrated_images_count", &Inspection::getIntegratedImagesCount)
  .def("get_sparse_measurements_in_radius", &Inspection::getSparseMeasurementsInRadius)
  .def("get_sparse_data_count", &Inspection::getSparseDataCount)
  .def("get_sparse_position", &Inspection::getSparsePosition)
  .def("get_sparse_orientation", &Inspection::getSparseOrientation)
  .def("get_sparse_value", &Inspection::getSparseValue)
  .def("get_sparse_user_color", &Inspection::getSparseUserColor)
  .def("get_sparse_timestamp", &Inspection::getSparseTimestamp)
  .def("integrate_image", &Inspection::integrateImage)
  .def("get_mesh", &Inspection::getMesh)
  .def("reinitialize_TSDF", &Inspection::reinitializeTSDF)
  .def("save", &Inspection::save)
  .def("save_dense_reconstruction", &Inspection::saveDenseReconstruction)
  .def("set_intrinsic", &Inspection::setIntrinsic)
  .def("set_intrinsic2", &Inspection::setIntrinsic2);
  m.def("load", &load);
  m.def("show_colored_mesh", &showColoredMesh);
  m.def("compute_colored_mesh", &computeColoredMesh);
  m.def("integrate_image_py", &integrateImage8_16);
  m.def("integrate_image_py", &integrateImage8_float);
}
}  // namespace vinspect
#endif
