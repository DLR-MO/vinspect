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

// WARNING! This methods uses a zero-copy mechanism. 
// The lifetime of the image data is still bound to the original python object
// make sure the input object outlives the result from this function! 
template<int CVType, typename T>
cv::Mat imagePyToCpp(const py::array_t<T> & image)
{
  auto rows = image.shape(0);
  auto cols = image.shape(1);

  cv::Mat cv_image(rows, cols, CVType, (unsigned char*)image.data());
  return cv_image;
}

void addImage_16(
  Inspection * inspection, const py::array_t<uint8_t> & color_image, 
  const py::array_t<uint16_t> & depth_image,
  float depth_trunc, const int sensor_id, const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world)
{
  auto color_img = imagePyToCpp<CV_8UC3, uint8_t>(color_image);
  auto depth_img = imagePyToCpp<CV_16UC1, uint16_t>(depth_image);
  inspection->addImage(color_img, depth_img, depth_trunc, sensor_id, extrinsic_optical, extrinsic_world);
}

void addImage_float(
  Inspection * inspection, const py::array_t<uint8_t> & color_image,
  const py::array_t<float> & depth_image,
  float depth_trunc, const int sensor_id, const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world)
{
  auto color_img = imagePyToCpp<CV_8UC3, uint8_t>(color_image);
  auto depth_img = imagePyToCpp<CV_32FC1, float>(depth_image);
  inspection->addImage(color_img, depth_img, depth_trunc, sensor_id, extrinsic_optical, extrinsic_world);
}


PYBIND11_MODULE(vinspect_py, m)
{
  m.doc() = "Vinspect Python bindings";

  py::class_<Inspection, std::unique_ptr<Inspection>>(m, "Inspection")
  .def(
    py::init<
      std::vector<SparseValueInfo>, 
      std::vector<DenseSensor>,
      std::string, 
      std::string,
      std::array<double, 3>, 
      std::array<double, 3>, 
      std::array<double, 6>, 
      std::array<double, 6>>(),
    py::arg("sparse_value_infos"),
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

  py::class_<SparseValueInfo, std::unique_ptr<SparseValueInfo>>(m, "ValueInfo")
  .def(
    py::init<std::string, std::string>(),
    py::arg("name"), py::arg("unit")
  )
  .def_readwrite("name", &SparseValueInfo::name)
  .def_readwrite("unit", &SparseValueInfo::unit);

  py::class_<DenseSensor, std::unique_ptr<DenseSensor>>(m, "DenseSensor")
  .def(
    py::init<int, unsigned int, unsigned int, double>(),
    py::arg("id"), py::arg("width"), py::arg("height"), py::arg("depth_scale")
  )
  .def("num_pixels", &DenseSensor::numPixels)
  .def("get_width", &DenseSensor::getWidth)
  .def("get_height", &DenseSensor::getHeight)
  .def("get_resolution", &DenseSensor::getResolution);

  m.def("show_colored_mesh", &showColoredMesh);
  m.def("compute_colored_mesh", &computeColoredMesh);
  m.def("add_image_py", &addImage_float);
  m.def("add_image_py", &addImage_16);
}
}  // namespace vinspect
#endif
