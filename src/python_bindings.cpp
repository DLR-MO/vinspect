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
#include "open3d/visualization/utility/Draw.h"

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

PYBIND11_MODULE(vinspect_py, m)
{
  m.doc() = "Vinspect Python bindings";

  py::class_<Inspection>(m, "Inspection")
  .def(
    py::init<
      std::vector<std::string>, std::vector<std::string>, std::vector<std::string>,
      std::vector<std::string>, std::string, std::tuple<int, int>, std::string,
      std::array<double, 3>, std::array<double, 3>, std::vector<double>, std::vector<double>>(),
    py::arg("sensor_types_names"), py::arg("sparse_types") = std::vector<std::string>(),
    py::arg("sparse_units") = std::vector<std::string>(),
    py::arg("joint_names") = std::vector<std::string>(), py::arg("mesh_file_path") = "",
    py::arg("dense_sensor_resolution") = std::tuple<int, int>(), py::arg("save_path") = "",
    py::arg("inspection_space_min") = std::array<double, 3>(),
    py::arg("inspection_space_max") = std::array<double, 3>(),
    py::arg("sparse_min_values") = std::vector<double>(),
    py::arg("sparse_max_values") = std::vector<double>())
  .def("add_sparse_measurement", &Inspection::addSparseMeasurement)
  .def("extract_dense_reconstruction", &Inspection::extractDenseReconstruction)
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
  .def("set_intrinsic", &Inspection::setIntrinsic);
  m.def("load", &load);
  m.def("show_colored_mesh", &showColoredMesh);
  m.def("compute_colored_mesh", &computeColoredMesh);
}
}  // namespace vinspect
#endif
