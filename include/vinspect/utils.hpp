// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef VINSPECT__UTILS_H_
#define VINSPECT__UTILS_H_

#include <math.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/TriangleMeshIO.h"
#include "vinspect/Octree/octree.h"
namespace vinspect
{

// Enum representing different usable sensor types
enum class SensorType
{
  RGB = 0,
  RGBD = 1,
  DEPTH = 2,
  SPARSE = 3
};

std::string typeToString(const SensorType type);
SensorType stringToType(const std::string & type);
std::vector<SensorType> stringsToTypes(const std::vector<std::string> & types);

std::string writeArray(const std::string & name, const std::vector<double> & values);
std::string writeArray(const std::string & name, const std::vector<std::string> & values);
std::string writeArray(const std::string & name, const std::vector<SensorType> & values);

std::string array3ToString(const std::array<double, 3> & array);
std::string array4ToString(const std::array<double, 4> & array);
std::string vectorToString(const std::vector<double> & vector);
std::string vectorToString(const Eigen::Vector3d & vector);

open3d::geometry::TriangleMesh meshFromPath(const std::string & mesh_file_path);
open3d::geometry::TriangleMesh meshFromFilestream(std::ifstream & f);

float euclideanDistance(
  const double x1, const double y1, const double z1, const double x2, const double y2,
  const double z2);

std::vector<std::string> splitStringArray(
  const std::string str, const std::string delimiter_start = "[",
  const std::string delimiter_end = "]");

Eigen::Vector3d hsv2rgb(const double h, const double s, const double v);

bool isPointInSpace(
  const OrthoTree::BoundingBox3D * inspection_space, const std::array<double, 3> position);

}  // namespace vinspect
#endif  // VINSPECT__UTILS_H_
