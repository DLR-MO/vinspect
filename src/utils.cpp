// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/utils.hpp"

namespace vinspect
{

std::string typeToString(const SensorType type)
{
  switch (type) {
    case SensorType::RGB:
      return "RGB";
    case SensorType::RGBD:
      return "RGBD";
    case SensorType::DEPTH:
      return "Depth";
    case SensorType::SPARSE:
      return "SPARSE";
  }
  return "";
}

SensorType stringToType(const std::string & type)
{
  if (type == "RGB") {
    return SensorType::RGB;
  } else if (type == "RGBD") {
    return SensorType::RGBD;
  } else if (type == "Depth") {
    return SensorType::DEPTH;
  } else if (type == "SPARSE") {
    return SensorType::SPARSE;
  }
  throw std::invalid_argument("Invalid sensor type: " + type);
}

std::vector<SensorType> stringsToTypes(const std::vector<std::string> & types)
{
  std::vector<SensorType> result;
  for (const auto & type : types) {
    result.push_back(stringToType(type));
  }
  return result;
}

std::string array3ToString(const std::array<double, 3> & array)
{
  return "(" + std::to_string(array[0]) + "," + std::to_string(array[1]) + "," +
         std::to_string(array[2]) + ")";
}

std::string array4ToString(const std::array<double, 4> & array)
{
  return "(" + std::to_string(array[0]) + "," + std::to_string(array[1]) + "," +
         std::to_string(array[2]) + "," + std::to_string(array[3]) + ")";
}

std::string vectorToString(const std::vector<double> & vector)
{
  std::stringstream ss;
  ss << "(";
  for (size_t i = 0; i < vector.size(); i++) {
    ss << vector[i];
    if (i < vector.size() - 1) {
      ss << ",";
    }
  }
  ss << ")";
  return ss.str();
}

std::string vectorToString(const Eigen::Vector3d & vector)
{
  std::stringstream ss;
  ss << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
  return ss.str();
}

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

open3d::geometry::TriangleMesh meshFromFilestream(std::ifstream & f)
{
  std::string line;
  std::getline(f, line);
  if (line != "Original mesh: MESH_START") {
    std::cout << line << std::endl;
    throw std::runtime_error("Could not find MESH_START in file.");
  }
  std::string mesh_ascii;
  while (true) {
    std::getline(f, line);
    if (line == "MESH_END") {
      break;
    }
    mesh_ascii += line + "\n";
  }
  open3d::geometry::TriangleMesh mesh;
  // write mesh data to file as we can only read meshes from files
  std::string mesh_path = "/tmp/mesh.ply";
  std::ofstream tmp_file(mesh_path);
  tmp_file << mesh_ascii;
  tmp_file.flush();
  open3d::io::ReadTriangleMesh(mesh_path, mesh);
  std::remove(mesh_path.c_str());
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

std::string writeArray(const std::string & name, const std::vector<double> & values)
{
  std::string result;
  result += name;
  result += ": [";
  for (uint64_t i = 0; i < values.size(); i++) {
    result += "" + std::to_string(values[i]) + "";
    if (i != values.size() - 1) {
      result += ",";
    }
  }
  result += "]\n";
  return result;
}

std::string writeArray(const std::string & name, const std::vector<std::string> & values)
{
  std::string result;
  result += name;
  result += ": [";
  for (uint64_t i = 0; i < values.size(); i++) {
    result += "" + values[i] + "";
    if (i != values.size() - 1) {
      result += ",";
    }
  }
  result += "]\n";
  return result;
}

std::string writeArray(const std::string & name, const std::vector<SensorType> & values)
{
  std::string result;
  result += name;
  result += ": [";
  for (uint64_t i = 0; i < values.size(); i++) {
    result += "" + typeToString(values[i]) + "";
    if (i != values.size() - 1) {
      result += ",";
    }
  }
  result += "]\n";
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

}  // namespace vinspect
