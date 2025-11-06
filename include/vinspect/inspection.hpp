// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef VINSPECT__INSPECTION_H_
#define VINSPECT__INSPECTION_H_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "rocksdb/db.h"
#include "rocksdb/options.h"

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/ModelIO.h"

#include "vinspect/sensors.hpp"
#include "vinspect/pose_tree/octree.h"
#include "vinspect/pose_tree/octree_container.h"
#include "vinspect/utils.hpp"

#include "dense.pb.h"

#include <unsupported/Eigen/EulerAngles>
/*
The unsupported implementation for Euler angles is used in this software because the standard
implementation uses the ranges [0:pi]x[-pi:pi]x[-pi:pi], but with these ranges ambiguous
representations can be achieved. The unsupported implementation overcomes this
problem by using the ranges [-pi, pi], [-pi/2, pi/2], [-pi, pi].

https://eigen.tuxfamily.org/dox/unsupported/group__EulerAngles__Module.html
*/

#define OCTREE_DEPTH 10
#define MAX_POSES_IN_LEAF 21

#define DB_NUMBER_DIGITS_FOR_IDX 24

#define DB_KEY_STATIC_METADATA std::string("/metadata/static")
#define DB_KEY_REFERENCE_MESH std::string("/reference/mesh")
#define DB_KEY_SPARSE_DATA_PREFIX std::string("/data/sparse/")
#define DB_KEY_DENSE_DATA_PREFIX std::string("/data/dense/")

using json = nlohmann::json;

namespace vinspect
{
/**
 *Class for storing and accessing the inspection data.
 **/
class Inspection
{
public:
  /**
   * Constructor for the Inspection class. 
   * @param sparse_value_infos The sparse values / data types used for the inspection.
   * @param dense_sensors The dense sensors used for the inspection.
   * @param mesh The reference mesh of the part that should be inspected.
   * @param save_path The path where the inspection data should be saved.
   * @param inspection_space_3d_min Defines the space in which measurements are recorded
   * @param inspection_space_3d_max Defines the space in which measurements are recorded
   * @param inspection_space_6d_min Defines the space in which pose measurements are recorded
   * @param inspection_space_6d_max Defines the space in which pose measurements are recorded
   **/
  Inspection(
    std::vector<SparseValueInfo> sparse_value_infos,
    std::vector<DenseSensor> dense_sensors,
    open3d::geometry::TriangleMesh mesh,
    std::string save_path, 
    std::array<double, 3> inspection_space_3d_min = {-1, -1, -1},
    std::array<double, 3> inspection_space_3d_max = {1, 1, 1}, 
    std::array<double, 6> inspection_space_6d_min = {-1, -1, -1, -1, -1, -1}, 
    std::array<double, 6> inspection_space_6d_max = {1, 1, 1, 1, 1, 1}
  );

  Inspection(
    std::vector<SparseValueInfo> sparse_value_infos,
    std::vector<DenseSensor> dense_sensors,
    std::string mesh_file_path, 
    std::string save_path,
    std::array<double, 3> inspection_space_3d_min = {-1, -1, -1},
    std::array<double, 3> inspection_space_3d_max = {1, 1, 1}, 
    std::array<double, 6> inspection_space_6d_min = {-1, -1, -1, -1, -1, -1}, 
    std::array<double, 6> inspection_space_6d_max = {1, 1, 1, 1, 1, 1}
  );

  Inspection(const std::string &file_path);

  // Allow move
  Inspection(Inspection&&) = default;
  Inspection& operator=(Inspection&&)      = default;

  // Prohibit copy
  Inspection(const Inspection&) = delete;
  Inspection& operator=(const Inspection&) = delete;

  std::string toString() const;

  /**
   * Returns reference to the open3d mesh
   * @return open3d mesh
   */
  open3d::geometry::TriangleMesh getMesh() const;

  inline bool getDenseUsage() const { return dense_sensors_.size() > 0; }

  /**
   * Returns the dense sensor object given an id
   * @param sensor_id sensor id
   * @return sensor object
   */
  DenseSensor getDenseSensor(int sensor_id) const {
    for (const auto& sensor : dense_sensors_) {
      if (sensor.getId() == sensor_id)
      {
        return sensor;
      }
    }
    throw std::runtime_error(fmt::format("No dense sensor with ID {}!", sensor_id));
  }

  /**
   * Returns the id of the closest sparse measurement to the given position.
   * @param position 3d position
   * @return id of the closest sparse measurement
   */
  int getClosestSparseMeasurement(const std::array<double, 3> & position) const;

  /**
   * Returns  sparse measurements in the given radius
   * @param position 3d position
   * @param radius maximum distance to be considered at this position
   * @return ids of the sparse measurements in the radius
   */
  std::vector<uint64_t> getSparseMeasurementsInRadius(
    const std::array<double, 3> & position, double radius) const;

  /**
   * Returns all datapoints at the given position
   * @param position 3d position
   * @param radius maximum distance to be considered at this position
   * @return numpy array of values at the given position
   */
  std::vector<double> getSparseValuesAtPosition(
    const std::string & value_name, const std::array<double, 3> & position, double radius) const;
  std::vector<double> getSparseValuesForIds(
    const std::string & value_name, const std::vector<uint64_t> & ids) const;

  /**
   * Returns the id of the closest sparse measurement to the given position.
   * @param pose 6d pose
   * @return id of the closest sparse measurement
   */
  int getClosestDenseMeasurement(const std::array<double, 6> & pose) const;

  /**
   * Returns the id of the closest sparse measurement to the given position.
   * @param pose 7d quaternion pose
   * @return id of the closest sparse measurement
   */
  int getClosestDenseMeasurement(const std::array<double, 7> & quat_pose) const;

  inline uint64_t getDenseDataCount() const {return dense_data_count_;}

  /**
   * Returns all dense at the given position.
   * @param position 6d pose
   * @param radius maximum distance to be considered at this position
   * @return numpy array of dense measurements at the given position
   */
  std::vector<double> getDenseAtPose(
    const std::array<double, 6> & pose, double radius) const;

  /**
   * Returns a image retrieved from the database based on the given id.
   * @param sample_id id of the sample
   * @return image as a std::vector<std::vector<uint8_t>>
   */
  std::vector<std::vector<std::array<u_int8_t, 3>>> getImageFromId(const int sample_id) const;

  /**
   * Returns the pose which is retrieved from the database based on the given id.
   * @param sample_id id of the sample
   * @return image as a std::vector<std::vector<uint8_t>>
   */
  std::array<double, 6> getDensePoseFromId(const int sample_id) const;

  /**
   * Returns a percentage of all poses available.
   * @param percentage percentage poses requested
   * @return multiple arrays of 6D poses in a vector, as a std::vector<std::array<double, 6>>
   */
  std::vector<std::array<double, 6>> getMultiDensePoses(const int percentage) const;

  /**
   * Adds a new data point to the inspection
   * @param position 3d position
   * @param values numpy array of data values
   * @param raw_id id of the raw measurements
   * @param insert_in_octree can be set to false if the octree is later recreated to improve performance
   */
  void addSparseMeasurement(
    double timestamp, int sensor_id, const std::array<double, 3> & position,
    const std::array<double, 4> & orientation, const std::vector<double> & values,
    const Eigen::Vector3d & user_color = {0.0, 0.0, 0.0}, bool insert_in_octree = true)
  {
    addSparseMeasurementImpl(timestamp, sensor_id, position, orientation, values, user_color, insert_in_octree, true);
  }

  /**
   * Recreates the octrees. Useful if the inspection space should be adapted to the current data.
   */
  void recreateOctrees();
  void reinitializeTSDF(double voxel_length);

  void clear();

  void addImage(
    const open3d::geometry::RGBDImage & image, const int sensor_id,
    const Eigen::Matrix4d & extrinsic_optical, const Eigen::Matrix4d & extrinsic_world)
  {
    addImageImpl(image, sensor_id, extrinsic_optical, extrinsic_world, true);
  }

  std::shared_ptr<open3d::geometry::TriangleMesh> extractDenseReconstruction() const;
  
  void saveDenseReconstruction(std::string filename) const;

  inline uint64_t getSparseDataCount() const {return sparse_data_count_;}
  inline const std::vector<std::array<double, 3>> & getSparsePosition() const
  {
    return sparse_position_;
  }
  inline const std::vector<std::array<double, 4>> & getSparseOrientation() const
  {
    return sparse_orientation_;
  }
  inline bool getSparseUsage() const { return sparse_value_infos_.size() > 0; }
  inline const std::vector<std::vector<double>> & getSparseValue() const {return sparse_value_;}
  inline const std::vector<Eigen::Vector3d> & getSparseUserColor() const
  {
    return sparse_user_color_;
  }
  inline const std::vector<double> & getSparseTimestamp() const {return sparse_timestamp_;}
  inline double getSparseMin(std::size_t dim) const
  {
    return sparse_min_values_[dim];
  }

  inline double getSparseMax(std::size_t dim) const
  {
    return sparse_max_values_[dim];
  }

  void setIntrinsic(open3d::camera::PinholeCameraIntrinsic intrinsic, int sensor_id)
  {
    intrinsic_[sensor_id] = intrinsic;
  }

  void setIntrinsic2( int sensor_id, int width, int height, double fx, double fy, double cx, double cy)
  {
    intrinsic_[sensor_id] = open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, cx, cy);
  }

  std::vector<std::string> getSparseUnits() {
    std::vector<std::string> sparse_units;
    for (const auto & value_info : sparse_value_infos_) {
      sparse_units.push_back(value_info.unit);
    }
    return sparse_units;
  }

  uint64_t getIntegratedImagesCount() {return dense_data_count_;}

private:
  /**
   * Loads or creates the DB file if it does not exist
   */
  void initDB(const std::string &file_path);

  /**
   * Shared setup code between the constructors
   */
  void setupSensors();

  /**
   * Stores the static meta data of the inspection
   * @return true if the save was successful
   */
  bool saveMetaData();

  /**
   * Stores mesh in the DB.
   * @param key the key under which the mesh should be stored
   * @param mesh the mesh to be stored
   */
  void storeMesh(const std::string &key, const open3d::geometry::TriangleMesh &mesh);

  /**
   * Loads a mesh from the DB.
   * @param key the key from which we want to load the mesh
   * @param mesh we load into this mesh
   */
  void loadMesh(const std::string &key, open3d::geometry::TriangleMesh &mesh);

  void addSparseMeasurementImpl(
    double timestamp, int sensor_id, const std::array<double, 3> & position,
    const std::array<double, 4> & orientation, const std::vector<double> & values,
    const Eigen::Vector3d & user_color = {0.0, 0.0, 0.0}, bool insert_in_octree = true, bool store_in_database = true);

  void addImageImpl(
    const open3d::geometry::RGBDImage & image, const int sensor_id,
    const Eigen::Matrix4d & extrinsic_optical, const Eigen::Matrix4d & extrinsic_world, bool store_in_database = true);


  std::vector<SparseValueInfo> sparse_value_infos_;
  std::vector<DenseSensor> dense_sensors_;
  uint64_t sparse_data_count_ = 0, dense_data_count_ = 0;
  open3d::geometry::TriangleMesh reference_mesh_;
  OrthoTree::OctreePointC sparse_octree_;
  OrthoTree::TreePointPoseND<6, {0, 0, 0, 1, 1, 1}, std::ratio<1, 2>, double> dense_posetree_;
  std::vector<double> sparse_timestamp_, dense_timestamp_;
  std::vector<std::array<double, 3>> sparse_position_;
  std::vector<std::array<double, 6>> dense_pose_;
  std::vector<std::array<double, 4>> sparse_orientation_, dense_orientation_;
  std::vector<std::vector<double>> sparse_value_;
  std::vector<Eigen::Vector3d> sparse_user_color_;
  OrthoTree::BoundingBox3D inspection_space_3d_;
  OrthoTree::BoundingBoxND<6> inspection_space_6d_;
  std::vector<double> sparse_min_values_;
  std::vector<double> sparse_max_values_;
  open3d::core::Device o3d_device_;
  open3d::t::geometry::VoxelBlockGrid voxel_grid_;
  std::map<int, open3d::camera::PinholeCameraIntrinsic> intrinsic_;
  open3d::geometry::AxisAlignedBoundingBox crop_box_;
  std::unique_ptr<rocksdb::DB> db_;
  rocksdb::Options db_options_;
  rocksdb::Status db_status_;
  std::mutex mtx_;
};
}  // namespace vinspect
#endif  // VINSPECT__INSPECTION_H_
