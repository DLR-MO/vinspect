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
#include <vector>

#include "rocksdb/db.h"
#include "rocksdb/options.h"
#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/ModelIO.h"
#include "open3d/pipelines/integration/TSDFVolume.h"
#include "vinspect/Octree/octree.h"
#include "vinspect/Octree/octree_container.h"
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

namespace vinspect
{

/**
 *Class for storing and accessing the inspection data.
 **/
class Inspection
{
public:
  /**
   * Constructor for the Inspection class. Although, different sensors can be
   * used together, currently only one type of dense sensor with one resolution
   * is supported.
   * @param sensor_types The sensor types that will be used.
   * @param sparse_types The sparse data type names. If multiple
   *sensors are used, this is the sum of their data type names.
   * @param sparse_units The units of the sparse data, only used for visualization.
   * @param joint_names The joint names, if a robot is used.
   * @param mesh The reference mesh of the part that should be inspected.
   * @param dense_sensor_resolution The dense sensor resolution.
   * @param save_path The path where the inspection data should be saved. If
   *None is given, the inspection data will not be saved.
   * @param inspection_space_3d_min Defines the space in which measurments are recorded
   * @param inspection_space_3d_max Defines the space in which measurments are recorded
   * @param inspection_space_6d_min Defines the space in which pose measurments are recorded
   * @param inspection_space_6d_max Defines the space in which pose measurments are recorded
   * @param sparse_color_min_values Defines the cutoff of the color range
   * @param sparse_color_max_values Defines the cutoff of the color range
   **/
  Inspection(
    std::vector<SensorType> sensor_types, std::vector<std::string> sparse_types,
    std::vector<std::string> sparse_units, std::vector<std::string> joint_names,
    open3d::geometry::TriangleMesh mesh, std::tuple<int, int> dense_sensor_resolution,
    std::string save_path, std::array<double, 3> inspection_space_3d_min = {-1, -1, -1},
    std::array<double, 3> inspection_space_3d_max = {1, 1, 1}, std::array<double,
    6> inspection_space_6d_min = {-1, -1, -1, -1, -1, -1}, std::array<double,
    6> inspection_space_6d_max = {1, 1, 1, 1, 1, 1},
    std::vector<double> sparse_color_min_values = {},
    std::vector<double> sparse_color_max_values = {});

  Inspection(
    std::vector<std::string> sensor_types_names, std::vector<std::string> sparse_types,
    std::vector<std::string> sparse_units, std::vector<std::string> joint_names,
    std::string mesh_file_path, std::tuple<int, int> dense_sensor_resolution, std::string save_path,
    std::array<double, 3> inspection_space_3d_min = {-1, -1, -1},
    std::array<double, 3> inspection_space_3d_max = {1, 1, 1}, std::array<double,
    6> inspection_space_6d_min = {-1, -1, -1, -1, -1, -1}, std::array<double,
    6> inspection_space_6d_max = {1, 1, 1, 1, 1, 1},
    std::vector<double> sparse_color_min_values = {},
    std::vector<double> sparse_color_max_values = {});
  Inspection();

  std::string toString() const;
  void startSaving();

  /**
   * Returns reference to the open3d mesh
   * @return open3d mesh
   */
  open3d::geometry::TriangleMesh getMesh() const;

  /**
   * Returns the id of the closest sparse measurment to the given position.
   * @param position 3d position
   * @return id of the closest sparse measurement
   */
  int getClosestSparseMeasurement(const std::array<double, 3> & position) const;

  /**
   * Returns  sparse measurments in the given radius
   * @param position 3d position
   * @param radius maximum distance to be considered at this position
   * @return ids of the sparse measurments in the radius
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
    const std::string & value_type, const std::array<double, 3> & position, double radius) const;
  std::vector<double> getValuesForIds(
    const std::string & value_type, const std::vector<uint64_t> & ids) const;

  /**
   * Returns the id of the closest sparse measurment to the given position.
   * @param pose 6d pose
   * @return id of the closest sparse measurement
   */
  int getClosestDenseMeasurement(const std::array<double, 6> & pose) const;

  /**
   * Returns the id of the closest sparse measurment to the given position.
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
   * Returns a image retrived from the database based on the given id.
   * @param id id of the datapoint in the database
   * @return image as a std::vector<std::vector<uint8_t>>
   */
  std::vector<std::vector<std::array<u_int8_t, 3>>> getImageFromId(const int id) const;

  /**
   * Returns the pose which is retrived from the database based on the given id.
   * @param id id of the datapoint in the database
   * @return image as a std::vector<std::vector<uint8_t>>
   */
  std::array<double, 6> getDensePoseFromId(const int id) const;

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
    const Eigen::Vector3d & user_color = {0.0, 0.0, 0.0}, bool insert_in_octree = true);

  /**
   * Recreates the octrees. Useful if the inspection space should be adapted to the current data.
   */
  void recreateOctrees();
  void reinitializeTSDF(double voxel_length, double sdf_trunc);

  void save(const std::string & filepath) const;

  void clear();

  void finish();

  /**
   * Extracts the pose from an 4D extrisic matrix
   * @param extrinsic_matrix extrinsic matrix
   * @return pose of with orientation as euler angles
   */
  std::array<double, 6> transformMatrixToPose(const Eigen::Matrix4d & extrinsic_matrix);
  std::array<double, 6> quatToEulerPose(const std::array<double, 7> quat_pose) const;
  std::array<double, 7> eulerToQuatPose(const std::array<double, 6> euler_pose) const;

  void integrateImage(
    const open3d::geometry::RGBDImage & image, const int sensor_id,
    const Eigen::Matrix4d & extrinsic_optical, const Eigen::Matrix4d & extrinsic_world);
  std::shared_ptr<open3d::geometry::TriangleMesh> extractDenseReconstruction() const;

  inline uint64_t getSparseDataCount() const {return sparse_data_count_;}
  inline const std::vector<std::array<double, 3>> & getSparsePosition() const
  {
    return sparse_position_;
  }
  inline const std::vector<std::array<double, 4>> & getSparseOrientation() const
  {
    return sparse_orientation_;
  }
  inline const bool & getSparseUsage() const {return sparse_usage_;}
  inline const std::vector<std::vector<double>> & getSparseValue() const {return sparse_value_;}
  inline const std::vector<Eigen::Vector3d> & getSparseUserColor() const
  {
    return sparse_user_color_;
  }
  inline const std::vector<double> & getSparseTimestamp() const {return sparse_timestamp_;}
  inline const std::vector<double> & getSparseMin() const
  {
    if (!same_min_max_colors_) {
      return sparse_color_min_values_;
    } else {
      return sparse_min_values_;
    }
  }
  inline const std::vector<double> & getSparseMax() const
  {
    if (!same_min_max_colors_) {
      return sparse_color_max_values_;
    } else {
      return sparse_max_values_;
    }
  }

  void setIntrinsic(open3d::camera::PinholeCameraIntrinsic intrinsic, int sensor_id)
  {
    intrinsic_[sensor_id] = intrinsic;
    intrinsic_recieved_[sensor_id] = true;
  }

  std::vector<std::string> getSparseUnits() {return sparse_units_;}

  uint64_t getIntegratedImagesCount() {return integrated_frames_;}

private:
  /**
   * Internal method to generate the base data for saving the inspection.
   * @return string with the base data
   */
  std::string baseDataForSave() const;

  /**
   * Saves the inspection to a journal file
   * @param filepath path to the file to be saved
   */
  std::string saveStringForSparseEntry(int i) const;

  /**
   * Serializes a filled protobuf object.
   * @param i
   * @param color_img color images to be saved
   * @param depth_img depth images to be saved
   */
  std::string serializedStructForDenseEntry(
    int i, std::vector<u_int8_t> color_img,
    std::vector<u_int8_t> depth_img) const;

  /**
   * Methods for online saving of data by appending new measurements to the end
   * of the journal file
   */
  void appendSave(const std::string & filepath);

  bool finished_;
  uint64_t sparse_data_count_, dense_data_count_;
  std::vector<SensorType> sensor_types_;
  std::vector<std::string> sparse_units_;
  bool sparse_usage_, dense_usage_, robot_usage_;
  SensorType dense_type_;
  std::vector<std::string> sparse_types_;
  std::vector<std::string> joint_names_;
  open3d::geometry::TriangleMesh reference_mesh_;
  std::tuple<int, int> dense_sensor_resolution_;
  std::string save_path_;
  OrthoTree::OctreePointC sparse_octree_;
  OrthoTree::TreePointPoseND<6, double> dense_posetree_;
  std::map<std::string, int> sparse_data_type_to_id_;
  std::vector<double> sparse_timestamp_, dense_timestamp_;
  std::vector<int> sparse_sensor_id_, dense_sensor_id_;
  std::vector<std::array<double, 3>> sparse_position_;
  std::vector<std::array<double, 6>> dense_pose_;
  std::vector<std::array<double, 4>> sparse_orientation_, dense_orientation_;
  std::vector<std::vector<double>> sparse_value_;
  std::vector<Eigen::Vector3d> sparse_user_color_;
  std::deque<std::vector<u_int8_t>> dense_image_, dense_depth_image_;
  std::vector<double> robot_timestamp_;
  std::vector<std::vector<double>> robot_states_;
  OrthoTree::BoundingBox3D inspection_space_3d_;
  OrthoTree::BoundingBoxND<6> inspection_space_6d_;
  bool fixed_min_max_values_;
  std::vector<double> sparse_min_values_;
  std::vector<double> sparse_max_values_;
  bool same_min_max_colors_;
  std::vector<double> sparse_color_min_values_;
  std::vector<double> sparse_color_max_values_;
  std::thread * saving_thread_;
  rocksdb::DB * db_;
  rocksdb::Options db_options_;
  rocksdb::Status db_status_;
  std::mutex * mtx_;
  // Note: mutex needed because the asynchrones append and popback can lead to unpredictable double-free or corrupted size.
  // As a pointer because default mutex can not be copied with the default copy-constructor we use.

  std::shared_ptr<open3d::pipelines::integration::ScalableTSDFVolume> tsdf_volume_;
  std::vector<open3d::camera::PinholeCameraIntrinsic> intrinsic_;
  std::vector<bool> intrinsic_recieved_;
  uint64_t integrated_frames_;
  open3d::geometry::AxisAlignedBoundingBox crop_box_;
};
/**
 * Loads the inspection from a journal file
 * @param filepath path to the file to be loaded
 * @return the loaded inspection
 */
Inspection load(const std::string filename);
}  // namespace vinspect
#endif  // VINSPECT__INSPECTION_H_
