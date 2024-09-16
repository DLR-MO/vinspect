// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/inspection.hpp"
namespace vinspect
{

Inspection::Inspection() {}

Inspection::Inspection(
  std::vector<SensorType> sensor_types, std::vector<std::string> sparse_types,
  std::vector<std::string> sparse_units, std::vector<std::string> joint_names,
  open3d::geometry::TriangleMesh mesh, std::tuple<int, int> dense_sensor_resolution,
  std::string save_path, std::array<double, 3> inspection_space_min,
  std::array<double, 3> inspection_space_max, std::vector<double> sparse_color_min_values,
  std::vector<double> sparse_color_max_values)
{
  save_path_ = save_path;
  inspection_space_.Min = inspection_space_min;
  inspection_space_.Max = inspection_space_max;
  // todo maybe rename sparse->point and dense->area?
  //  Check if what type of sensor is used
  sensor_types_ = sensor_types;
  sparse_units_ = sparse_units;
  sparse_usage_ = false;
  dense_usage_ = false;
  dense_type_ = SensorType::RGB;
  for (auto sensor_type : sensor_types_) {
    if (sensor_type == SensorType::SPARSE) {
      sparse_usage_ = true;
    } else {
      dense_usage_ = true;
      // todo this does not really make sense. what about the other types
      if (dense_type_ == SensorType::RGB) {
        dense_type_ = sensor_type;
      } else if (dense_type_ != sensor_type) {
        throw std::runtime_error("It is not possible to use two different dense sensor types.");
      }
    }
  }

  if (sparse_usage_) {
    sparse_data_count_ = 0;
    if (sparse_types.empty()) {
      throw std::runtime_error(
        "sparse_types must be provided if a sparse sensor is "
        "used.");
    }
    // List of data type names, e.g. distance, angle, depth
    sparse_types_ = sparse_types;
    std::array<std::array<double, 3>, 0> constexpr points;
    // todo would be better to reuse the data by using a not container octree
    sparse_octree_ = OrthoTree::OctreePointC(points, OCTREE_DEPTH, inspection_space_);
    sparse_data_type_to_id_ = std::map<std::string, int>();
    for (uint64_t i = 0; i < sparse_types_.size(); i++) {
      sparse_data_type_to_id_[sparse_types_[i]] = i;
    }

    fixed_min_max_values_ = false;
    sparse_min_values_ =
      std::vector<double>(sparse_types_.size(), std::numeric_limits<double>::max());
    sparse_max_values_ =
      std::vector<double>(sparse_types_.size(), std::numeric_limits<double>::lowest());

    if (sparse_color_min_values.size() == 0 && sparse_color_max_values.size() == 0) {
      same_min_max_colors_ = true;
    } else if (
      sparse_color_min_values.size() != sparse_types_.size() ||
      sparse_color_max_values.size() != sparse_types_.size()) {
      throw std::runtime_error(
        "sparse_min_color_values and sparse_max_color_values must be the same size as "
        "sparse_types");
    } else {
      same_min_max_colors_ = false;
      sparse_color_min_values_ = sparse_color_min_values;
      sparse_color_max_values_ = sparse_color_max_values;
    }
  }

  integrated_frames_ = 0;
  if (dense_usage_) {
    dense_sensor_resolution_ = dense_sensor_resolution;
    // TODO check if we really need to save the dense_sensor_resolution
    // todo the camera infos should be provided during construction
    // todo should use dense_sensor_resolution_.size());
    // todo this sensor_types.size() - sparse_types.size() is not so nice
    intrinsic_ = std::vector<open3d::camera::PinholeCameraIntrinsic>(sensor_types.size() - sparse_types.size());
    intrinsic_recieved_ =
      std::vector<bool>(1);  // todo are these automatically initialized to false?
    crop_box_ = open3d::geometry::AxisAlignedBoundingBox(
      Eigen::Vector3d(inspection_space_min[0], inspection_space_min[1], inspection_space_min[2]),
      Eigen::Vector3d(inspection_space_max[0], inspection_space_max[1], inspection_space_max[2]));
    // todo we might also want to have multiple TSDF volumes for different dense sensor types,
    // but also use multiple dense sensors of the same type for the same tsdf
    // todo don't hardcode the parameters of the tsdf volume
    tsdf_volume_ = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
      4.0 / 512.0, 0.04, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
    // todo this octree would also need to store orientation
    std::array<std::array<double, 3>, 0> constexpr points;
    dense_octree_ = OrthoTree::OctreePointC(points, OCTREE_DEPTH, inspection_space_);
    dense_data_count_ = 0;
  }

  reference_mesh_ = mesh;
  // Save joint angles coninously to playback robot trajectory later
  // They can also be matched by the timestamp to a sensor reading
  robot_usage_ = joint_names_.size() > 0;
  if (robot_usage_) {
    joint_names_ = joint_names;
    robot_timestamp_ = std::vector<double>(10000);
    // robot_states_ = std::vector<<std::vector<double>>(10000);
  }

  finished_ = false;
}

Inspection::Inspection(
  std::vector<std::string> sensor_types_names, std::vector<std::string> sparse_types,
  std::vector<std::string> sparse_units, std::vector<std::string> joint_names,
  std::string mesh_file_path, std::tuple<int, int> dense_sensor_resolution, std::string save_path,
  std::array<double, 3> inspection_space_min, std::array<double, 3> inspection_space_max,
  std::vector<double> sparse_color_min_values, std::vector<double> sparse_color_max_values)
: Inspection(
    stringsToTypes(sensor_types_names), sparse_types, sparse_units, joint_names,
    meshFromPath(mesh_file_path), dense_sensor_resolution, save_path, inspection_space_min,
    inspection_space_max, sparse_color_min_values, sparse_color_max_values)
{
}

std::string Inspection::toString() const
{
  return "Inspection: " + std::to_string(sparse_data_count_) + " sparse and " +
         std::to_string(dense_data_count_) + " dense measurements";
}

void Inspection::startSaving()
{
  // we can not start a thread with a member function in the constructor
  // since the object is not yet initialized
  if (save_path_ != "") {
    std::string base_data_string = baseDataForSave();
    // if a file is already existing, we default to appending to it
    if (std::filesystem::exists(save_path_)) {
      // todo check if header is the same to ensure that we do not mix different
      // kinds of inspection data
    } else {
      std::ofstream file(save_path_, std::ios::out | std::ios::app);
      file << base_data_string;
      file.close();
    }
    saving_thread_ = new std::thread([this] { appendSave(save_path_); });
  }
}

open3d::geometry::TriangleMesh Inspection::getMesh() const { return reference_mesh_; }

int Inspection::getClosestSparseMeasurement(const std::array<double, 3> & position) const
{  // todo should return unsigned long
  if (sparse_position_.size() == 0) {
    return -1;
  } else {
    return sparse_octree_.GetNearestNeighbors(position, 1)[0];
  }
}

std::vector<uint64_t> Inspection::getSparseMeasurementsInRadius(
  const std::array<double, 3> & position, double radius) const
{
  if (sparse_position_.size() == 0) {
    return std::vector<uint64_t>();
  } else {
    auto const search_box_cpp = OrthoTree::BoundingBox3D{
      {position[0] - radius, position[1] - radius, position[2] - radius},
      {position[0] + radius, position[1] + radius, position[2] + radius}};
    std::vector<uint64_t> ids_in_box = sparse_octree_.RangeSearch(search_box_cpp);
    std::vector<uint64_t> ids_in_radius = {};
    for (uint64_t i = 0; i < ids_in_box.size(); i++) {
      int id = ids_in_box[i];
      if (
        euclideanDistance(
          position[0], position[1], position[2], sparse_position_[id][0], sparse_position_[id][1],
          sparse_position_[id][2]) < radius)
        ids_in_radius.push_back(id);
    }
    return ids_in_radius;
  }
}

std::vector<double> Inspection::getSparseValuesAtPosition(
  const std::string & value_type, const std::array<double, 3> & position, double radius) const
{
  std::vector<uint64_t> ids = getSparseMeasurementsInRadius(position, radius);
  return getValuesForIds(value_type, ids);
}

std::vector<double> Inspection::getValuesForIds(
  const std::string & value_type, const std::vector<uint64_t> & ids) const
{
  int value_index = sparse_data_type_to_id_.at(value_type);
  std::vector<double> values(ids.size());
  for (uint64_t i = 0; i < ids.size(); i++) {
    values[i] = sparse_value_[ids[i]][value_index];
  }
  return values;
}

std::vector<double> Inspection::getDenseAtPosition(
  const std::array<double, 3> & position, double radius) const
{
  // make sure there is no code duplication with the sparse version, when
  // implementing this
  throw std::runtime_error("Not implemented yet.");
}

// todo add a option that only a certain number of measurments are added at a specific
// location. This way the data does
// not get out of hand if the sensor rests at the same spot for a long time. can be implemented by
// using the range function of the octree
void Inspection::addSparseMeasurement(
  double timestamp, int sensor_id, const std::array<double, 3> & position,
  const std::array<double, 4> & orientation, const std::vector<double> & values,
  const Eigen::Vector3d & user_color, bool insert_in_octree)
{
  if (!isPointInSpace(&inspection_space_, position)) {
    std::cout << "Ignoring measurement at (" << position[0] << ", " << position[1] << ","
              << position[2] << ") because it is outside inspection space." << std::endl;
  } else {
    for (uint64_t i = 0; i < sparse_types_.size(); i++) {
      // update the min and max values
      if (values[i] < sparse_min_values_[i]) {
        sparse_min_values_[i] = values[i];
      }
      if (values[i] > sparse_max_values_[i]) {
        sparse_max_values_[i] = values[i];
      }
    }

    sparse_orientation_.push_back(orientation);
    sparse_value_.push_back(values);
    sparse_user_color_.push_back(user_color);
    sparse_timestamp_.push_back(timestamp);
    sparse_sensor_id_.push_back(sensor_id);
    sparse_position_.push_back(position);

    // Adding all values to leafs of the octree. Was faster in experiments
    // todo check if this is also the case when running the software online
    if (insert_in_octree) {
      sparse_octree_.Add(sparse_position_[sparse_data_count_], true);
    }
    sparse_data_count_++;
  }
}

void Inspection::integrateImage(
  const open3d::geometry::RGBDImage & image, const int sensor_id, const Eigen::Matrix4d & extrinsic)
{
  // we can only integrate if we already recieved the intrinsic calibration for this sensor
  if (!intrinsic_recieved_[sensor_id]) {
    std::cout << "No intrinsic calibration available for sensor " << sensor_id
              << " Will not integrate." << std::endl;
    return;
  }
  tsdf_volume_->Integrate(image, intrinsic_[sensor_id], extrinsic);
  integrated_frames_++;
  // todo we should append the images and depth images to the dense octree and save them
  // self.depth_images.append(data[0]);
  // self.color_images.append(data[1]);
  // self.rgb_poses.append(rgb_pose);
}

std::shared_ptr<open3d::geometry::TriangleMesh> Inspection::extractDenseReconstruction() const
{
  // todo beware of copying the returned mesh
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = tsdf_volume_->ExtractTriangleMesh();
  // todo maybe give some warning if the whole mesh is cropped to 0 triangles
  std::shared_ptr<open3d::geometry::TriangleMesh> croped_mesh = mesh->Crop(crop_box_);
  return croped_mesh;
}

void Inspection::recreateOctrees()
{
  if (sparse_usage_) {
    sparse_octree_ = OrthoTree::OctreePointC(sparse_position_, OCTREE_DEPTH);
  }
  if (dense_usage_) {
    dense_octree_ = OrthoTree::OctreePointC(dense_position_, OCTREE_DEPTH);
  }
}

void Inspection::reinitializeTSDF(double voxel_length, double sdf_trunc)
{
  tsdf_volume_ = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
    voxel_length, sdf_trunc, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
}

void Inspection::clear()
{
  // end saving thread
  finished_ = true;
  // todo this could lead to an error if we did not create this thread before
  saving_thread_->join();
  finished_ = false;
  // remove the saved file
  if (std::filesystem::exists(save_path_)) {
    std::remove(save_path_.c_str());
  }
  // reset the inspection
  sparse_data_count_ = 0;
  dense_data_count_ = 0;
  if (sparse_usage_) {
    sparse_octree_.Clear();
    // need to reset the octree space
    std::array<std::array<double, 3>, 0> constexpr points;
    sparse_octree_ = OrthoTree::OctreePointC(points, OCTREE_DEPTH, inspection_space_);
    sparse_timestamp_.clear();
    sparse_sensor_id_.clear();
    sparse_position_.clear();
    sparse_orientation_.clear();
    sparse_value_.clear();
    sparse_user_color_.clear();
    sparse_min_values_ =
      std::vector<double>(sparse_types_.size(), std::numeric_limits<double>::max());
    sparse_max_values_ =
      std::vector<double>(sparse_types_.size(), std::numeric_limits<double>::lowest());
  }
  if (dense_usage_) {
    std::array<std::array<double, 3>, 0> constexpr points;
    dense_octree_ = OrthoTree::OctreePointC(points, OCTREE_DEPTH, inspection_space_);
    dense_timestamp_.clear();
    dense_sensor_id_.clear();
    dense_position_.clear();
    dense_orientation_.clear();
    dense_image_.clear();
    dense_depth_image_.clear();
  }
  if (robot_usage_) {
    robot_timestamp_.clear();
    robot_states_.clear();
  }
  // restart saving thread
  startSaving();
}

void Inspection::finish()
{
  finished_ = true;
  saving_thread_->join();
}

std::string Inspection::baseDataForSave() const
{
  // todo we could clean up this function by using a utils function that creates the string
  // representation of the vectors
  // todo we could also add a date to the header just to know when it was recorded
  // todo could also add a field to store an arbitrary string as a note or comment
  std::string what_to_write;
  what_to_write += "Inspection space min: (";
  what_to_write += std::to_string(inspection_space_.Min[0]) + ", " +
                   std::to_string(inspection_space_.Min[1]) + ", " +
                   std::to_string(inspection_space_.Min[2]);
  what_to_write += ")\n";
  what_to_write += "Inspection space max: (";
  what_to_write += std::to_string(inspection_space_.Max[0]) + ", " +
                   std::to_string(inspection_space_.Max[1]) + ", " +
                   std::to_string(inspection_space_.Max[2]);
  what_to_write += ")\n";

  what_to_write += writeArray("Sensor types", sensor_types_);

  what_to_write += writeArray("Sparse types", sparse_types_);
  what_to_write += writeArray("Sparse units", sparse_units_);
  what_to_write += writeArray("Sparse color min values", sparse_color_min_values_);
  what_to_write += writeArray("Sparse color max values", sparse_color_max_values_);

  // todo adapt this for multiple sensors
  what_to_write += "Dense sensor resolution: (";
  what_to_write += std::to_string(std::get<0>(dense_sensor_resolution_));
  what_to_write += ",";
  what_to_write += std::to_string(std::get<1>(dense_sensor_resolution_));
  what_to_write += ")\n";

  what_to_write += writeArray("Joint names", joint_names_);

  std::string temp_file = "/tmp/mesh.ply";
  open3d::io::WriteTriangleMesh(temp_file, reference_mesh_, true);
  std::ifstream file(temp_file);
  std::string mesh_ascii((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  what_to_write += "Original mesh: MESH_START\n" + mesh_ascii + "MESH_END\n";
  return what_to_write;
}

std::string Inspection::saveStringForSparseEntry(int i) const
{
  return "D T" + std::to_string(sparse_timestamp_[i]) + " S" +
         std::to_string(sparse_sensor_id_[i]) + " P" + array3ToString(sparse_position_[i]) + " O" +
         array4ToString(sparse_orientation_[i]) + " V" + vectorToString(sparse_value_[i]) + " C" +
         vectorToString(sparse_user_color_[i]) + "\n";
}

void Inspection::save(const std::string & filepath) const
{
  // todo add service call to manually save something, or maybe do it with the GUI
  std::string what_to_write = baseDataForSave();
  // write all data entries
  for (uint64_t i = 0; i < sparse_data_count_; i++) {
    if (i % 100 == 0) {
      std::cout << "Processed " << i << "/" << sparse_data_count_ << " sparse entries."
                << std::endl;
    }
    what_to_write += saveStringForSparseEntry(i);
  }
  // write all measurements
  for (uint64_t i = 0; i < dense_data_count_; i++) {
    if (i % 100 == 0) {
      std::cout << "Processed " << i << "/" << dense_data_count_ << " dense entries." << std::endl;
    }
    // todo dense data
  }
  std::ofstream file(filepath);
  file << what_to_write;
  file.flush();
  std::cout << "Inspection saved." << std::endl;
}

void Inspection::appendSave(const std::string & filepath)
{
  // we assume anything already in the inspection is already saved
  // this is important if this data was already loaded from a file
  uint64_t sparse_data_written = sparse_data_count_;
  uint64_t dense_data_written = dense_data_count_;
  std::ofstream file(filepath, std::ios_base::app);
  while (!finished_) {
    while (sparse_data_written < sparse_data_count_) {
      file << saveStringForSparseEntry(sparse_data_written);
      sparse_data_written++;
    }
    // todo dense data
    file.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

Inspection load(const std::string filepath)
{
  // todo we could clean up the code duplication in this method
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("File does not exist: " + filepath);
  }
  std::cout << "Loading data from file" << std::endl;
  std::ifstream f(filepath);
  std::string line;
  // read the header
  std::getline(f, line);
  std::vector<std::string> inspection_space_min_strings = splitStringArray(line, "(", ")");
  std::array<double, 3> inspection_space_min;
  inspection_space_min[0] = std::stod(inspection_space_min_strings[0]);
  inspection_space_min[1] = std::stod(inspection_space_min_strings[1]);
  inspection_space_min[2] = std::stod(inspection_space_min_strings[2]);
  std::getline(f, line);
  std::vector<std::string> inspection_space_max_strings = splitStringArray(line, "(", ")");
  std::array<double, 3> inspection_space_max;
  inspection_space_max[0] = std::stod(inspection_space_max_strings[0]);
  inspection_space_max[1] = std::stod(inspection_space_max_strings[1]);
  inspection_space_max[2] = std::stod(inspection_space_max_strings[2]);

  std::getline(f, line);
  std::vector<SensorType> sensor_types;
  for (std::string sensor_type : splitStringArray(line)) {
    sensor_types.push_back(stringToType(sensor_type));
  }

  std::getline(f, line);
  std::vector<std::string> sparse_types = splitStringArray(line);
  std::getline(f, line);
  std::vector<std::string> sparse_units = splitStringArray(line);
  std::getline(f, line);
  std::vector<std::string> sparse_color_min_values_strings = splitStringArray(line);
  std::vector<double> sparse_color_min_values;
  for (std::string value_string : sparse_color_min_values_strings) {
    sparse_color_min_values.push_back(std::stod(value_string));
  }
  std::getline(f, line);
  std::vector<std::string> sparse_color_max_values_strings = splitStringArray(line);
  std::vector<double> sparse_color_max_values;
  for (std::string value_string : sparse_color_max_values_strings) {
    sparse_color_max_values.push_back(std::stod(value_string));
  }

  std::getline(f, line);
  std::vector<std::string> sensor_resolution_strings = splitStringArray(line, "(", ")");
  std::tuple<int, int> sensor_resolution;
  sensor_resolution = std::make_tuple(
    std::stoi(sensor_resolution_strings[0]), std::stoi(sensor_resolution_strings[1]));

  std::getline(f, line);
  std::vector<std::string> joint_names = splitStringArray(line);

  open3d::geometry::TriangleMesh mesh = meshFromFilestream(f);

  Inspection inspection = Inspection(
    sensor_types, sparse_types, sparse_units, joint_names, mesh, sensor_resolution, filepath,
    inspection_space_min, inspection_space_max, sparse_color_min_values, sparse_color_max_values);

  std::cout << "Loaded header" << std::endl;

  std::cout << "Processing measurements." << std::endl;
  // read sparse data
  int i = 0;
  while (std::getline(f, line)) {
    line = line.substr(line.find(" ") + 1, line.size());
    double time = std::stod(line.substr(line.find("T") + 1, line.find(" ")));
    line = line.substr(line.find(" ") + 1, line.size());
    int sensor_id = std::stoi(line.substr(line.find("S") + 1, line.find(" ")));
    line = line.substr(line.find(" ") + 1, line.size());
    std::vector<std::string> position_strings =
      splitStringArray(line.substr(line.find("P") + 1, line.find(" ")), "(", ")");
    std::array<double, 3> position = {
      std::stod(position_strings[0]), std::stod(position_strings[1]),
      std::stod(position_strings[2])};
    line = line.substr(line.find(" ") + 1, line.size());
    std::vector<std::string> orientation_strings =
      splitStringArray(line.substr(line.find("O") + 1, line.find(" ")), "(", ")");
    std::array<double, 4> orientation = {
      std::stod(orientation_strings[0]), std::stod(orientation_strings[1]),
      std::stod(orientation_strings[2]), std::stod(orientation_strings[3])};
    line = line.substr(line.find(" ") + 1, line.size());
    std::vector<std::string> value_strings =
      splitStringArray(line.substr(line.find("V") + 1, line.size()), "(", ")");
    std::vector<double> values;
    for (std::string value_string : value_strings) {
      values.push_back(std::stod(value_string));
    }
    std::vector<std::string> color_strings =
      splitStringArray(line.substr(line.find("C") + 1, line.size()), "(", ")");
    Eigen::Vector3d color = Eigen::Vector3d(
      std::stod(color_strings[0]), std::stod(color_strings[1]), std::stod(color_strings[2]));
    inspection.addSparseMeasurement(time, sensor_id, position, orientation, values, color, false);
    i++;
    if (i % 1000 == 0) {
      std::cout << "\x1b[1A"
                << "\x1b[2K"
                << "Processed " << i << " measurements." << std::endl;
    }
  }
  std::cout << "\x1b[1A"
            << "\x1b[2K"
            << "Finished reading " << i << " measurments" << std::endl;

  // necessary to set the inspection space correctly to the loaded data
  inspection.recreateOctrees();

  std::cout << "Data loaded." << std::endl;
  return inspection;
}
}  // namespace vinspect
