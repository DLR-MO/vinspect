// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/inspection.hpp"
namespace vinspect
{

// This constructor loads an already existing vinspect project file
Inspection::Inspection(const std::string & file_path) {
  if (!std::filesystem::exists(file_path)) {
    throw std::runtime_error("File does not exist: " + file_path);
  }
  std::cout << "Loading data from file" << std::endl;

  // Initialize the database
  initDB(file_path);
  
  // Load static metadata
  std::string serialized_meta_data;
  if(!db_->Get(rocksdb::ReadOptions(), DB_KEY_STATIC_METADATA, &serialized_meta_data).ok()) {
    throw std::runtime_error("Malformed project file");
  }
  json static_metadata = json::parse(serialized_meta_data);
  
  std::cout << "--------- Metadata ----------" << std::endl;

  std::cout << serialized_meta_data << std::endl;

  std::cout << "-----------------------------" << std::endl;
  
  inspection_space_3d_ = {
    .Min = static_metadata["3D Inspection space"]["min"], 
    .Max = static_metadata["3D Inspection space"]["max"]
  };
  inspection_space_6d_ = {
    .Min = static_metadata["6D Inspection space"]["min"], 
    .Max = static_metadata["6D Inspection space"]["max"]
  };

  // Deserialize sensors
  sparse_value_infos_ = static_metadata["Sensors"]["Sparse"]["Value infos"];
  dense_sensors_ = static_metadata["Sensors"]["Dense"];
  
  // Make common initializations for both dense and sparse sensors
  setupSensors();
  
  recreateOctrees();
  
  loadMesh(DB_KEY_REFERENCE_MESH, reference_mesh_);
  
  std::cout << "Processing measurements..." << std::endl;
  
  // Iterate over sparse measurements
  if (getSparseUsage())
  {
    // Create an iterator
    auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));
  
    // Iterate over keys with the specified prefix using a for loop
    for (it->Seek(DB_KEY_SPARSE_DATA_PREFIX); it->Valid() && it->key().starts_with(DB_KEY_SPARSE_DATA_PREFIX); it->Next()) {
      json sample = json::parse(it->value().ToStringView());

      Eigen::Vector3d user_color(
        sample["user_color"]["r"], 
        sample["user_color"]["g"], 
        sample["user_color"]["b"]
      );

      addSparseMeasurementImpl(
        sample["timestamp"],
        sample["sensor_id"],
        sample["position"],
        sample["orientation"],
        sample["value"],
        user_color,
        false
      );
    }
  }
  // Iterate over dense measurements
  if(getDenseUsage())
  {
    // Create an iterator
    auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));
  
    // Iterate over keys with the specified prefix using a for loop
    for (it->Seek(DB_KEY_DENSE_DATA_PREFIX); it->Valid() && it->key().starts_with(DB_KEY_DENSE_DATA_PREFIX); it->Next()) {
      auto sample = it->value().ToString();

      Dense retrievedEntry;
      if (!retrievedEntry.ParseFromString(sample)) {
        throw std::runtime_error("Failed to parse dense data");
      }

      // Get sensor information
      auto sensor = getDenseSensor(retrievedEntry.sensor_id());

      // Convert image to open3d
      open3d::geometry::Image o3d_color_img, o3d_depth_img;
      o3d_depth_img.Prepare(sensor.getWidth(), sensor.getHeight(), 1, 2);
      o3d_color_img.Prepare(sensor.getWidth(), sensor.getHeight(), 3, 1);
      memcpy(o3d_depth_img.data_.data(), retrievedEntry.depth_image().data(), o3d_depth_img.data_.size());
      memcpy(o3d_color_img.data_.data(), retrievedEntry.color_image().data(), o3d_color_img.data_.size());

      std::shared_ptr<open3d::geometry::RGBDImage> rgbd =
        open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        o3d_color_img, o3d_depth_img, 1.0f, 99999999.0f, false);

      addImageImpl(
        *rgbd, 
        retrievedEntry.sensor_id(), 
        matrixFromFlatProtoArray(retrievedEntry.extrinsic_optical_matrix()), 
        matrixFromFlatProtoArray(retrievedEntry.extrinsic_optical_matrix()),
        false
      );
    }
  }
  
  std::cout << "Data loaded." << std::endl;
  
}

// todo maybe rename sparse->point and dense->area?

// todo pass aggregation models to vinspect

// This constructor creates a new vinspect project with a new file
Inspection::Inspection(
  std::vector<SparseValueInfo> sparse_value_infos,
  std::vector<DenseSensor> dense_sensors,
  open3d::geometry::TriangleMesh reference_mesh,
  std::string save_path, 
  std::array<double, 3> inspection_space_3d_min, 
  std::array<double, 3> inspection_space_3d_max, 
  std::array<double, 6> inspection_space_6d_min, 
  std::array<double, 6> inspection_space_6d_max) 
  : 
  sparse_value_infos_{sparse_value_infos}, 
  dense_sensors_{dense_sensors},
  reference_mesh_{reference_mesh},
  inspection_space_3d_{.Min = inspection_space_3d_min, .Max = inspection_space_3d_max},
  inspection_space_6d_{.Min = inspection_space_6d_min, .Max = inspection_space_6d_max}
{
  // Create new database
  initDB(save_path);

  // Make common initializations for both dense and sparse sensors
  setupSensors();

  // Apply workspace boundaries
  recreateOctrees();
 
  // Save static metadata that is unlikely to change in the DB
  if (!saveMetaData())
  {
    throw std::runtime_error("Something went wrong while accessing the DB during metadata save.");
  }

  // Save the reference mesh in the DB
  storeMesh(DB_KEY_REFERENCE_MESH, reference_mesh_);
}
  
Inspection::Inspection(
  std::vector<SparseValueInfo> sparse_value_infos,
  std::vector<DenseSensor> dense_sensors,
  std::string mesh_file_path, 
  std::string save_path,
  std::array<double, 3> inspection_space_3d_min, 
  std::array<double, 3> inspection_space_3d_max,
  std::array<double, 6> inspection_space_6d_min, 
  std::array<double, 6> inspection_space_6d_max)
: Inspection(sparse_value_infos, dense_sensors, meshFromPath(mesh_file_path), save_path, inspection_space_3d_min,
    inspection_space_3d_max, inspection_space_6d_min, inspection_space_6d_max)
{ }

void Inspection::initDB(const std::string &file_path)
{
  db_options_.create_if_missing = true;
  rocksdb::DB* db;
  rocksdb::Status s = rocksdb::DB::Open(db_options_, file_path, &db);
  db_ = std::unique_ptr<rocksdb::DB>(db);
  if(!s.ok()) {
    throw std::runtime_error("Could not open " + file_path +
      ". Maybe the file exists and has wrong permissions.");
  }
}

void Inspection::setupSensors()
{
  if (getSparseUsage()) {
    // Find the min/max values for each sparse value type
    sparse_min_values_ =
      std::vector<double>(sparse_value_infos_.size(), std::numeric_limits<double>::max());
    sparse_max_values_ =
      std::vector<double>(sparse_value_infos_.size(), std::numeric_limits<double>::lowest());
  }

  if (getDenseUsage()) {
    crop_box_ = open3d::geometry::AxisAlignedBoundingBox(cast(inspection_space_3d_.Min), cast(inspection_space_3d_.Max));
    // todo we might also want to have multiple TSDF volumes for different dense sensor types,
    // but also use multiple dense sensors of the same type for the same tsdf
    // todo don't hardcode the parameters of the tsdf volume
    tsdf_volume_ = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
      4.0 / 512.0, 0.04, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
  }
}

std::string Inspection::toString() const
{
  return "Inspection: " + std::to_string(sparse_data_count_) + " sparse and " +
         std::to_string(dense_data_count_) + " dense measurements";
}

open3d::geometry::TriangleMesh Inspection::getMesh() const {return reference_mesh_;}

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
      {
        ids_in_radius.push_back(id);
      }
    }
    return ids_in_radius;
  }
}

std::vector<double> Inspection::getSparseValuesAtPosition(
  const std::string & value_name, 
  const std::array<double, 3> & position, 
  double radius) const
{
  std::vector<uint64_t> ids = getSparseMeasurementsInRadius(position, radius);
  return getSparseValuesForIds(value_name, ids);
}

std::vector<double> Inspection::getSparseValuesForIds(
  const std::string & value_name, 
  const std::vector<uint64_t> & ids) const
{
  // Find the value info that matches the name
  auto it = std::find_if(
    sparse_value_infos_.begin(), 
    sparse_value_infos_.begin(), 
    [&value_name](const auto & info){
      return info.name == value_name;
    });
  
  if (it == sparse_value_infos_.end()) {
    throw std::runtime_error(fmt::format("Value info with name {} not found in sensor value infos", value_name));
  }

  std::size_t value_index = std::distance(sparse_value_infos_.begin(), it);

  std::vector<double> values(ids.size());
  for (uint64_t i = 0; i < ids.size(); i++) {
    values[i] = sparse_value_[ids[i]][value_index];
  }
  return values;
}

std::array<double, 6> Inspection::getDensePoseFromId(const int sample_id) const
{
  std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", sample_id, DB_NUMBER_DIGITS_FOR_IDX);
  std::string retrievedStringData;
  db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);

  Dense retrievedEntry;
  if (!retrievedEntry.ParseFromString(retrievedStringData)) {
    throw std::runtime_error("Error parsing dense sample");
  }
  
  assert(retrievedEntry.entry_nr() == sample_id);
  
  return transformMatrixToPose(matrixFromFlatProtoArray(retrievedEntry.extrinsic_world_matrix()));
}

std::vector<std::array<double, 6>> Inspection::getMultiDensePoses(const int percentage) const
{
  std::vector<std::array<double, 6>> all_poses;
  float float_percentage = percentage / 100.0;
  float x = dense_data_count_ * float_percentage;
  float entries_to_skip = dense_data_count_ / x;

  for (size_t i = 0; i < dense_data_count_; i = i + entries_to_skip) {
    std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", i, DB_NUMBER_DIGITS_FOR_IDX);
    std::string retrievedStringData;
    db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);

    Dense retrievedEntry;
    if (!retrievedEntry.ParseFromString(retrievedStringData)) {
      throw std::runtime_error("Error parsing dense sample");
    }

    all_poses.push_back(transformMatrixToPose(matrixFromFlatProtoArray(retrievedEntry.extrinsic_world_matrix())));
  }

  return all_poses;
}

std::vector<std::vector<std::array<u_int8_t, 3>>> Inspection::getImageFromId(const int sample_id) const
{
  std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", sample_id, DB_NUMBER_DIGITS_FOR_IDX);
  std::string retrievedStringData;
  std::vector<std::vector<std::array<u_int8_t, 3>>> image;
  db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);
  Dense retrievedEntry;
  if (retrievedEntry.ParseFromString(retrievedStringData)) {
    // Get sensor infos
    auto sensor = getDenseSensor(retrievedEntry.sensor_id());

    // Allocate memory
    image.resize(
      sensor.getHeight(),
      std::vector<std::array<uint8_t, 3>>(sensor.getWidth()));

    // Copy data
    int index = 0;
    for (std::size_t row = 0; row < sensor.getHeight(); row++) {
      for (std::size_t col = 0; col < sensor.getWidth(); col++) {
        image[row][col] =
        {
          static_cast<u_int8_t>(retrievedEntry.color_image(index)), 
          static_cast<u_int8_t>(retrievedEntry.color_image(index + 1)),
          static_cast<u_int8_t>(retrievedEntry.color_image(index + 2))
        };
        index += 3;
      }
    }
  }
  return image;
}

int Inspection::getClosestDenseMeasurement(const std::array<double, 6> & pose) const
{  // todo should return unsigned long
  if (dense_pose_.size() == 0) {
    return -1;
  } else {
    return dense_posetree_.GetNearestNeighbors(pose, 1, dense_pose_)[0];
  }
}

int Inspection::getClosestDenseMeasurement(const std::array<double, 7> & quat_pose) const
{  // todo should return unsigned long
  if (dense_pose_.size() == 0) {
    return -1;
  } else {
    std::array<double, 6> euler_pose = quatToEulerPose(quat_pose);
    auto res = dense_posetree_.GetNearestNeighbors(euler_pose, 1, dense_pose_);
    return res[0];
  }
}

std::vector<double> Inspection::getDenseAtPose(
  [[maybe_unused]] const std::array<double, 6> & pose, [[maybe_unused]] double radius) const
{
  // make sure there is no code duplication with the sparse version, when
  // implementing this
  throw std::runtime_error("Not implemented yet.");
}

// todo add a option that only a certain number of measurements are added at a specific
// location. This way the data does
// not get out of hand if the sensor rests at the same spot for a long time. can be implemented by
// using the range function of the octree
void Inspection::addSparseMeasurementImpl(
  double timestamp, int sensor_id, const std::array<double, 3> & position,
  const std::array<double, 4> & orientation, const std::vector<double> & values,
  const Eigen::Vector3d & user_color, bool insert_in_octree, bool store_in_database)
{
  if (!isPointInSpace(&inspection_space_3d_, position)) {
    std::cout << "Ignoring measurement at (" << position[0] << ", " << position[1] << ","
    << position[2] << ") because it is outside inspection space." << std::endl;
  } else {

    // Acquire the mutex
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Estimate min and max value for each data type reported by the sparse sensor
    for (uint64_t i = 0; i < sparse_value_infos_.size(); i++) {
      sparse_min_values_[i] = std::min(sparse_min_values_[i], values[i]);
      sparse_max_values_[i] = std::max(sparse_max_values_[i], values[i]);
    }
    
    sparse_orientation_.push_back(orientation);
    sparse_value_.push_back(values);
    sparse_user_color_.push_back(user_color);
    sparse_timestamp_.push_back(timestamp);
    sparse_position_.push_back(position);
    
    // Adding all values to leafs of the octree. Was faster in experiments
    // todo check if this is also the case when running the software online
    if (insert_in_octree) {
      sparse_octree_.Add(sparse_position_[sparse_data_count_], true);
    }

    if (store_in_database)
    {

      // Serialize data point
      json j = {
        {"timestamp", timestamp},
        {"position", position},
        {"orientation", orientation},
        {"value", values},
        {"user_color", 
          {
            {"r", user_color[0]},
            {"g", user_color[1]},
            {"b", user_color[2]}
          }},
        {"sensor_id", sensor_id}
      };
      std::string serialized_datapoint = j.dump();
  
      // Construct a key in the format "prefix/{sensor_id}/{meassurement_id}"
      std::string key = DB_KEY_SPARSE_DATA_PREFIX + fmt::format("{:0>{}}", sparse_data_count_, DB_NUMBER_DIGITS_FOR_IDX);
  
      // Store in DB
      if (!db_->Put(rocksdb::WriteOptions(), key, serialized_datapoint).ok())
      {
        std::cerr << "Failed to save sparse entry in DB";
      }
    }

    sparse_data_count_++;
  }
}

void Inspection::addImageImpl(
  const open3d::geometry::RGBDImage & image, const int sensor_id,
  const Eigen::Matrix4d & extrinsic_optical, const Eigen::Matrix4d & extrinsic_world, bool store_in_database)
{
  // we can only integrate if we already received the intrinsic calibration for this sensor
  auto intrinsics = intrinsic_.find(sensor_id);
  if (intrinsics == intrinsic_.end()) {
    std::cout << "No intrinsic calibration available for sensor " << sensor_id
              << " Will not integrate." << std::endl;
    return;
  }

  tsdf_volume_->Integrate(image, intrinsics->second, extrinsic_optical);
  std::array<double, 6> image_pose = transformMatrixToPose(extrinsic_world);

  {
    std::lock_guard<std::mutex> mtx_lock(mtx_);
    dense_posetree_.Insert(dense_data_count_, image_pose, false);
    dense_pose_.push_back(image_pose);
    
    // save dense data to database
    if (store_in_database) {
      std::string key = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", sparse_data_count_, DB_NUMBER_DIGITS_FOR_IDX);
      std::string value = serializedStructForDenseEntry(dense_data_count_, sensor_id, image.color_.data_, image.depth_.data_, extrinsic_optical, extrinsic_world);
      db_->Put(rocksdb::WriteOptions(), key, value);
    }

    dense_data_count_++;
  }
}

std::shared_ptr<open3d::geometry::TriangleMesh> Inspection::extractDenseReconstruction() const
{
  // todo beware of copying the returned mesh
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = tsdf_volume_->ExtractTriangleMesh();
  // todo maybe give some warning if the whole mesh is cropped to 0 triangles
  std::shared_ptr<open3d::geometry::TriangleMesh> cropped_mesh = mesh->Crop(crop_box_);
  return cropped_mesh;
}

void Inspection::saveDenseReconstruction(std::string filename) const
{
  auto mesh = extractDenseReconstruction();
  open3d::io::WriteTriangleMesh(filename, *mesh, true);
}

void Inspection::recreateOctrees()
{
  if (getSparseUsage()) {
    sparse_octree_ = OrthoTree::OctreePointC(sparse_position_, OCTREE_DEPTH, inspection_space_3d_);
  }
  if (getDenseUsage()) {
    dense_posetree_ = OrthoTree::TreePointPoseND<6, {0, 0, 0, 1, 1, 1}, std::ratio<1, 2>, double>();
    dense_posetree_.Create(
      dense_posetree_, dense_pose_, OCTREE_DEPTH, inspection_space_6d_,
      MAX_POSES_IN_LEAF);
  }
}

void Inspection::reinitializeTSDF(double voxel_length, double sdf_trunc)
{
  tsdf_volume_ = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
    voxel_length, sdf_trunc, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
}

void Inspection::clear()
{
  // delete dynamic database keys
  db_->DeleteRange(rocksdb::WriteOptions(), DB_KEY_SPARSE_DATA_PREFIX, DB_KEY_SPARSE_DATA_PREFIX + '\xFF');
  db_->DeleteRange(rocksdb::WriteOptions(), DB_KEY_DENSE_DATA_PREFIX, DB_KEY_DENSE_DATA_PREFIX + '\xFF');

  // reset the inspection
  sparse_data_count_ = 0;
  dense_data_count_ = 0;
  sparse_octree_.Clear();
  sparse_timestamp_.clear();
  sparse_position_.clear();
  sparse_orientation_.clear();
  sparse_value_.clear();
  sparse_user_color_.clear();
  if (getSparseUsage())
  {
    auto num_sparse_types = sparse_value_infos_.size();
    sparse_min_values_ = std::vector<double>(num_sparse_types, std::numeric_limits<double>::max());
    sparse_max_values_ = std::vector<double>(num_sparse_types, std::numeric_limits<double>::lowest());
  }

  dense_timestamp_.clear();
  dense_pose_.clear();
  dense_orientation_.clear();

  recreateOctrees();
}

// todo could also add a field to store an arbitrary string as a note or comment

bool Inspection::saveMetaData()
{
  json j = {
    {"header",  {
      // Store the current unix timestamp
      {"Creation time", std::chrono::system_clock::now().time_since_epoch().count()},
      // Git hash of the version that created the file
      {"Git hash", GIT_COMMIT_HASH }
    }},
    {"3D Inspection space", {
      { "min", inspection_space_3d_.Min},
      { "max", inspection_space_3d_.Max}
    }},
    {"6D Inspection space", {
      { "min", inspection_space_6d_.Min},
      { "max", inspection_space_6d_.Max}
    }},
    {"Sensors", {
       {"Sparse", {
          {"Value infos", sparse_value_infos_}
       }},
       {"Dense", dense_sensors_}
      }
    }
  };
  
  // Serialize the metadata into a JSON string
  std::string metadata = j.dump(2);

  // Store in the database
  return db_->Put(rocksdb::WriteOptions(), DB_KEY_STATIC_METADATA, metadata).ok();
}

void Inspection::storeMesh(const std::string &key, const open3d::geometry::TriangleMesh &mesh)
{
  // Check if the mesh has any vertices
  if (mesh.vertices_.size() == 0)
  {
    return;
  }

  // Open3d only supports export to files, not byte streams, we we need to use a temp file
  std::string temp_file = fmt::format("/tmp/vinspect_mesh_{}.ply", std::rand());
  open3d::io::WriteTriangleMesh(temp_file, mesh, true);
  std::ifstream file(temp_file);
  
  // Check if file can be opened
  if (!file.is_open())
  {
    throw std::runtime_error(fmt::format("Error opening temporary file: '{}'", temp_file));
  }

  // Read the entire content of the file into a string stream
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();

  // Close and remove the file
  file.close();
  std::remove(temp_file.c_str());

  // Save in database
  if(!db_->Put(rocksdb::WriteOptions(), key, content).ok())
  {
    throw std::runtime_error("Could not store mesh in db");
  }
}

void Inspection::loadMesh(const std::string &key, open3d::geometry::TriangleMesh &mesh)
{
  // Get mesh from db 
  std::string serialized_mesh;

  auto response = db_->Get(rocksdb::ReadOptions(), key, &serialized_mesh);
  if (response.IsNotFound())
  {
    // No mesh present, initialize with empty mesh instead
    mesh = open3d::geometry::TriangleMesh();
    return;
  } else if (!response.ok())
  {
    throw std::runtime_error("Could load mesh from db");
  }

  // Open3d only supports export to files, not byte streams, we we need to use a temp file
  std::string temp_file = fmt::format("/tmp/vinspect_mesh_{}.ply", std::rand());

  std::ofstream file(temp_file);
  // Check if file can be opened
  if (!file.is_open())
  {
    throw std::runtime_error(fmt::format("Error opening temporary file: '{}'", temp_file));
  }

  // Write mesh to file
  file << serialized_mesh;
  file.close();

  // Load mesh into open3d
  if (!open3d::io::ReadTriangleMesh(temp_file, mesh))
  {
    throw std::runtime_error("Could not load mesh from file");
  }
  
  // Close and remove the file
  std::remove(temp_file.c_str());
}
}  // namespace vinspect
