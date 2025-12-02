// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "vinspect/inspection.hpp"
namespace vinspect
{

// This constructor loads an already existing vinspect project file
Inspection::Inspection(const std::string & file_path)
: o3d_device_{selectDevice()}
{
  if (!fs::exists(file_path)) {
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
  
  loadMesh(DB_KEY_REFERENCE_MESH, reference_mesh_);
  
  std::cout << "Processing measurements..." << std::endl;
  
  // Build application state from raw data in DB
  if (getSparseUsage()) {
    reconstructSparseFromDB();
  }
  if(getDenseUsage()) {
    reinitializeTSDF(0.01); // TODO Store the original value
  }

  // Test the export
  saveDiconde("/home/vahl_fl/colcon_ws");

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
:sparse_value_infos_{sparse_value_infos},
  dense_sensors_{dense_sensors},
  reference_mesh_{reference_mesh},
  inspection_space_3d_{.Min = inspection_space_3d_min, .Max = inspection_space_3d_max},
  inspection_space_6d_{.Min = inspection_space_6d_min, .Max = inspection_space_6d_max},
  o3d_device_{selectDevice()}
{
  // Create new database
  initDB(save_path);

  // Make common initializations for both dense and sparse sensors
  setupSensors();

  // Reset/Init inspection state
  clear();

  // Save static metadata that is unlikely to change in the DB
  if (!saveMetaData()) {
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
: Inspection(sparse_value_infos, dense_sensors, meshFromPath(mesh_file_path), save_path,
    inspection_space_3d_min,
    inspection_space_3d_max, inspection_space_6d_min, inspection_space_6d_max)
{}

void Inspection::initDB(const std::string & file_path)
{
  db_options_.create_if_missing = true;
  rocksdb::DB * db;
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
    crop_box_ = open3d::geometry::AxisAlignedBoundingBox(cast(inspection_space_3d_.Min),
        cast(inspection_space_3d_.Max));
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
    throw std::runtime_error(fmt::format("Value info with name {} not found in sensor value infos",
        value_name));
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
  std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", sample_id,
      DB_NUMBER_DIGITS_FOR_IDX);
  std::string retrievedStringData;
  db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);

  Dense retrievedEntry;
  if (!retrievedEntry.ParseFromString(retrievedStringData)) {
    throw std::runtime_error("Error parsing dense sample");
  }

  assert(retrievedEntry.entry_nr() == sample_id);

  auto pose = transformMatrixToPose(matrixFromFlatProtoArray<4, 4>(retrievedEntry.extrinsic_world_matrix()));
  return pose;
}

std::vector<std::array<double, 6>> Inspection::getMultiDensePoses(const int percentage) const
{
  std::vector<std::array<double, 6>> all_poses;
  float float_percentage = percentage / 100.0;
  float x = dense_data_count_ * float_percentage;
  float entries_to_skip = dense_data_count_ / x;

  for (size_t i = 0; i < dense_data_count_; i = i + entries_to_skip) {
    std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", i,
        DB_NUMBER_DIGITS_FOR_IDX);
    std::string retrievedStringData;
    db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);

    Dense retrievedEntry;
    if (!retrievedEntry.ParseFromString(retrievedStringData)) {
      throw std::runtime_error("Error parsing dense sample");
    }

    all_poses.push_back(transformMatrixToPose(matrixFromFlatProtoArray<4, 4>(
        retrievedEntry.extrinsic_world_matrix())));
  }

  return all_poses;
}

cv::Mat Inspection::getImageFromId(const int sample_id) const
{
  // Retrieve sample from DB
  std::string retrieveKey = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", sample_id,
      DB_NUMBER_DIGITS_FOR_IDX);
  std::string retrievedStringData;
  auto res = db_->Get(rocksdb::ReadOptions(), retrieveKey, &retrievedStringData);
  if (!res.ok()) {
    throw std::runtime_error(fmt::format("Error retrieving dense sample from db. ({})", static_cast<unsigned>(res.code())));
  }

  // Deserialize sample
  Dense retrievedEntry;
  if (!retrievedEntry.ParseFromString(retrievedStringData)) {
    throw std::runtime_error("Error parsing dense sample");
  }

  // Get sensor infos
  auto sensor = getDenseSensor(retrievedEntry.sensor_id());

  // Copy image into the correct format
  // We need to do a copy here because the original memory will be
  // freed by RAII at the end of this function
  cv::Mat image(sensor.getHeight(), sensor.getWidth(), CV_8UC3);
  memcpy(image.data, retrievedEntry.color_image().data(), retrievedEntry.color_image().size());
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

    if (store_in_database) {

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
      std::string key = DB_KEY_SPARSE_DATA_PREFIX + fmt::format("{:0>{}}", sparse_data_count_,
          DB_NUMBER_DIGITS_FOR_IDX);

      // Store in DB
      if (!db_->Put(rocksdb::WriteOptions(), key, serialized_datapoint).ok()) {
        std::cerr << "Failed to save sparse entry in DB";
      }
    }

    sparse_data_count_++;
  }
}

void Inspection::addImageImpl(
  const cv::Mat & color_image,
  const cv::Mat & depth_image,
  double depth_trunc,
  const int sensor_id,
  const Eigen::Matrix4d & extrinsic_optical,
  const Eigen::Matrix4d & extrinsic_world,
  bool store_in_database)
{
  // we can only integrate if we already received the intrinsic calibration for this sensor
  const auto intrinsics_it = intrinsic_.find(sensor_id);
  if (intrinsics_it == intrinsic_.end()) {
    std::cerr << "No intrinsic calibration available for sensor " << sensor_id
              << " Will not integrate." << std::endl;
    return;
  }
  const auto & intrinsics = intrinsics_it->second;

  // Get sensor
  const auto sensor = getDenseSensor(sensor_id);
  assert(color_image.rows == sensor.getHeight());
  assert(color_image.cols == sensor.getWidth());
  const double depth_scale = sensor.getDepthScale();

  // Convert image to open3d representation
  assert(color_image.type() == CV_8UC3);
  // This is zero-copy, so make sure the original Mat outlives this object
  const open3d::core::Tensor color_image_o3d_view(
    color_image.data,
    open3d::core::UInt8,
    {color_image.rows, color_image.cols, color_image.channels()},
            /*stride in elements (not bytes)*/
    {int64_t(color_image.step[0] / color_image.elemSize1()),
      int64_t(color_image.step[1] / color_image.elemSize1()), 1});
  // This copies the data to the GPU if needed
  // If we run on the CPU, this is also zero copy
  // Make sure the original Mat outlives this object
  auto o3d_color_image = color_image_o3d_view.To(o3d_device_);
  
  // TSDF with voxel block grid currently only supports to
  // have color images as floats when depth is float
  if(depth_image.type() == CV_32FC1) {
    o3d_color_image = o3d_color_image.To(open3d::core::Dtype::Float32) / 255;
  }

  // Convert depth to open3d representation
  // Map types
  assert(depth_image.type() == CV_16UC1 or depth_image.type() == CV_32FC1);
  const std::map<int, open3d::core::Dtype> depth_type_map = {
    {CV_32FC1, open3d::core::Float32},
    {CV_16UC1, open3d::core::UInt16},
  };
  // This is zero-copy, so make sure the original Mat outlives this object
  const open3d::core::Tensor depth_image_o3_view(
    depth_image.data,
    depth_type_map.at(depth_image.type()),
    {depth_image.rows, depth_image.cols, depth_image.channels()},
            /*stride in elements (not bytes)*/
    {int64_t(depth_image.step[0] / depth_image.elemSize1()),
      int64_t(depth_image.step[1] / depth_image.elemSize1()), 1});
  // This copies the data to the GPU if needed
  // If we run on the CPU, this is also zero copy
  // Make sure the original Mat outlives this object
  const auto o3d_depth_image = depth_image_o3_view.To(o3d_device_);

  const auto focal_length = intrinsics.GetFocalLength();
  const auto principal_point = intrinsics.GetPrincipalPoint();

  const open3d::core::Tensor intrinsic_tens = open3d::core::Tensor::Init<double>(
    {{focal_length.first, 0, principal_point.first},
      {0, focal_length.second, principal_point.second},
      {0, 0, 1}});

  const open3d::core::Tensor extrinsic_tens =
    open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_optical);

  open3d::core::Tensor frustum_block_coords;
  try {
    //todo it might make sense to restrict these blocks to the cropped area (would reduce RAM usage)
    frustum_block_coords = voxel_grid_.GetUniqueBlockCoordinates(o3d_depth_image, intrinsic_tens,
      extrinsic_tens, depth_scale, depth_trunc);
  } catch (const std::runtime_error & e) {
    std::cerr << "no block is touched in tsdf volume, abort integration of this image. "
      "please check depth_scale and voxel_size, as well as depth_max of GetUniqueBlockCoordinates"
              << std::endl;
    return;
  }

  // todo ensure depth maps are registered before hand (only one camera info needed in that case)
  voxel_grid_.Integrate(frustum_block_coords, o3d_depth_image, o3d_color_image, intrinsic_tens,
      extrinsic_tens, depth_scale, depth_trunc);

  const std::array<double, 6> image_pose = transformMatrixToPose(extrinsic_world);

  {
    std::lock_guard<std::mutex> mtx_lock(mtx_);
    dense_posetree_.Insert(dense_data_count_, image_pose, false);
    dense_pose_.push_back(image_pose);


    // save dense data to database
    if (store_in_database) {
      const std::string key = DB_KEY_DENSE_DATA_PREFIX + fmt::format("{:0>{}}", dense_data_count_,
          DB_NUMBER_DIGITS_FOR_IDX);
      std::string value = serializedStructForDenseEntry(
        dense_data_count_,
        sensor_id,
        color_image,
        depth_image,
        depth_trunc,
        extrinsic_optical,
        extrinsic_world,
        intrinsics
      );
      db_->Put(rocksdb::WriteOptions(), key, value);
    }

    dense_data_count_++;
  }
}

void Inspection::reconstructSparseFromDB()
{
  // Reset non-persistent state so we can reconstruct it from the DB
  clearSparse(false);

  // Create an iterator
  auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));

  // Iterate over keys with the specified prefix using a for loop
  for (it->Seek(DB_KEY_SPARSE_DATA_PREFIX);
    it->Valid() && it->key().starts_with(DB_KEY_SPARSE_DATA_PREFIX); it->Next())
  {
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
      true,
      false
    );
  }
}

void Inspection::reconstructDenseFromDB()
{
  // Clear local state
  clearDense(false);

  // Create an iterator
  auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));

  // Iterate over keys with the specified prefix using a for loop
  for (it->Seek(DB_KEY_DENSE_DATA_PREFIX);
    it->Valid() && it->key().starts_with(DB_KEY_DENSE_DATA_PREFIX); it->Next())
  {
    const auto sample = it->value().ToString();

    Dense retrievedEntry;
    if (!retrievedEntry.ParseFromString(sample)) {
      throw std::runtime_error("Failed to parse dense data");
    }

    // Get sensor information
    const auto sensor = getDenseSensor(retrievedEntry.sensor_id());

    // Zero-copy the data to OpenCV Mat objects
    // We need to make sure that the original object outlives these
    // references, which is the case here, but be careful when you
    // modify something in addImageImpl or here
    const cv::Mat color_img(
      sensor.getHeight(),
      sensor.getWidth(),
      CV_8UC3,
      static_cast<void *>(retrievedEntry.mutable_color_image()->data()));
    const cv::Mat depth_img(
      sensor.getHeight(),
      sensor.getWidth(),
      retrievedEntry.depth_dtype(),
      static_cast<void *>(retrievedEntry.mutable_depth_image()->data()));

    // Add intrinsics for sensor
    setIntrinsic(
      open3d::camera::PinholeCameraIntrinsic(
        sensor.getWidth(),
        sensor.getHeight(),
        matrixFromFlatProtoArray<3, 3>(retrievedEntry.intrinsics_matrix())
      ),
      retrievedEntry.sensor_id()
    );

    // "Observe" the measurement again to integrate it into e.g. the TSDF
    addImageImpl(
      color_img,
      depth_img,
      retrievedEntry.depth_trunc(),
      retrievedEntry.sensor_id(),
      matrixFromFlatProtoArray<4, 4>(retrievedEntry.extrinsic_optical_matrix()),
      matrixFromFlatProtoArray<4, 4>(retrievedEntry.extrinsic_world_matrix()),
      false
    );
  }
}

std::shared_ptr<open3d::geometry::TriangleMesh> Inspection::extractDenseReconstruction()
{
  // TODO it should probably be also a parameter how often we want to see one point
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>(
    voxel_grid_.ExtractTriangleMesh(10.0f).ToLegacy()
  );

  // todo maybe give some warning if the whole mesh is cropped to 0 triangles
  std::shared_ptr<open3d::geometry::TriangleMesh> cropped_mesh = mesh->Crop(crop_box_);

  return cropped_mesh;
}

void Inspection::saveDenseReconstruction(std::string filename)
{
  auto mesh = extractDenseReconstruction();
  open3d::io::WriteTriangleMesh(filename, *mesh, true);
}

void Inspection::reinitializeTSDF(double voxel_length)
{
  voxel_grid_ =
    open3d::t::geometry::VoxelBlockGrid({"tsdf", "weight", "color"},
      {open3d::core::Dtype::Float32, open3d::core::Dtype::UInt16, open3d::core::Dtype::UInt16},
      {{1}, {1}, {3}}, voxel_length, 16, 50000, o3d_device_);   //todo make block count a parameter
  
  reconstructDenseFromDB();
}

void Inspection::clear()
{
  clearSparse(true);
  clearDense(true);
}

void Inspection::clearSparse(bool wipe_db)
{
  if (!getSparseUsage()) {
    return;
  }

  // Clear persistent storage
  if(wipe_db) 
  {
    db_->DeleteRange(rocksdb::WriteOptions(), DB_KEY_SPARSE_DATA_PREFIX,
      DB_KEY_SPARSE_DATA_PREFIX + '\xFF');
  }

  // Clear non persistent storage
  sparse_data_count_ = 0;
  sparse_timestamp_.clear();
  sparse_position_.clear();
  sparse_orientation_.clear();
  sparse_value_.clear();
  sparse_user_color_.clear();

  auto num_sparse_types = sparse_value_infos_.size();
  sparse_min_values_ = std::vector<double>(num_sparse_types, std::numeric_limits<double>::max());
  sparse_max_values_ = std::vector<double>(num_sparse_types,
      std::numeric_limits<double>::lowest());
  
  // Recreate octree
  sparse_octree_ = OrthoTree::OctreePointC(sparse_position_, OCTREE_DEPTH, inspection_space_3d_);
}

void Inspection::clearDense(bool wipe_db) 
{
  if (!getDenseUsage()) {
    return;
  }

  // Clear persistent storage
  if(wipe_db) 
  {
    db_->DeleteRange(rocksdb::WriteOptions(), DB_KEY_DENSE_DATA_PREFIX,
    DB_KEY_DENSE_DATA_PREFIX + '\xFF');
  }

  // Clear non persistent storage
  dense_data_count_ = 0;
  dense_timestamp_.clear();
  dense_pose_.clear();
  dense_orientation_.clear();

  // Recreate Octree
  dense_posetree_ = OrthoTree::TreePointPoseND<6, {0, 0, 0, 1, 1, 1}, std::ratio<1, 2>, double>();
  dense_posetree_.Create(
    dense_posetree_, dense_pose_, OCTREE_DEPTH, inspection_space_6d_,
    MAX_POSES_IN_LEAF);
}

// todo could also add a field to store an arbitrary string as a note or comment

bool Inspection::saveMetaData()
{
  json j = {
    {"header", {
      // Store the current unix timestamp
        {"Creation time", std::chrono::system_clock::now().time_since_epoch().count()},
      // Git hash of the version that created the file
        {"Git hash", GIT_COMMIT_HASH}
      }},
    {"3D Inspection space", {
        {"min", inspection_space_3d_.Min},
        {"max", inspection_space_3d_.Max}
      }},
    {"6D Inspection space", {
        {"min", inspection_space_6d_.Min},
        {"max", inspection_space_6d_.Max}
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

void Inspection::storeMesh(const std::string & key, const open3d::geometry::TriangleMesh & mesh)
{
  // Check if the mesh has any vertices
  if (mesh.vertices_.size() == 0) {
    return;
  }

  // Open3d only supports export to files, not byte streams, we we need to use a temp file
  std::string temp_file = fmt::format("/tmp/vinspect_mesh_{}.ply", std::rand());
  open3d::io::WriteTriangleMesh(temp_file, mesh, true);
  std::ifstream file(temp_file);

  // Check if file can be opened
  if (!file.is_open()) {
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
  if(!db_->Put(rocksdb::WriteOptions(), key, content).ok()) {
    throw std::runtime_error("Could not store mesh in db");
  }
}

void Inspection::saveDiconde(const std::string & folder_path)
{
  // Acquire mutex for the full save duration, so nothing changes during the saving process
  std::lock_guard<std::mutex> lock(mtx_);

  // Create Folder
  const fs::path save_folder(folder_path);
  const fs::path diconde_folder = save_folder / "diconde";
  const fs::path image_folder = diconde_folder / "images"; 

  std::filesystem::create_directory(diconde_folder);
  std::filesystem::create_directory(image_folder);


  // Create global metadata    // TODO rename to DICONDE names
  auto patient_name = "Doe^John"; // TODO make parameter
  auto patient_id = "0000";

  char studyInstanceUID[100];
  dcmGenerateUniqueIdentifier(studyInstanceUID);
  
  // Dense export
  {
    // TODO maybe a series for each sensor
    char imageSeriesUID[100], depthSeriesUID[100];
    dcmGenerateUniqueIdentifier(imageSeriesUID);
    dcmGenerateUniqueIdentifier(depthSeriesUID);

    // Go through all images
    std::size_t img_idx = 0;
    auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));
    for (it->Seek(DB_KEY_DENSE_DATA_PREFIX);
      it->Valid() && it->key().starts_with(DB_KEY_DENSE_DATA_PREFIX); it->Next())
    {
      // Decode data
      const auto sample = it->value().ToString();
      Dense retrievedEntry;
      if (!retrievedEntry.ParseFromString(sample)) {
        throw std::runtime_error("Failed to parse dense data");
      }
      
      // Get sensor information
      const auto sensor = getDenseSensor(retrievedEntry.sensor_id());
      
      // Create UIDs
      char imageSOPUID[100], depthSOPUID[100], frameOfReferenceUID[100];
      dcmGenerateUniqueIdentifier(imageSOPUID, SITE_INSTANCE_UID_ROOT);
      dcmGenerateUniqueIdentifier(depthSOPUID, SITE_INSTANCE_UID_ROOT);
      dcmGenerateUniqueIdentifier(frameOfReferenceUID);

      // Write RGB Image
      {  
        DcmFileFormat fileformat;
        DcmDataset *dataset = fileformat.getDataset();
    
        dataset->putAndInsertString(DCM_PatientID, patient_id);
        dataset->putAndInsertString(DCM_PatientName, patient_name);
        dataset->putAndInsertString(DCM_SOPClassUID, UID_SecondaryCaptureImageStorage);        
        dataset->putAndInsertString(DCM_SOPInstanceUID, imageSOPUID);
        dataset->putAndInsertString(DCM_SeriesInstanceUID, imageSeriesUID);
        dataset->putAndInsertString(DCM_SeriesDescription, "Dense RGB Image");
        dataset->putAndInsertString(DCM_Modality, "OT");   // Other
        dataset->putAndInsertString(DCM_ConversionType, "DV");
        dataset->putAndInsertString(DCM_StudyInstanceUID, studyInstanceUID);
        dataset->putAndInsertString(DCM_FrameOfReferenceUID, frameOfReferenceUID);

        // Cross reference the depth
        {
          DcmItem *depthReference = new DcmItem();
          depthReference->putAndInsertString(DCM_ReferencedSOPClassUID, UID_SecondaryCaptureImageStorage);
          depthReference->putAndInsertString(DCM_ReferencedSOPInstanceUID, depthSOPUID);
          {
            DcmItem *purposeReferenceCode = new DcmItem();
            purposeReferenceCode->putAndInsertString(DCM_CodeValue, "121313");
            purposeReferenceCode->putAndInsertString(DCM_CodingSchemeDesignator, "DCM");
            purposeReferenceCode->putAndInsertString(DCM_CodeMeaning, "Other partial views");  // Maybe find a better code
            depthReference->insertSequenceItem(DCM_PurposeOfReferenceCodeSequence, purposeReferenceCode);
          }
          dataset->insertSequenceItem(DCM_ReferencedImageSequence, depthReference);
        }
    
        // ------------------------------------------------------------------
        // Set the image attributes
        // ------------------------------------------------------------------
        dataset->putAndInsertUint16(DCM_Rows, sensor.getHeight());
        dataset->putAndInsertUint16(DCM_Columns, sensor.getWidth());
        dataset->putAndInsertUint16(DCM_SamplesPerPixel, 3);            // RGB
        dataset->putAndInsertString(DCM_PhotometricInterpretation, "RGB");
        dataset->putAndInsertUint16(DCM_BitsAllocated, 8);              // 8‑bit per channel
        dataset->putAndInsertUint16(DCM_BitsStored, 8);
        dataset->putAndInsertUint16(DCM_HighBit, 7);
        dataset->putAndInsertUint16(DCM_PixelRepresentation, 0);       // unsigned
        dataset->putAndInsertUint16(DCM_PlanarConfiguration, 0);        // interleaved RGB
        dataset->putAndInsertString(DCM_NumberOfFrames, "1");
        dataset->putAndInsertUint16(DCM_SmallestImagePixelValue, 0);
        dataset->putAndInsertUint16(DCM_LargestImagePixelValue, 255);
    
    
        // ------------------------------------------------------------------
        // Store the raw pixel data
        // ------------------------------------------------------------------
        dataset->putAndInsertUint8Array(
          DCM_PixelData, 
          reinterpret_cast<const std::uint8_t*>(
            retrievedEntry.mutable_color_image()->data()
          ), 
          retrievedEntry.mutable_color_image()->size());
          
        // Write to disk
        fs::path dcm_current_image_path = image_folder / fmt::format("{:0>{}}.dcm", img_idx, 12);
    
        OFCondition cond = fileformat.saveFile(dcm_current_image_path.c_str(), EXS_LittleEndianExplicit);
        if (!cond.good())
        {
            std::cerr << "Error saving DICOM file: " << cond.text() << std::endl;
        }  
      }
  
      // Write Depth Image
      {
        DcmFileFormat fileformat;
        DcmDataset *dataset = fileformat.getDataset();
    
        dataset->putAndInsertString(DCM_PatientID, patient_id);
        dataset->putAndInsertString(DCM_PatientName, patient_name);
        dataset->putAndInsertString(DCM_SOPClassUID, UID_SecondaryCaptureImageStorage);        
        dataset->putAndInsertString(DCM_SOPInstanceUID, depthSOPUID);
        dataset->putAndInsertString(DCM_SeriesInstanceUID, depthSeriesUID);
        dataset->putAndInsertString(DCM_SeriesDescription, "Dense Depth Image");
        dataset->putAndInsertString(DCM_Modality, "OT");   // Other
        dataset->putAndInsertString(DCM_ConversionType, "DV");
        dataset->putAndInsertString(DCM_StudyInstanceUID, studyInstanceUID);
        dataset->putAndInsertString(DCM_FrameOfReferenceUID, frameOfReferenceUID);

        // Cross reference the RGB
        {
          DcmItem *rgbReference = new DcmItem();
          rgbReference->putAndInsertString(DCM_ReferencedSOPClassUID, UID_SecondaryCaptureImageStorage);
          rgbReference->putAndInsertString(DCM_ReferencedSOPInstanceUID, imageSOPUID);
          {
            DcmItem *purposeReferenceCode = new DcmItem();
            purposeReferenceCode->putAndInsertString(DCM_CodeValue, "121313");
            purposeReferenceCode->putAndInsertString(DCM_CodingSchemeDesignator, "DCM");
            purposeReferenceCode->putAndInsertString(DCM_CodeMeaning, "Other partial views");  // Maybe find a better code
            rgbReference->insertSequenceItem(DCM_PurposeOfReferenceCodeSequence, purposeReferenceCode);
          }
          dataset->insertSequenceItem(DCM_ReferencedImageSequence, rgbReference);
        }


    
        // ------------------------------------------------------------------
        // Set the image attributes
        // ------------------------------------------------------------------
        dataset->putAndInsertUint16(DCM_Rows, sensor.getHeight());
        dataset->putAndInsertUint16(DCM_Columns, sensor.getWidth());
        dataset->putAndInsertUint16(DCM_SamplesPerPixel, 1);            // Depth
        dataset->putAndInsertString(DCM_PhotometricInterpretation, "MONOCHROME2");
        dataset->putAndInsertUint16(DCM_SmallestImagePixelValue, 0);
        dataset->putAndInsertUint16(DCM_PixelRepresentation, 0);  // unsigned
  
        if (retrievedEntry.depth_dtype() == CV_16UC1) {
          dataset->putAndInsertUint16(DCM_BitsAllocated, 16);
          dataset->putAndInsertUint16(DCM_BitsStored, 16);  // TODO check if the alignment etc are correct
          dataset->putAndInsertUint16(DCM_HighBit, 15);
  
          dataset->putAndInsertUint16Array(
            DCM_PixelData,
            reinterpret_cast<const std::uint16_t*>(
              retrievedEntry.mutable_depth_image()->data()
            ), 
            retrievedEntry.mutable_depth_image()->size()
          );
        } else if (retrievedEntry.depth_dtype() == CV_32FC1)
        {
          dataset->putAndInsertUint16(DCM_BitsAllocated, 32);
          dataset->putAndInsertUint16(DCM_BitsStored, 32);
          dataset->putAndInsertUint16(DCM_HighBit, 31);
  
          dataset->putAndInsertFloat32Array(
            DCM_FloatPixelData,
            reinterpret_cast<const float*>(
              retrievedEntry.mutable_depth_image()->data()
            ), 
            retrievedEntry.mutable_depth_image()->size());
        } else {
          std::runtime_error("Unsupported depth data type");
        }
    
          
        // Write to disk
        fs::path dcm_current_image_path = image_folder / fmt::format("{:0>{}}_depth.dcm", img_idx, 12);
    
        OFCondition cond = fileformat.saveFile(dcm_current_image_path.c_str(), EXS_LittleEndianExplicit);
        if (!cond.good())
        {
            std::cerr << "Error saving DICOM file: " << cond.text() << std::endl;
        }  
      }
      img_idx++;
    }
  }


  // Save sparse data
  {
    char sparseSeriesUid[100], sop_uid[100];
    dcmGenerateUniqueIdentifier(sparseSeriesUid);
    dcmGenerateUniqueIdentifier(sop_uid, SITE_INSTANCE_UID_ROOT);

    DcmFileFormat fileformat;
    DcmDataset *dataset = fileformat.getDataset();

    dataset->putAndInsertString(DCM_PatientID, patient_id);
    dataset->putAndInsertString(DCM_PatientName, patient_name);
    dataset->putAndInsertString(DCM_Modality, "OT");
    dataset->putAndInsertString(DCM_SeriesDescription, "Sparse Point Measurements");
    
    dataset->putAndInsertString(DCM_SOPInstanceUID, sop_uid);
    dataset->putAndInsertString(DCM_SeriesInstanceUID, sparseSeriesUid);
    dataset->putAndInsertString(DCM_SeriesDescription, "Dense Depth Image");

    // TODO timestamps

    std::size_t sample_idx = 0;
    auto it = std::unique_ptr<rocksdb::Iterator>(db_->NewIterator(rocksdb::ReadOptions()));
    for (it->Seek(DB_KEY_SPARSE_DATA_PREFIX);
      it->Valid() && it->key().starts_with(DB_KEY_SPARSE_DATA_PREFIX); it->Next())
    {
      json sample = json::parse(it->value().ToStringView());

      Eigen::Vector3d user_color(
        sample["user_color"]["r"],
        sample["user_color"]["g"],
        sample["user_color"]["b"]
      );

      // TODO finalize

      // addSparseMeasurementImpl(
      //   sample["timestamp"],
      //   sample["sensor_id"],
      //   sample["position"],
      //   sample["orientation"],
      //   sample["value"],
      //   user_color,
      //   true,
      //   false
      // );

    }
  }

}

void Inspection::loadMesh(const std::string & key, open3d::geometry::TriangleMesh & mesh)
{
  // Get mesh from db
  std::string serialized_mesh;

  auto response = db_->Get(rocksdb::ReadOptions(), key, &serialized_mesh);
  if (response.IsNotFound()) {
    // No mesh present, initialize with empty mesh instead
    mesh = open3d::geometry::TriangleMesh();
    return;
  } else if (!response.ok()) {
    throw std::runtime_error("Could load mesh from db");
  }

  // Open3d only supports export to files, not byte streams, we we need to use a temp file
  std::string temp_file = fmt::format("/tmp/vinspect_mesh_{}.ply", std::rand());

  std::ofstream file(temp_file);
  // Check if file can be opened
  if (!file.is_open()) {
    throw std::runtime_error(fmt::format("Error opening temporary file: '{}'", temp_file));
  }

  // Write mesh to file
  file << serialized_mesh;
  file.close();

  // Load mesh into open3d
  if (!open3d::io::ReadTriangleMesh(temp_file, mesh)) {
    throw std::runtime_error("Could not load mesh from file");
  }

  // Close and remove the file
  std::remove(temp_file.c_str());
}
}  // namespace vinspect
