/*
 * SlamWrapper.cpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam/SlamWrapper.hpp"

#include <open3d/Open3D.h>
#include <chrono>
#include <iomanip>
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/MotionCompensation.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
const double timingStatsEveryNsec = 15.0;
}  // namespace

SlamWrapper::SlamWrapper() {
  motionCompensationOdom_ = std::make_shared<MotionCompensation>();
  motionCompensationMap_ = std::make_shared<MotionCompensation>();
  latestMapToRangeMeasurement_.transform_ = Transform::Identity();
  latestScanToMapRefinementTimestamp_ = fromUniversal(0);
}

SlamWrapper::~SlamWrapper() {
  if (odometryWorker_.joinable()) {
    odometryWorker_.join();
    std::cout << "Joined odometry worker \n";
  }
  if (mappingWorker_.joinable()) {
    mappingWorker_.join();
    std::cout << "Joined mapping worker \n";
  }
  if (params_.mapper_.isAttemptLoopClosures_ && loopClosureWorker_.joinable()) {
    loopClosureWorker_.join();
    std::cout << "Joined the loop closure worker \n";
  }

  if (params_.mapper_.isBuildDenseMap_ && denseMapWorker_.joinable()) {
    denseMapWorker_.join();
    std::cout << "Joined the dense map worker! \n";
  }

  if (unifiedWorker_.joinable()) {
    unifiedWorker_.join();
    std::cout << "Joined unifiedWorker_ worker \n";
  }

  if (unifiedWorkerOdom_.joinable()) {
    unifiedWorkerOdom_.join();
    std::cout << "Joined unifiedWorkerOdom_ worker \n";
  }

  if (unifiedWorkerMap_.joinable()) {
    unifiedWorkerMap_.join();
    std::cout << "Joined unifiedWorkerMap_ worker \n";
  }

  std::cout << "    Scan insertion: Avg execution time: " << mapperOnlyTimer_.getAvgMeasurementMsec()
            << " msec , frequency: " << 1e3 / mapperOnlyTimer_.getAvgMeasurementMsec() << " Hz \n";

  if (params_.saving_.isSaveAtMissionEnd_) {
    std::cout << "Saving maps .... \n";
    if (params_.saving_.isSaveMap_) {
      if (!saveMap(mapSavingFolderPath_)) {
      std::cout << "\033[31m[Error] Failed to save map to " << mapSavingFolderPath_ << "\033[0m" << std::endl;
      return;
      }
    }
    if (params_.saving_.isSaveSubmaps_) {
      saveSubmaps(mapSavingFolderPath_);
    }
    if (params_.mapper_.isBuildDenseMap_ && params_.saving_.isSaveDenseSubmaps_) {
      saveDenseSubmaps(mapSavingFolderPath_);
    }
    std::cout << "All done! \n";
    std::cout << "Maps saved in " << mapSavingFolderPath_ << "\n";
  }
}

const MapperParameters& SlamWrapper::getMapperParameters() const {
  return params_.mapper_;
}

bool SlamWrapper::isIMUattitudeInitializationEnabled() {
  return params_.odometry_.isIMUattitudeInitializationEnabled_;
}

MapperParameters* SlamWrapper::getMapperParametersPtr() {
  return mapper_->getParametersPtr();
}
size_t SlamWrapper::getOdometryBufferSize() const {
  return odometryBuffer_.size();
}
size_t SlamWrapper::getMappingBufferSize() const {
  return mappingBuffer_.size();
}

size_t SlamWrapper::getOdometryBufferSizeLimit() const {
  return odometryBuffer_.size_limit();
}
size_t SlamWrapper::getMappingBufferSizeLimit() const {
  return mappingBuffer_.size_limit();
}

void SlamWrapper::appendPoseToTrackedPath(geometry_msgs::msg::PoseStamped transform) {
  {
    std::lock_guard<std::mutex> lock(mapper_->pathMutex_);
    mapper_->trackedPath_.poses.push_back(transform);
  }
}

void SlamWrapper::appendPoseToBestGuessPath(geometry_msgs::msg::PoseStamped transform) {
  {
    // TODO pass reference herE?
    std::lock_guard<std::mutex> lock(mapper_->pathMutex_);
    mapper_->bestGuessPath_.poses.push_back(transform);
  }
}

bool SlamWrapper::isExternalOdometryFrameToCloudFrameCalibrationSet() {
  return mapper_->isCalibrationSet_;
}

Transform SlamWrapper::getExternalOdometryFrameToCloudFrameCalibration() {
  if (!isExternalOdometryFrameToCloudFrameCalibrationSet()) {
    std::cout << "ASKED FOR CALIBRATION BEFORE ITS SET RETURNING IDENTITY \n";
    return Transform::Identity();
  }

  return mapper_->calibration_;
}

bool SlamWrapper::addOdometryPoseToBuffer(const Transform& transform, const Time& timestamp) const {
  // if (!odometry_->odomToRangeSensorBuffer_.empty()) {
  //   try {
  //     auto latestTime = odometry_->odomToRangeSensorBuffer_.latest_time();
  //     std::cout << "Latest time in odomToRangeSensorBuffer_: " << toSecondsSinceFirstMeasurement(latestTime) << std::endl;
  //   } catch (const std::exception& e) {
  //     std::cout << "Exception when getting latest_time: " << e.what() << std::endl;
  //   }
  // } else {
  //   std::cout << "odomToRangeSensorBuffer_ is empty." << std::endl;
  // }
  // // std::cout << "Current odometry buffer size: " << odometry_->odomToRangeSensorBuffer_.size() << std::endl;

  if (!(params_.odometry_.useOdometryTopic_) || odometry_->odomToRangeSensorBuffer_.has(timestamp)) {
    std::cout << "WARNING: you are trying to add an odometry pose to the buffer, but the buffer already has it! \n";
    std::cout << "The timestamp is: " << std::setprecision(10) << toSecondsSinceFirstMeasurement(timestamp) << std::endl;
    return false;
  }

  updateFirstMeasurementTime(timestamp);

  if (!odometryBuffer_.empty()) {
    const auto latestTime = odometryBuffer_.peek_back().time_;
    if (timestamp < latestTime) {
      std::cerr << "You are trying to add a pose odometry measurement out of order! Its okay at the start. Dropping the measurement! \n";
      return false;
    }
  }

  if (!odometry_->odomToRangeSensorBuffer_.empty()) {
    const auto latestAvailableOdometryTime = odometry_->odomToRangeSensorBuffer_.latest_time();
    if (timestamp < latestAvailableOdometryTime) {
      std::cerr << "You are trying to add a pose odometry measurement out of order! Its okay. Dropping the measurement. \n";
      return false;
    }
  } else {
    std::cout << "odometry_->odomToRangeSensorBuffer_ is empty when adding odometry pose.\n";
  }

  // std::cout << "Pushing odometry pose with timestamp: " << toSecondsSinceFirstMeasurement(timestamp) << std::endl;
  odometry_->odomToRangeSensorBuffer_.push(timestamp, transform);
  return true;
}

// this is a function that returns a boolean whether the use odometry topic or not using the parameters.
bool SlamWrapper::isUsingOdometryTopic() const {
  return params_.odometry_.useOdometryTopic_;
}

bool SlamWrapper::addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp) {
  // Set the time regardless of whats going to happen next.
  updateFirstMeasurementTime(timestamp);

  if (params_.odometry_.useOdometryTopic_) {
    // Doesn't make sense to add the measurement if the pose buffer is empty.
    if (odometry_->odomToRangeSensorBuffer_.empty()) {
      std::cerr << "open3d_slam: You are trying to add a range scan without a pose in the buffer. Its okay. Skipping. \n";
      return false;
    }

    const auto earliestAvailableOdometryTime = odometry_->odomToRangeSensorBuffer_.earliest_time();
    if (timestamp < earliestAvailableOdometryTime) {
      std::cerr << "open3d_slam: You are trying to add a range scan earlier than all the odometry poses. Its okay. Dropping. \n";
      std::cout << "Earliest available odometry time: " << toString(earliestAvailableOdometryTime) << std::endl;
      std::cout << "Requested time: " << toString(timestamp) << std::endl;
      return false;
    }
  }

  if (!odometryBuffer_.empty()) {
    const auto latestTime = odometryBuffer_.peek_back().time_;
    if (timestamp < latestTime) {
      std::cerr << "open3d_slam: You are trying to add a range scan out of order! Dropping the measurement! \n";
      return false;
    }
  }

  // TODO, do we need this?
  // auto removedNans = removePointsWithNonFiniteValues(cloud);
  const TimestampedPointCloud timestampedCloud{timestamp, cloud};

  odometryBuffer_.push(timestampedCloud);
  return true;
}

std::tuple<PointCloud, Time, Transform> SlamWrapper::getLatestRegisteredCloudTimestampPair() const {
  if (registeredCloudBuffer_.empty()) {
    return {std::make_tuple(PointCloud(), latestScanToMapRefinementTimestamp_, Transform())};
  }
  RegisteredPointCloud c = registeredCloudBuffer_.peek_back();
  //	c.raw_.cloud_.Transform(c.transform_.matrix());
  return {std::make_tuple(c.raw_.cloud_, c.raw_.time_, c.transform_)};
}

std::tuple<Time, Transform> SlamWrapper::getLatestRegistrationBestGuess() const {
  if (registrationBestGuessBuffer_.empty()) {
    return {std::make_tuple(latestScanToMapRefinementTimestamp_, Transform())};
  }

  ScanToMapRegistrationBestGuess c = registrationBestGuessBuffer_.peek_back();
  return {std::make_tuple(c.time_, c.transform_)};
}

void SlamWrapper::setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform) {
  // Eigen::Isometry3d calibrationIsometry = Eigen::Translation3d(0, 0.364, -0.1422) * Eigen::AngleAxisd(M_PI / 2,
  // Eigen::Vector3d::UnitZ()); Not thread safe?
  mapper_->calibration_ = transform.matrix();
  mapper_->isCalibrationSet_ = true;
  return;
}

TimestampedTransform SlamWrapper::getLatestMapToRangeMeasurement() const {
  if (mapper_->getMapToRangeSensorBuffer().empty()) {
    return TimestampedTransform();
  }

  auto latestMapToRangeMeasurement_ = mapper_->getMapToRangeSensorBuffer().latest_measurement();
  return latestMapToRangeMeasurement_;
}

TimestampedTransform SlamWrapper::getLatestOdometryPoseMeasurement() const {
  if (odometry_->getBuffer().empty()) {
    std::cout << "\033[31mOdometry buffer is empty! Returning empty transform.\033[0m" << std::endl;
    return TimestampedTransform();
  }

  auto latestOdomMeasurement = odometry_->getBuffer().latest_measurement();
  return latestOdomMeasurement;
}

void SlamWrapper::finishProcessing() {
  while (isRunWorkers_) {
    if (!mappingBuffer_.empty()) {
      std::cout << "  Waiting for the mapping buffer to be emptied \n";
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    } else {
      std::cout << "  Mapping buffer emptied \n";
      break;
    }
  }
  std::cout << "Finishing all submaps! \n";
  numLatesLoopClosureConstraints_ = -1;
  submaps_->forceNewSubmapCreation();
  while (isRunWorkers_) {
    computeFeaturesIfReady();
    if (params_.mapper_.isAttemptLoopClosures_) {
      attemptLoopClosuresIfReady();
    } else {
      break;
    }
    if (numLatesLoopClosureConstraints_ == 0) {
      break;
    }
    if (isOptimizedGraphAvailable_) {
      isOptimizedGraphAvailable_ = false;
      const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
      std::cout << "latest pose before update: \n " << asStringXYZRPY(poseBeforeUpdate.transform_) << "\n";
      updateSubmapsAndTrajectory();
      const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
      std::cout << "latest pose after update: \n " << asStringXYZRPY(poseAfterUpdate.transform_) << "\n";
      if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
        submaps_->dumpToFile(folderPath_, "after", false);
      }
      break;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  std::cout << "All submaps fnished! \n";
}

void SlamWrapper::offlineFinishProcessing() {
  if (!mappingBuffer_.empty()) {
    std::cout << "  Waiting for the mapping buffer to be emptied \n";
    return;
  }

  std::cout << "Finishing all submaps! \n";
  numLatesLoopClosureConstraints_ = -1;
  submaps_->forceNewSubmapCreation();
  computeFeaturesIfReady();
  if (params_.mapper_.isAttemptLoopClosures_) {
    // In the current setup the async thread is not helping.

    attemptLoopClosuresIfReady();
  } else {
    std::cout << "Loop closure feature is toggled off. Will not attempt! \n";
    return;
  }
  if (numLatesLoopClosureConstraints_ == 0) {
    std::cout << "Not enough loop closure constraints to optimize. Will not attempt! \n";
    return;
  }

  if (isOptimizedGraphAvailable_) {
    isOptimizedGraphAvailable_ = false;
    const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
    std::cout << "latest pose before update: \n " << asStringXYZRPY(poseBeforeUpdate.transform_) << "\n";
    updateSubmapsAndTrajectory();
    const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
    std::cout << "latest pose after update: \n " << asStringXYZRPY(poseAfterUpdate.transform_) << "\n";
    if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
      submaps_->dumpToFile(folderPath_, "after", false);
    }
    return;
  }

  std::cout << "All submaps fnished! \n";
}

// Set Directory Path
void SlamWrapper::setDirectoryPath(const std::string& path) {
  folderPath_ = path;
}

// Set Saving Directory Path
void SlamWrapper::setMapSavingDirectoryPath(const std::string& path) {
  mapSavingFolderPath_ = path;
}

void SlamWrapper::offlineTfWorker() {
  std::cout << "Starting offline tf worker! \n";
}

void SlamWrapper::offlineVisualizationWorker() {
  std::cout << "Starting offline visualization worker! \n";

  const Time scanToMapTimestamp = latestScanToMapRefinementTimestamp_;
  if (isTimeValid(scanToMapTimestamp)) {
    offlinePublishMaps(scanToMapTimestamp);
  }
}

void SlamWrapper::offlinePublishMaps(const Time& time) {
  /*const ros::Time timestamp = toRos(time);
  {
    PointCloud map = mapper_->getAssembledMapPointCloud();
    voxelize(params_.visualization_.assembledMapVoxelSize_, &map);
    o3d_slam::publishCloud(map, frames_.mapFrame, timestamp, assembledMapPub_);
  }
  o3d_slam::publishCloud(mapper_->getPreprocessedScan(), frames_.rangeSensorFrame, timestamp, mappingInputPub_);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
  if (submapsPub_.getNumSubscribers() > 0) {
    open3d::geometry::PointCloud cloud;
    o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
    voxelize(params_.visualization_.submapVoxelSize_, &cloud);
    o3d_slam::publishCloud(cloud, frames_.mapFrame, timestamp, submapsPub_);
  }
  */
}

void SlamWrapper::loadParametersAndInitialize() {
  //	auto &logger = open3d::utility::Logger::GetInstance();
  //	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

  odometry_ = std::make_shared<o3d_slam::LidarOdometry>();
  odometry_->setParameters(params_.odometry_);

  submaps_ = std::make_shared<o3d_slam::SubmapCollection>();
  submaps_->setFolderPath(folderPath_);

  mapper_ = std::make_shared<o3d_slam::Mapper>(odometry_->getBuffer(), submaps_);
  mapper_->setParameters(params_.mapper_);

  optimizationProblem_ = std::make_shared<o3d_slam::OptimizationProblem>();
  optimizationProblem_->setParameters(params_.mapper_);

  // set the verobsity for timing statistics
  Timer::isDisablePrintInDestructor_ = !params_.mapper_.isPrintTimingStatistics_;

  if (params_.motionCompensation_.isUndistortInputCloud_) {
    auto motionCompOdom = std::make_shared<ConstantVelocityMotionCompensation>(odometry_->getBuffer());
    motionCompOdom->setParameters(params_.motionCompensation_);
    motionCompensationOdom_ = motionCompOdom;
    auto motionCompMap = std::make_shared<ConstantVelocityMotionCompensation>(mapper_->getMapToRangeSensorBuffer());
    motionCompMap->setParameters(params_.motionCompensation_);
    motionCompensationMap_ = motionCompMap;
  }

  // Set the buffer sizes. This is not done in the constructer
  odometryBuffer_.set_size_limit(params_.odometry_.odometryBufferSize_);
  mappingBuffer_.set_size_limit(params_.mapper_.mappingBufferSize_);
  registeredCloudBuffer_.set_size_limit(params_.odometry_.scanProcessing_.pointCloudBufferSize_);
}

bool SlamWrapper::isUseExistingMapEnabled() const {
  return params_.mapper_.isUseInitialMap_;
}

void SlamWrapper::setInitialMap(const PointCloud& initialMap) {
  TimestampedPointCloud measurement{fromUniversal(0), std::move(initialMap)};
  {
    Timer t("initial map preparation");
    mapper_->getScanToMapRegistration().prepareInitialMap(&measurement.cloud_);
  }

  bool success = mapper_->setInitialMap(measurement.cloud_);
  std::cout << "Initial map prepared! \n";
  // const bool mappingResult = mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);
  if (!success) {
    std::cerr << "WARNING: mapping initialization has failed!!!! \n";
  }
}

void SlamWrapper::setInitialTransform(const Eigen::Matrix4d initialTransform) {
  odometry_->setInitialTransform(initialTransform);
  mapper_->setMapToRangeSensorInitial(Transform(initialTransform));
}

bool SlamWrapper::isInitialTransformSet() {
  bool total = mapper_->isInitialTransformSet_ && odometry_->isInitialTransformSet_;
  // std::cerr << "Mapper is set: " << mapper_->isNewValueSetMapper_<< " \n";
  // std::cerr << "Odometry is set: " << odometry_->isInitialTransformSet_ << " \n";

  return total;
}

void SlamWrapper::callofflineOdometryWorker() {
  offlineOdometryWorker();

  return;
}

void SlamWrapper::callofflineMappingWorker() {
  offlineMappingWorker();

  return;
}

void SlamWrapper::callofflineLoopClosureWorker() {
  offlineLoopClosureWorker();

  return;
}

void SlamWrapper::usePairForRegistration() {
  // std::cout << " Called pair for regist" << std::endl;
  offlineOdometryWorker();
  // std::cout << " Done with odometry" << std::endl;
  offlineMappingWorker();
  // std::cout << " Done with mapping" << std::endl;
  offlineLoopClosureWorker();
  // std::cout << " Done with loop" << std::endl;
}

void SlamWrapper::startWorkers() {
  // This is the new-multi threaded.
  unifiedWorkerOdom_ = std::thread([this]() { unifiedWorkerOdom(); });
  unifiedWorkerMap_ = std::thread([this]() { unifiedWorkerMap(); });

  // // This is single threaded.
  // unifiedWorker_ = std::thread([this]() { unifiedWorker(); });

  // Old threads for reference.
  if (params_.mapper_.isAttemptLoopClosures_) {
    loopClosureWorker_ = std::thread([this]() { loopClosureWorker(); });
  }
  if (params_.mapper_.isBuildDenseMap_) {
    denseMapWorker_ = std::thread([this]() { denseMapWorker(); });
  }
}

void SlamWrapper::stopWorkers() {
  isRunWorkers_ = false;
  odometryBuffer_.close();
  mappingBuffer_.close();

  if (unifiedWorkerOdom_.joinable()) {
    unifiedWorkerOdom_.join();
    std::cout << "Joined unifiedWorkerOdom_ worker \n";
  }

  if (unifiedWorkerMap_.joinable()) {
    unifiedWorkerMap_.join();
    std::cout << "Joined unifiedWorkerMap_ worker \n";
  }

  if (unifiedWorker_.joinable()) {
    unifiedWorker_.join();
    std::cout << "Joined unifiedWorker_ worker \n";
  }
}

// Save Regular Map
bool SlamWrapper::saveMap(const std::string& directory) {
  // Check if directory exists and create if necessary
  if (directory.empty()) {
    std::cout << "\033[36m[saveMap] Provided directory path is empty. Aborting save.\033[0m" << std::endl;
    return false;
  }

  // Create directory if it does not exist
  if (createDirectoryOrNoActionIfExists(directory)) {
    std::cout << "\033[36m[saveMap] Directory '" << directory << "' created or already exists.\033[0m" << std::endl;
  } else {
    std::cout << "\033[36m[saveMap] Failed to create or access directory '" << directory << "'. Aborting save.\033[0m" << std::endl;
    return false;
  }

  // Ensure directory ends with a slash
  std::string dir = directory;
  if (dir.back() != '/' && dir.back() != '\\') {
    dir += '/';
    std::cout << "\033[36m[saveMap] Directory path did not end with a slash. Appending '/'.\033[0m" << std::endl;
  } else {
    std::cout << "\033[36m[saveMap] Directory path ends with a slash. No modification needed.\033[0m" << std::endl;
  }

  // Assemble the map point cloud
  PointCloud map = mapper_->getAssembledMapPointCloud();
  std::cout << "\033[36m[saveMap] Assembled map point cloud. Saving to file...\033[0m" << std::endl;

  // Compose filename
  const std::string filename = dir + "map.pcd";
  std::cout << "\033[36m[saveMap] Saving map to '" << filename << "'\033[0m" << std::endl;

  // Save to file
  bool result = saveToFile(filename, map);
  if (result) {
    std::cout << "\033[36m[saveMap] Map successfully saved to '" << filename << "'\033[0m" << std::endl;
  } else {
    std::cout << "\033[36m[saveMap] Failed to save map to '" << filename << "'\033[0m" << std::endl;
  }
  return result;
}

// Save Dense Maps
bool SlamWrapper::saveDenseSubmaps(const std::string& directory) {
  return saveSubmaps(directory, true);
}

// Map saving API
bool SlamWrapper::saveSubmaps(const std::string& directory, const bool& isDenseMap) {
  createDirectoryOrNoActionIfExists(directory);
  const std::string cloudName = isDenseMap ? "denseSubmap" : "submap";
  const bool savingResult = mapper_->getSubmaps().dumpToFile(directory, cloudName, isDenseMap);
  return savingResult;
}

bool SlamWrapper::isOdometryPoseBufferEmpty() {
  return odometry_->getBuffer().empty();
}

void SlamWrapper::offlineOdometryWorker() {
  if (!odometry_->getBuffer().empty()) {
    const auto latestOdomMeasurement = odometry_->getBuffer().latest_measurement();
    latestScanToScanRegistrationTimestamp_ = latestOdomMeasurement.time_;
  }

  if (odometryBuffer_.empty()) {
    // This is the point cloud measurements, not pose.
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::cerr << "Odometry buffer empty" << std::endl;
    return;
  }

  const TimestampedPointCloud measurement = odometryBuffer_.pop();

  const auto isOdomOkay = odometry_->addRangeScan(measurement.cloud_, measurement.time_);

  if (!isOdomOkay) {
    std::cerr << "WARNING: odometry has failed!!!! \n";
    return;
  }

  mappingBuffer_.push(measurement);

  // This is the limitting factor in odometry publishing, currently limits the odom -> range sensor tf transform publishing to the rate of
  // the lidar.
  const auto latestOdomMeasurement = odometry_->getBuffer().latest_measurement();
  latestScanToScanRegistrationTimestamp_ = latestOdomMeasurement.time_;
  return;
}

void SlamWrapper::unifiedWorkerOdom() {
  try {
    while (isRunWorkers_) {
      if (!odometry_->getBuffer().empty()) {
        auto m = odometry_->getBuffer().latest_measurement(0);
        latestScanToScanRegistrationTimestamp_ = m.time_;
      }

      TimestampedPointCloud meas = odometryBuffer_.wait_and_pop();

      if (!odometry_->addRangeScan(meas.cloud_, meas.time_)) {
        std::cerr << "WARNING: odometry has failed!\n";
        continue;
      }

      mappingBuffer_.push(meas);

      if (!odometry_->getBuffer().empty()) {
        const auto latest = odometry_->getBuffer().latest_measurement();
        latestScanToScanRegistrationTimestamp_ = latest.time_;
      }
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "unifiedWorkerOdom stopped: " << e.what() << '\n';
  }
}

void SlamWrapper::unifiedWorkerMap() {
  try {
    while (isRunWorkers_) {
      TimestampedPointCloud meas = mappingBuffer_.wait_and_pop();

      if (!odometry_->getBuffer().has_query(meas.time_)) {
        std::cout << "unifiedWorkerMap(): Weird, the odom buffer does not seem to have the transform for time " << toSecondsSinceFirstMeasurement(meas.time_) << "!\n";
        std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/" << odometry_->getBuffer().size_limit() << '\n';
        auto& b = odometry_->getBuffer();
        std::cout << "earliest:  " << toSecondsSinceFirstMeasurement(b.earliest_time()) << '\n'
                  << "latest:    " << toSecondsSinceFirstMeasurement(b.latest_time()) << '\n'
                  << "requested: " << toSecondsSinceFirstMeasurement(meas.time_) << '\n';
      }

      bool mappingResult = mapper_->addRangeMeasurement(meas.cloud_, meas.time_);

      if (mappingResult) {
        size_t activeSubmapIdx = mapper_->getActiveSubmap().getId();

        RegisteredPointCloud reg;
        reg.submapId_ = activeSubmapIdx;
        reg.raw_ = meas;
        reg.transform_ = mapper_->getMapToRangeSensor(meas.time_);
        reg.sourceFrame_ = frames_.rangeSensorFrame;
        reg.targetFrame_ = frames_.mapFrame;
        registeredCloudBuffer_.push(reg);

        latestScanToMapRefinementTimestamp_ = meas.time_;

        ScanToMapRegistrationBestGuess guess;
        guess.time_ = meas.time_;
        guess.transform_ = mapper_->getRegistrationBestGuess(meas.time_);
        guess.sourceFrame_ = frames_.rangeSensorFrame;
        guess.targetFrame_ = frames_.mapFrame;
        registrationBestGuessBuffer_.push(guess);
      }
      computeFeaturesIfReady();
      if (params_.mapper_.isAttemptLoopClosures_) {
        attemptLoopClosuresIfReady();
        checkIfOptimizedGraphAvailable();
      }
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "unifiedWorkerMap stopped: " << e.what() << '\n';
  }
}

void SlamWrapper::unifiedWorker() {
  try {
    while (isRunWorkers_) {
      TimestampedPointCloud meas = odometryBuffer_.wait_and_pop();

      // analyzeBufferedMeasurements();

      // 2. Odometry step
      if (!odometry_->addRangeScan(meas.cloud_, meas.time_)) {
        std::cerr << "WARNING: odometry has failed!\n";
        continue;
      }

      // 3. Bookkeeping: update latest odometry timestamp if available
      if (!odometry_->getBuffer().empty()) {
        auto m = odometry_->getBuffer().latest_measurement(0);
        latestScanToScanRegistrationTimestamp_ = m.time_;
      } else {
        std::cerr << "WARNING: odometry buffer is empty when updating latestScanToScanRegistrationTimestamp_!\n";
      }

      // 4. Mapping step (uses the same measurement)
      if (!odometry_->getBuffer().has_query(meas.time_)) {
        std::cout << "unifiedWorker(): Weird, the odom buffer does not seem to have the transform for time " << meas.time_ << "!\n";
        std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/" << odometry_->getBuffer().size_limit() << '\n';
        const auto& b = odometry_->getBuffer();
        std::cout << "earliest:  " << toSecondsSinceFirstMeasurement(b.earliest_time()) << '\n'
                  << "latest:    " << toSecondsSinceFirstMeasurement(b.latest_time()) << '\n'
                  << "requested: " << toSecondsSinceFirstMeasurement(meas.time_) << '\n';
      }

      bool mappingResult = mapper_->addRangeMeasurement(meas.cloud_, meas.time_);

      if (mappingResult) {
        size_t activeSubmapIdx = mapper_->getActiveSubmap().getId();

        RegisteredPointCloud reg;
        reg.submapId_ = activeSubmapIdx;
        reg.raw_ = meas;
        reg.transform_ = mapper_->getMapToRangeSensor(meas.time_);
        reg.sourceFrame_ = frames_.rangeSensorFrame;
        reg.targetFrame_ = frames_.mapFrame;
        registeredCloudBuffer_.push(reg);

        latestScanToMapRefinementTimestamp_ = meas.time_;

        ScanToMapRegistrationBestGuess guess;
        guess.time_ = meas.time_;
        guess.transform_ = mapper_->getRegistrationBestGuess(meas.time_);
        guess.sourceFrame_ = frames_.rangeSensorFrame;
        guess.targetFrame_ = frames_.mapFrame;
        registrationBestGuessBuffer_.push(guess);
      }

      computeFeaturesIfReady();

      if (params_.mapper_.isAttemptLoopClosures_) {
        attemptLoopClosuresIfReady();
        checkIfOptimizedGraphAvailable();
      }
    }
  } catch (const std::runtime_error& e) {
    std::cerr << "unifiedWorker stopped: " << e.what() << '\n';
  }
}
void SlamWrapper::analyzeBufferedMeasurements() {
  std::deque<TimestampedPointCloud> snapshot = odometryBuffer_.snapshot();
  for (const auto& meas : snapshot) {
    // Analysis logic here. Example: print timestamps or sizes.
    std::cout << "Buffered measurement time: " << meas.time_ << '\n';
  }
  std::cout << "Total buffered: " << snapshot.size() << '\n';
}

bool SlamWrapper::doesOdometrybufferHasMeasurement(const Time& t) {
  bool success = odometry_->getBuffer().has_query(t);
  if (!success) {
    std::cout << "doesOdometrybufferHasMeasurement(const Time&): Weird, the odom buffer does not seem to have the transform for time " << t << "!\n";
    std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/" << odometry_->getBuffer().size_limit() << std::endl;
    const auto& b = odometry_->getBuffer();
    std::cout << "earliest: " << toSecondsSinceFirstMeasurement(b.earliest_time()) << std::endl;
    std::cout << "latest: " << toSecondsSinceFirstMeasurement(b.latest_time()) << std::endl;
    std::cout << "requested: " << toSecondsSinceFirstMeasurement(t) << std::endl;
  }

  return success;
}

// Offline Scan2Map Mapping Worker
void SlamWrapper::offlineMappingWorker() {
  if (mappingBuffer_.empty()) {
    return;
  }

  TimestampedPointCloud measurement = mappingBuffer_.pop();

  if (!odometry_->getBuffer().has_query(measurement.time_)) {
    std::cout << "offlineMappingWorker(): Weird, the odom buffer does not seem to have the transform for time " << measurement.time_ << "!\n";
    std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/" << odometry_->getBuffer().size_limit() << std::endl;
    const auto& b = odometry_->getBuffer();
    std::cout << "earliest: " << toSecondsSinceFirstMeasurement(b.earliest_time()) << std::endl;
    std::cout << "latest: " << toSecondsSinceFirstMeasurement(b.latest_time()) << std::endl;
    std::cout << "requested: " << toSecondsSinceFirstMeasurement(measurement.time_) << std::endl;
  }

  mapperOnlyTimer_.startStopwatch();

  // Entry point to the mapper. Also does the registration.
  const bool mappingResult = mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);

  const double timeElapsed = mapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
  mapperOnlyTimer_.addMeasurementMsec(timeElapsed);

  if (mappingResult) {
    // Get the active submap size.
    const size_t activeSubmapIdx = mapper_->getActiveSubmap().getId();

    RegisteredPointCloud registeredCloud;
    registeredCloud.submapId_ = activeSubmapIdx;
    registeredCloud.raw_ = measurement;
    registeredCloud.transform_ = mapper_->getMapToRangeSensor(measurement.time_);
    registeredCloud.sourceFrame_ = frames_.rangeSensorFrame;
    registeredCloud.targetFrame_ = frames_.mapFrame;
    registeredCloudBuffer_.push(registeredCloud);
    latestScanToMapRefinementTimestamp_ = measurement.time_;

    if ((!mapper_->isRegistrationBestGuessBufferEmpty())) {
      ScanToMapRegistrationBestGuess bestGuess;
      bestGuess.time_ = measurement.time_;
      bestGuess.transform_ = mapper_->getRegistrationBestGuess(measurement.time_);
      bestGuess.sourceFrame_ = frames_.rangeSensorFrame;
      bestGuess.targetFrame_ = frames_.mapFrame;
      registrationBestGuessBuffer_.push(bestGuess);
    }
  }

  computeFeaturesIfReady();
  // Compute the loop closure features if needed.
  if (params_.mapper_.isAttemptLoopClosures_) {
    attemptLoopClosuresIfReady();
  }

  checkIfOptimizedGraphAvailable();

  return;
}

void SlamWrapper::offlineLoopClosureWorker() {
  if (loopClosureCandidates_.empty() || isOptimizedGraphAvailable_) {
    return;
  }

  Constraints loopClosureConstraints;
  {
    //			Timer t("loop_closing_attempt");
    const auto lcc = loopClosureCandidates_.popAllElements();
    loopClosureConstraints = submaps_->buildLoopClosureConstraints(lcc);
    numLatesLoopClosureConstraints_ = loopClosureConstraints.size();
  }

  if (loopClosureConstraints.empty()) {
    return;
  }

  {
    Timer t("optimization_problem");
    auto odometryConstraints = submaps_->getOdometryConstraints();
    computeOdometryConstraints(*submaps_, &odometryConstraints);

    //			optimizationProblem_->clearLoopClosureConstraints();
    optimizationProblem_->clearOdometryConstraints();
    optimizationProblem_->insertLoopClosureConstraints(loopClosureConstraints);
    optimizationProblem_->insertOdometryConstraints(odometryConstraints);
    optimizationProblem_->buildOptimizationProblem(*submaps_);

    //			optimizationProblem_->print();
    if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
      submaps_->dumpToFile(folderPath_, "before", false);
      optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
    }

    optimizationProblem_->solve();
    // optimizationProblem_->print();
    lastLoopClosureConstraints_ = loopClosureConstraints;
    isOptimizedGraphAvailable_ = true;
  }
}

void SlamWrapper::checkIfOptimizedGraphAvailable() {
  if (isOptimizedGraphAvailable_) {
    isOptimizedGraphAvailable_ = false;
    const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
    std::cout << "latest pose before update: \n " << asStringXYZRPY(poseBeforeUpdate.transform_) << "\n";
    updateSubmapsAndTrajectory();
    const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
    std::cout << "latest pose after update: \n " << asStringXYZRPY(poseAfterUpdate.transform_) << "\n";
    if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
      submaps_->dumpToFile(folderPath_, "after", false);
    }
  }
}

void SlamWrapper::denseMapWorker() {
  while (isRunWorkers_) {
    if (registeredCloudBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    denseMapStatiscticsTimer_.startStopwatch();

    const RegisteredPointCloud regCloud = registeredCloudBuffer_.pop();

    mapper_->getSubmapsPtr()
        ->getSubmapPtr(regCloud.submapId_)
        ->insertScanDenseMap(regCloud.raw_.cloud_, regCloud.transform_, regCloud.raw_.time_, true);

    const double timeMeasurement = denseMapStatiscticsTimer_.elapsedMsecSinceStopwatchStart();
    denseMapStatiscticsTimer_.addMeasurementMsec(timeMeasurement);
    if (params_.mapper_.isPrintTimingStatistics_ && denseMapStatiscticsTimer_.elapsedSec() > timingStatsEveryNsec) {
      std::cout << "Dense mapping timing stats: Avg execution time: " << denseMapStatiscticsTimer_.getAvgMeasurementMsec()
                << " msec , frequency: " << 1e3 / denseMapStatiscticsTimer_.getAvgMeasurementMsec() << " Hz \n";
      denseMapStatiscticsTimer_.reset();
    }

  }  // end while
}

void SlamWrapper::computeFeaturesIfReady() {
  // std::cout << "Finished number of submaps: " << submaps_->numFinishedSubmaps() << std::endl;
  // std::cout << "Is computing features: " << submaps_->isComputingFeatures() << std::endl;
  if (submaps_->numFinishedSubmaps() > 0 && !submaps_->isComputingFeatures()) {
    computeFeaturesResult_ = std::async(std::launch::async, [this]() {
      const auto finishedSubmapIds = submaps_->popFinishedSubmapIds();
      submaps_->computeFeatures(finishedSubmapIds);
    });
  }
}

void SlamWrapper::attemptLoopClosuresIfReady() {
  const auto timeout = std::chrono::milliseconds(0);
  if (computeFeaturesResult_.valid() && computeFeaturesResult_.wait_for(timeout) == std::future_status::ready) {
    computeFeaturesResult_.get();  // consume the future
    // std::cout << "Number of Loop closure Candidates: " << submaps_->numLoopClosureCandidates() << std::endl;
    if (submaps_->numLoopClosureCandidates() > 0) {
      const auto lcc = submaps_->popLoopClosureCandidates();
      loopClosureCandidates_.insert(lcc.begin(), lcc.end());
    }
  }
}

void SlamWrapper::loopClosureWorker() {
  while (isRunWorkers_) {
    if (loopClosureCandidates_.empty() || isOptimizedGraphAvailable_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    Constraints loopClosureConstraints;
    {
      //			Timer t("loop_closing_attempt");
      const auto lcc = loopClosureCandidates_.popAllElements();
      loopClosureConstraints = submaps_->buildLoopClosureConstraints(lcc);
      numLatesLoopClosureConstraints_ = loopClosureConstraints.size();
    }

    if (loopClosureConstraints.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    {
      Timer t("optimization_problem");
      auto odometryConstraints = submaps_->getOdometryConstraints();
      computeOdometryConstraints(*submaps_, &odometryConstraints);

      //			optimizationProblem_->clearLoopClosureConstraints();
      optimizationProblem_->clearOdometryConstraints();
      optimizationProblem_->insertLoopClosureConstraints(loopClosureConstraints);
      optimizationProblem_->insertOdometryConstraints(odometryConstraints);
      optimizationProblem_->buildOptimizationProblem(*submaps_);

      //			optimizationProblem_->print();
      if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
        submaps_->dumpToFile(folderPath_, "before", false);
        optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
      }
      optimizationProblem_->solve();
      // optimizationProblem_->print();
      lastLoopClosureConstraints_ = loopClosureConstraints;
      isOptimizedGraphAvailable_ = true;
    }

  }  // end while
}

void SlamWrapper::updateSubmapsAndTrajectory() {
  std::cout << "Updating the maps: \n";
  const Timer t("submaps_update");
  const auto optimizedTransformations = optimizationProblem_->getOptimizedTransformIncrements();
  submaps_->transform(optimizedTransformations);

  // get The correct time
  const Constraint latestLoopClosureConstraint =
      *std::max_element(lastLoopClosureConstraints_.begin(), lastLoopClosureConstraints_.end(),
                        [](const Constraint& c1, const Constraint& c2) { return c1.timestamp_ < c2.timestamp_; });

  // loop closing constraints are built such that the source node is always the ne being transformed
  // hence the source node should be the one whose transform we should apply
  assert_gt(latestLoopClosureConstraint.sourceSubmapIdx_, latestLoopClosureConstraint.targetSubmapIdx_,
            "Wrapper ros, update submaps and trajectory: ");
  const auto dT = optimizedTransformations.at(latestLoopClosureConstraint.sourceSubmapIdx_);

  std::cout << "Transforming the pose buffer with the delta T from submap " << latestLoopClosureConstraint.sourceSubmapIdx_
            << " the transform is: \n"
            << asStringXYZRPY(dT.dT_) << std::endl;
  mapper_->loopClosureUpdate(dT.dT_);

  // now here you would update the lc constraints
  Constraints loopClosureConstraints = optimizationProblem_->getLoopClosureConstraints();
  for (int i = 0; i < loopClosureConstraints.size(); ++i) {
    const Constraint& oldConstraint = loopClosureConstraints.at(i);
    Constraint c = oldConstraint;
    c.sourceToTarget_.setIdentity();
    optimizationProblem_->updateLoopClosureConstraint(i, c);
    loopClosureConstraints.at(i) = c;
    //		std::cout << "Loop closure constraint " << i << " new transform: " << asStringXYZRPY(c.sourceToTarget_)
    //				<< std::endl;
  }

  submaps_->updateAdjacencyMatrix(loopClosureConstraints);
}

}  // namespace o3d_slam
