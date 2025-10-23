/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include <open3d/io/PointCloudIO.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include <tf2_msgs/msg/tf_message.hpp>


#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/time.h>

// rosbag2_cpp API
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
// #include <rosbag2_cpp/typesupport_converter_factory.hpp>

#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <memory>


namespace o3d_slam {


RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(
    const rclcpp::Node::SharedPtr& nh)
  : BASE(nh), nh_(nh) {}

void RosbagRangeDataProcessorRos::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr&) {}

void RosbagRangeDataProcessorRos::poseStampedCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr&) {}

void RosbagRangeDataProcessorRos::processRosbagForIMU() {}

void RosbagRangeDataProcessorRos::exportIMUData() {}

bool RosbagRangeDataProcessorRos::run() { return processRosbag(); }


void RosbagRangeDataProcessorRos::initialize() {
    initCommonRosStuff();

    slam_ = std::make_shared<SlamWrapperRos>(nh_);
    slam_->loadParametersAndInitialize();

    rosbagFilename_ = nh_->declare_parameter<std::string>("rosbag_filepath", "");

    RCLCPP_INFO_STREAM(nh_->get_logger(), "Reading from rosbag: " << rosbagFilename_);

  bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions wr_opts;
  wr_opts.uri        = rosbagFilename_;       // directory, no extension
  wr_opts.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions wr_conv;
  wr_conv.input_serialization_format  = "cdr";
  wr_conv.output_serialization_format = "cdr";

  bag_writer_->open(wr_opts, wr_conv);
    
    // Initialize calibration Transform (Identity)
    baseToLidarTransform_.transform.rotation.w = 1.0;
    baseToLidarTransform_.transform.rotation.x = 0.0;
    baseToLidarTransform_.transform.rotation.y = 0.0;
    baseToLidarTransform_.transform.rotation.z = 0.0;
    baseToLidarTransform_.transform.translation.x = 0.0;
    baseToLidarTransform_.transform.translation.y = 0.0;
    baseToLidarTransform_.transform.translation.z = 0.0;

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());

    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_, nh_, false);

    // And default construction, e.g.
    transformBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    staticTransformBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

    reader_ = std::make_unique<rosbag2_cpp::Reader>();


    // Print checklist
    RCLCPP_INFO_STREAM(nh_->get_logger(), "\033[33m Did you check the odometry topic frame? \033[39m");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "\033[33m Did you check if loopclosure enabled? \033[39m");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "\033[33m Is clock available in your bag? \033[39m");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "\033[33m Did you pray this works? \033[39m");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "\033[33m Did you check if tf_static exists in your bag? \033[39m");

    rclcpp::sleep_for(std::chrono::seconds(2));
}

bool RosbagRangeDataProcessorRos::createOutputDirectory() {
    if (std::filesystem::is_directory("log/slam_loggers")) {
        return true;
    }
    try {
        return std::filesystem::create_directories("log/slam_loggers");
    } catch (const std::exception& exception) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Caught an exception trying to create output folder: " << exception.what());
    }
    return false;
}

std::string RosbagRangeDataProcessorRos::buildUpLogFilename(const std::string& typeSuffix, const std::string& extension) {
    // Use rclcpp::Clock for time, or std::chrono for filename uniqueness
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
    std::stringstream ss;
    ss << sec << "_" << nsec;
    std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + extension;  // Or append ss.str() if needed
    return filename;
}

visualization_msgs::msg::Marker RosbagRangeDataProcessorRos::createLineStripMarker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = slam_->frames_.mapFrame;
    marker.header.stamp = nh_->get_clock()->now();
    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.points.reserve(trackedPath_.poses.size());

    for (const auto& pose : trackedPath_.poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.z = pose.pose.position.z;
        marker.points.push_back(point);
    }
    return marker;
}

o3d_slam::PointCloud RosbagRangeDataProcessorRos::lineStripToPointCloud(const visualization_msgs::msg::MarkerArray& marker_array,
                                                                        const int num_samples) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& marker : marker_array.markers) {
        for (size_t j = 0; j + 1 < marker.points.size(); ++j) {
            float dx = (marker.points[j + 1].x - marker.points[j].x) / num_samples;
            float dy = (marker.points[j + 1].y - marker.points[j].y) / num_samples;
            float dz = (marker.points[j + 1].z - marker.points[j].z) / num_samples;
            for (int i = 0; i < num_samples; ++i) {
                Eigen::Vector3d point;
                point.x() = marker.points[j].x + i * dx;
                point.y() = marker.points[j].y + i * dy;
                point.z() = marker.points[j].z + i * dz;
                points.push_back(point);
            }
        }
    }
    o3d_slam::PointCloud cloud(points);
    return cloud;
}

visualization_msgs::msg::MarkerArray RosbagRangeDataProcessorRos::convertPathToMarkerArray(const nav_msgs::msg::Path& path) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(1);
    marker_array.markers[0] = createLineStripMarker();
    return marker_array;
}

void RosbagRangeDataProcessorRos::calculateSurfaceNormals(o3d_slam::PointCloud& cloud) {
    if (cloud.points_.empty()) {
        RCLCPP_ERROR(nh_->get_logger(), "Cloud is empty. Skipping the normal calculation.");
        return;
    }
    open3d::geometry::KDTreeSearchParamHybrid param(5, 20);
    cloud.EstimateNormals(param);
    cloud.NormalizeNormals();
    cloud.OrientNormalsTowardsCameraLocation();
}

void RosbagRangeDataProcessorRos::startProcessing() {
    if (!createOutputDirectory()) {
        std::cout << "Couldn't create the directory\n";
        return;
    }

    if (slam_->exportIMUdata_) {
        // TODO: Implement exportIMUData() in ROS 2 style if needed.
        std::cout << "IMU Exporting is complete\n";
        rclcpp::sleep_for(std::chrono::seconds(10));
        std::cout << "Sleeping 10 seconds..\n";
    }

    std::string trackedPosesFilename_ = buildUpLogFilename("slam_poses");
    std::remove(trackedPosesFilename_.c_str());

    const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";
    poseFile_.open(trackedPosesFilename_, std::ios_base::app);
    poseFile_.precision(std::numeric_limits<double>::max_digits10);
    poseFile_ << poseLogFileHeader_ << std::endl;

    std::string outBagPath_ = buildUpLogFilename("processed_slam", ".db3");
    std::remove(outBagPath_.c_str());
    // NOTE: You need to implement rosbag2_cpp API writing instead of rosbag::Bag.

    if (processRosbag()) {
        auto lineStrip = convertPathToMarkerArray(trackedPath_);
        o3d_slam::PointCloud samplecloud;
        int numPoints = 50;
        float tube_radius = 0.02;
        samplecloud = lineStripToPointCloud(lineStrip, numPoints);
        calculateSurfaceNormals(samplecloud);

        o3d_slam::PointCloud tube_cloud;
        std::vector<Eigen::Vector3d> tube_cloud_normal_vector;
        std::vector<Eigen::Vector3d> tube_cloud_point_vector;
        tube_cloud_normal_vector.reserve(samplecloud.points_.size() * 36);
        tube_cloud_point_vector.reserve(samplecloud.points_.size() * 36);

        for (int i = 0; i < static_cast<int>(samplecloud.points_.size()) - numPoints; i++) {
            Eigen::Vector3d normal = samplecloud.normals_[i].normalized();
            const Eigen::Vector3d direction = normal.unitOrthogonal().cross(normal).normalized();
            const Eigen::Vector3d current_point = samplecloud.points_[i];
            const Eigen::Vector3d current_normal = samplecloud.normals_[i];
            for (float angle = 0; angle <= 360; angle += 10) {
                float x = current_point.x() + tube_radius * (direction.x() * cos(angle) + normal.x() * sin(angle));
                float y = current_point.y() + tube_radius * (direction.y() * cos(angle) + normal.y() * sin(angle));
                float z = current_point.z() + tube_radius * (direction.z() * cos(angle) + normal.z() * sin(angle));
                Eigen::Vector3d tube_point(x, y, z);
                tube_cloud_normal_vector.push_back(current_normal);
                tube_cloud_point_vector.push_back(tube_point);
            }
        }
        tube_cloud.normals_ = tube_cloud_normal_vector;
        tube_cloud.points_ = tube_cloud_point_vector;

        for (int i = 0; i < static_cast<int>(samplecloud.points_.size()) - numPoints; i++) {
            for (int j = 0; j < 36; j++) {
                tube_cloud.normals_[i * 36 + j] = samplecloud.normals_[i];
            }
        }

        std::string nameWithCorrectSuffix = slam_->mapSavingFolderPath_ + "robotPathAsMesh.pcd";
        open3d::io::WritePointCloudToPCD(nameWithCorrectSuffix, tube_cloud, open3d::io::WritePointCloudOption());
        RCLCPP_INFO(nh_->get_logger(), "Successfully saved the poses as point cloud. Waiting for user to terminate.");
    }

    poseFile_.close();
    // Close rosbag2_writer here if you opened one.
    slam_->stopWorkers();
}

bool RosbagRangeDataProcessorRos::validateTopicsInRosbag(const std::string& bagfile_path, const std::vector<std::string>& mandatoryTopics) {
    rosbag2_cpp::Info info;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bagfile_path;
    storage_options.storage_id = "sqlite3";
    auto metadata = info.read_metadata(storage_options.uri, storage_options.storage_id);

    std::set<std::string> topics_in_bag;
    for (const auto& topic_metadata : metadata.topics_with_message_count) {
        topics_in_bag.insert(topic_metadata.topic_metadata.name);
    }

    bool all_found = true;
    for (const auto& topic : mandatoryTopics) {
        if (topics_in_bag.count(topic) == 0) {
            RCLCPP_ERROR(nh_->get_logger(), "Mandatory topic %s not found in rosbag2.", topic.c_str());
            all_found = false;
        }
    }
    if (!all_found) {
        RCLCPP_ERROR(nh_->get_logger(), "Not all required topics are present. Terminating replay.");
    }
    return all_found;
}


void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  /*bool success = slam_->addRangeScan(cloud, timestamp);
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = std::get<0>(cloudTimePair).IsEmpty();
  if (isTimeValid(std::get<1>(cloudTimePair)) && !isCloudEmpty) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)), rawCloudPub_);
  }
  */
}

#include <filesystem>
#include <fstream>
#include <iomanip>

void RosbagRangeDataProcessorRos::logTiming(const rclcpp::Duration& duration, const std::string& filepath_base) {
  namespace fs = std::filesystem;
  static std::string currentFilePath;
  static bool isInitialized = false;

  // On first use, determine final file path
  if (isFirstWrite) {
    fs::path basePath(filepath_base);
    fs::path dir = basePath.parent_path();
    std::string stem = basePath.stem().string();      // filename without extension
    std::string ext = basePath.extension().string();  // e.g. ".txt"

    if (!fs::exists(dir)) {
      try {
        fs::create_directories(dir);
      } catch (const fs::filesystem_error& e) {
        std::cerr << "Failed to create directory: " << e.what() << std::endl;
        return;
      }
    }

    // If file exists, create a new one with incremented suffix
    fs::path finalPath = basePath;
    int index = 1;
    while (fs::exists(finalPath)) {
      finalPath = dir / fs::path(stem + "_" + std::to_string(index) + ext);
      ++index;
    }

    currentFilePath = finalPath.string();
    isFirstWrite = false;

    // First write: truncate/overwrite
    std::ofstream firstWrite(currentFilePath, std::ios::trunc);
    if (!firstWrite.is_open()) {
      std::cerr << "Failed to open log file: " << currentFilePath << std::endl;
      return;
    }
    // firstWrite << "Log Start\n";
    firstWrite.close();
  }

  // Append measurement
  std::ofstream out(currentFilePath, std::ios::app);
  if (!out.is_open()) {
    std::cerr << "Failed to open log file for appending: " << currentFilePath << std::endl;
    return;
  }

  out << std::fixed << std::setprecision(3)
      << duration.seconds() * 1000.0 << "\n";
}

bool RosbagRangeDataProcessorRos::processBuffers(SlamInputsBuffer& buffer) {
    if (buffer.empty()) {
        RCLCPP_DEBUG(nh_->get_logger(), "Empty buffer");
        return false;
    }

    if (slam_->useSyncedPoses_) {
        auto& odometryPose = buffer.front()->odometryPose_;
        geometry_msgs::msg::Pose odomPose = odometryPose->pose;

        if (isFirstMessage_) {
            Eigen::Isometry3d eigenTransform = Eigen::Isometry3d::Identity();
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
        }

        if (isFirstMessage_ && isStaticTransformFound_) {
            geometry_msgs::msg::Pose initialPose;
            initialPose.position = odomPose.position;
            initialPose.orientation.w = 1.0;
            initialPose.orientation.z = 0.0;
            initialPose.orientation.y = 0.0;
            initialPose.orientation.x = 0.0;
            slam_->setInitialTransform(o3d_slam::getTransform(initialPose).matrix());
            isFirstMessage_ = false;
        }

        if (!slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose), fromRos2(odometryPose->header.stamp))) {
            RCLCPP_ERROR(nh_->get_logger(), "Couldn't Add pose to buffer");
            return false;
        }

    } else {
        if (slam_->isOdometryPoseBufferEmpty()) {
            RCLCPP_ERROR(nh_->get_logger(), "Odometry Buffer is empty!");
            return false;
        }

        if (!slam_->isInitialTransformSet()) {
            RCLCPP_ERROR(nh_->get_logger(), "Initial transform not set yet! Popping the measurement.");
            return false;
        }
    }

    auto& pointCloud = buffer.front()->pointCloud_;

    open3d::geometry::PointCloud cloud;
    if (!open3d_conversions::rosToOpen3d(pointCloud, cloud, false, true)) {
        RCLCPP_ERROR(nh_->get_logger(), "Couldn't convert the point cloud");
        return false;
    }

    const Time timestamp = fromRos2(pointCloud->header.stamp);

    if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
        buffer.pop_front();
        return false;
    }

    if (slam_->addRangeScan(cloud, timestamp)) {
        auto timeTuple = usePairForRegistration();
        // logTiming() as needed, convert to rclcpp types
    } else {
        buffer.pop_front();
        return false;
    }

    slam_->offlineTfWorker();
    slam_->offlineVisualizationWorker();

    std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
    Transform calculatedTransform = std::get<2>(cloudTimePair);

    std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();
    Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

    geometry_msgs::msg::PoseStamped bestGuessPoseStamped;
    Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

    bestGuessPoseStamped.header.stamp = toRos2(std::get<0>(bestGuessTimePair));
    bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
    bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
    bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
    bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
    bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
    bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
    bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();
    bestGuessPath_.poses.push_back(bestGuessPoseStamped);
    bestGuessPath_.header.stamp = toRos2(std::get<0>(bestGuessTimePair));
    bestGuessPath_.header.frame_id = slam_->frames_.mapFrame;

    if (offlineBestGuessPathPub_->get_subscription_count() > 0) {
        offlineBestGuessPathPub_->publish(bestGuessPath_);
    }

    geometry_msgs::msg::PoseStamped poseStamped;
    Eigen::Quaterniond rotation(calculatedTransform.rotation());

    poseStamped.header.stamp = toRos2(std::get<1>(cloudTimePair));
    poseStamped.pose.position.x = calculatedTransform.translation().x();
    poseStamped.pose.position.y = calculatedTransform.translation().y();
    poseStamped.pose.position.z = calculatedTransform.translation().z();
    poseStamped.pose.orientation.w = rotation.w();
    poseStamped.pose.orientation.x = rotation.x();
    poseStamped.pose.orientation.y = rotation.y();
    poseStamped.pose.orientation.z = rotation.z();
    trackedPath_.poses.push_back(poseStamped);

    // Replace file writes and outBag.write with rosbag2_cpp::Writer (see below)

    trackedPath_.header.stamp = toRos2(std::get<1>(cloudTimePair));
    trackedPath_.header.frame_id = slam_->frames_.mapFrame;

    if (offlinePathPub_->get_subscription_count() > 0) {
        offlinePathPub_->publish(trackedPath_);
    }


    drawLinesBetweenPoses(trackedPath_, bestGuessPath_,
                          toRclcpp(std::get<1>(cloudTimePair)));

    if (isTimeValid(std::get<1>(cloudTimePair)) && !(std::get<0>(cloudTimePair).IsEmpty())) {
        o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos2(std::get<1>(cloudTimePair)), registeredCloudPub_);
        rclcpp::spin_some(nh_);
    } else {
        RCLCPP_ERROR(nh_->get_logger(), "Cloud is empty or time is invalid. Skipping the cloud.");
        return false;
    }

    if (surfaceNormalPub_->get_subscription_count() > 0) {
        auto surfaceNormalLineMarker{
            generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos2(std::get<1>(cloudTimePair)), colorMap_[ColorKey::kRed])};
        if (surfaceNormalLineMarker.has_value()) {
            surfaceNormalPub_->publish(surfaceNormalLineMarker.value());
        }
    }

    buffer.pop_front();

    sensor_msgs::msg::PointCloud2 outCloud;
    open3d_conversions::open3dToRos(std::get<0>(cloudTimePair), outCloud, slam_->frames_.rangeSensorFrame);
    outCloud.header.stamp = toRos2(std::get<1>(cloudTimePair));
    // Write to rosbag2 using rosbag2_cpp::Writer (see below)

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = poseStamped.header.stamp;
    transformStamped.header.frame_id = slam_->frames_.mapFrame;
    transformStamped.child_frame_id = slam_->frames_.rangeSensorFrame;
    transformStamped.transform.translation.x = poseStamped.pose.position.x;
    transformStamped.transform.translation.y = poseStamped.pose.position.y;
    transformStamped.transform.translation.z = poseStamped.pose.position.z;
    transformStamped.transform.rotation = poseStamped.pose.orientation;

    tf2_msgs::msg::TFMessage tfMessage;
    tfMessage.transforms.push_back(transformStamped);

    // Write tfMessage to rosbag2 (see below)

    PointCloud& transformedCloud = std::get<0>(cloudTimePair);
    transformedCloud.Transform(std::get<2>(cloudTimePair).matrix());
    sensor_msgs::msg::PointCloud2 transformedRosCloud;
    open3d_conversions::open3dToRos(transformedCloud, transformedRosCloud, slam_->frames_.rangeSensorFrame);
    transformedRosCloud.header.stamp = toRos2(std::get<1>(cloudTimePair));
    // Write to rosbag2

rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(slam_->relativeSleepDuration_)));

    return true;
}

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

std::optional<visualization_msgs::msg::Marker> RosbagRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
    const open3d::geometry::PointCloud& pointCloud,
    const rclcpp::Time& timestamp,
    const o3d_slam::RgbaColorMap::Values& color) {
  if (pointCloud.IsEmpty()) {
    RCLCPP_WARN(nh_->get_logger(), "Point cloud is empty.");
    return {};
  }
  if (!pointCloud.HasNormals()) {
    RCLCPP_WARN(nh_->get_logger(), "Point cloud has no normals");
    return {};
  }

  std_msgs::msg::ColorRGBA colorMsg;
  colorMsg.r = color[0];
  colorMsg.g = color[1];
  colorMsg.b = color[2];
  colorMsg.a = color[3];

  visualization_msgs::msg::Marker vectorsMarker;
  vectorsMarker.header.stamp = timestamp;
  vectorsMarker.header.frame_id = slam_->frames_.rangeSensorFrame;
  vectorsMarker.ns = "surface_normals";
  vectorsMarker.action = visualization_msgs::msg::Marker::ADD;
  vectorsMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
  vectorsMarker.pose.orientation.w = 1.0;
  vectorsMarker.id = 0;
  vectorsMarker.scale.x = 0.02;
  vectorsMarker.color = colorMsg;
  vectorsMarker.points.resize(pointCloud.points_.size() * 2);

  const auto& surfaceNormalsView = pointCloud.normals_;
  for (size_t i = 0; i < pointCloud.points_.size(); i += 2) {
    // Start point
    vectorsMarker.points[i].x = pointCloud.points_[i][0];
    vectorsMarker.points[i].y = pointCloud.points_[i][1];
    vectorsMarker.points[i].z = pointCloud.points_[i][2];
    // End of arrow
    vectorsMarker.points[i + 1].x = pointCloud.points_[i][0] + surfaceNormalsView[i][0] * 0.09;
    vectorsMarker.points[i + 1].y = pointCloud.points_[i][1] + surfaceNormalsView[i][1] * 0.09;
    vectorsMarker.points[i + 1].z = pointCloud.points_[i][2] + surfaceNormalsView[i][2] * 0.09;
  }

  return vectorsMarker;
}

void RosbagRangeDataProcessorRos::drawLinesBetweenPoses(
    const nav_msgs::msg::Path& path1,
    const nav_msgs::msg::Path& path2,
    const rclcpp::Time& stamp) {
  if (offlineDifferenceLinePub_->get_subscription_count() == 0) {
      return;
  }


  if (path1.poses.size() != path2.poses.size()) {
    RCLCPP_ERROR(nh_->get_logger(), "Path sizes are not equal. Skipping the line drawing.");
    return;
  }

  visualization_msgs::msg::Marker line_list;
  line_list.header.frame_id = slam_->frames_.mapFrame;
  line_list.header.stamp = stamp;
  line_list.ns = "paths";
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list.scale.x = 0.008;  // Line width

  // Cyan lines
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  for (size_t i = 0; i < path1.poses.size(); i++) {
    geometry_msgs::msg::Point p_start;
    p_start.x = path1.poses[i].pose.position.x;
    p_start.y = path1.poses[i].pose.position.y;
    p_start.z = path1.poses[i].pose.position.z;
    line_list.points.push_back(p_start);

    geometry_msgs::msg::Point p_end;
    p_end.x = path2.poses[i].pose.position.x;
    p_end.y = path2.poses[i].pose.position.y;
    p_end.z = path2.poses[i].pose.position.z;
    line_list.points.push_back(p_end);
  }

  offlineDifferenceLinePub_->publish(line_list);
}

std::tuple<rclcpp::Duration, rclcpp::Duration, rclcpp::Duration> RosbagRangeDataProcessorRos::usePairForRegistration() {
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  const auto odometryProcessingStartTime = steady_clock.now();
  slam_->callofflineOdometryWorker();
  const auto odometryProcessingElapsed = steady_clock.now() - odometryProcessingStartTime;

  const auto mappingProcessingStartTime = steady_clock.now();
  slam_->callofflineMappingWorker();
  const auto mappingProcessingElapsed = steady_clock.now() - mappingProcessingStartTime;

  RCLCPP_INFO_STREAM(nh_->get_logger(),
      "Mapping Operations took " << mappingProcessingElapsed.seconds() * 1000.0 << " ms");

  const auto loopclosureProcessingStartTime = steady_clock.now();
  slam_->callofflineLoopClosureWorker();
  const auto loopclosureProcessingElapsed = steady_clock.now() - loopclosureProcessingStartTime;

  return std::make_tuple(odometryProcessingElapsed, mappingProcessingElapsed, loopclosureProcessingElapsed);
}


bool RosbagRangeDataProcessorRos::readCalibrationIfNeeded() {
  if (slam_->useSyncedPoses_) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    baseToLidarTransform_ = tf2::eigenToTransform(transform);
    isStaticTransformFound_ = true;
    return true;
  }

  if (!isStaticTransformFound_ &&
      (slam_->frames_.rangeSensorFrame != slam_->frames_.assumed_external_odometry_tracked_frame)) {
    try {
      auto transformation = tfBuffer_->lookupTransform(
          slam_->frames_.rangeSensorFrame,
          slam_->frames_.assumed_external_odometry_tracked_frame,
          rclcpp::Time(0), rclcpp::Duration(0, 0)); // Get the latest available transform immediately

      RCLCPP_INFO_STREAM(nh_->get_logger(),
          "\033[92mFound the transform between " << slam_->frames_.rangeSensorFrame << " and "
          << slam_->frames_.assumed_external_odometry_tracked_frame << "\033[0m");

      baseToLidarTransform_ = transformation;
      isStaticTransformFound_ = true;

    } catch (const tf2::TransformException& exception) {
      RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 200,
          "Caught exception while looking for the transform Fingers Crossed it will appear soon: " << exception.what());
      return true;
    }
    return true;
  } else {
    isStaticTransformFound_ = true;
    return true;
  }
}

bool RosbagRangeDataProcessorRos::processRosbag() {
  std::vector<std::string> topics;
  topics.push_back(clockTopic_);
  topics.push_back(cloudTopic_);
  topics.push_back(tfStaticTopic_);
  if (slam_->isUsingOdometryTopic()) {
    if (slam_->useSyncedPoses_) {
      topics.push_back(poseStampedTopic_);
    } else {
      topics.push_back(slam_->asyncOdometryTopic_);
    }
  }
  if (slam_->rePublishTf_) {
    RCLCPP_INFO(nh_->get_logger(),
      "\033[33m So you like living risky? Tf will be republished. It is YOUR responsibility to ensure there is no frame duplication.\033[39m");
    topics.push_back(tfTopic_);
  }

  // Open bag with rosbag2_cpp
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbagFilename_;  // directory to bag file (NO extension)
  storage_options.storage_id = "sqlite3";
  ::rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format  = "cdr";
  converter_options.output_serialization_format = "cdr";

  // rosbag2_cpp::Reader reader;
  try {
    reader_->open(storage_options, converter_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Error opening rosbag2: " << e.what());
    return false;
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(), "rosbag2 '" << rosbagFilename_ << "' open.");

  // Map topic to type
  std::map<std::string, std::string> topic_type_map;
  for (const auto &info : reader_->get_all_topics_and_types()) {
    topic_type_map[info.name] = info.type;
  }

  // Processing
  SlamInputsBuffer slamInputsBuffer;
  std::unique_ptr<SlamInputs> slamInputs;
  bool isBagReadyToPlay_ = true;
  rclcpp::Time stampLastIteration(0, 0, RCL_ROS_TIME);
  rclcpp::Time tracker(0, 0, RCL_ROS_TIME);
  rclcpp::Duration timeDiff_(0, 0);
  rclcpp::Time bag_begin_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time bag_end_time(0, 0, RCL_ROS_TIME);

  // Get bag start/end times
  {
    const auto & metadata = reader_->get_metadata();
    using namespace std::chrono;

    // convert chrono::time_point -> int64 nanoseconds since epoch
    const int64_t begin_ns =
      duration_cast<nanoseconds>(metadata.starting_time.time_since_epoch()).count();

    // convert chrono::duration   -> int64 nanoseconds
    const int64_t dur_ns =
      duration_cast<nanoseconds>(metadata.duration).count();

    bag_begin_time = rclcpp::Time(begin_ns, RCL_ROS_TIME);
    bag_end_time   = rclcpp::Time(begin_ns + dur_ns, RCL_ROS_TIME);
  }
  Timer rosbagTimer;

  while (reader_->has_next()) {
    auto bag_message = reader_->read_next();
    if (std::find(topics.begin(), topics.end(), bag_message->topic_name) == topics.end())
      continue;

    if (!slamInputs) {
      slamInputs = std::make_unique<SlamInputs>();
    }
    bool isInvalidMessageInBag = false;
    const std::string &topic = bag_message->topic_name;
    const std::string &msg_type = topic_type_map[topic];
    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

    // Clock
    if (topic == clockTopic_) {
      if (msg_type == "rosgraph_msgs/msg/Clock") {
        rosgraph_msgs::msg::Clock clock_msg;
        rclcpp::Serialization<rosgraph_msgs::msg::Clock> serializer;
        serializer.deserialize_message(&serialized_msg, &clock_msg);
        // (publish clock_msg if needed)
        tracker = clock_msg.clock;
      } else if (msg_type == "nav_msgs/msg/Odometry" || msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        // Not implemented: Provide custom time assignment
        RCLCPP_WARN(nh_->get_logger(), "Using odometry as clock is not supported in this code.");
        return false;
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "This type of clock alternative is not supported");
        return false;
      }

      // Calibration if needed (stub, implement as needed)
      if (!readCalibrationIfNeeded()) {
        RCLCPP_ERROR(nh_->get_logger(), "Calibration failed to read. Exiting.");
        return false;
      }

      // Align the clock and the bag time.
      if (timeDiff_.nanoseconds() == 0) {
        timeDiff_ = tracker - bag_begin_time;
        if (timeDiff_.nanoseconds() < 0) {
          timeDiff_ = bag_begin_time - tracker;
        }
        RCLCPP_INFO_STREAM(nh_->get_logger(),
          "The calculated time difference between the clock and bag is: " << timeDiff_.nanoseconds() / 1000000 << "ms");
      }

      // Skip processes based on required bag start time.
      if ((std::abs((tracker + timeDiff_ - bag_begin_time).seconds()) <= slam_->bagReplayStartTime_) &&
          (slam_->bagReplayStartTime_ != 0.0)) {
        RCLCPP_INFO_STREAM(nh_->get_logger(),
          "Rosbag starting: " << std::abs((tracker + timeDiff_ - bag_begin_time).seconds()) << " / "
                              << slam_->bagReplayStartTime_ << " s");
        isBagReadyToPlay_ = false;
        continue;
      } else {
        isBagReadyToPlay_ = true;
      }

      // Buffer processing
      if (processBuffers(slamInputsBuffer)) {
        // Progress report
        RCLCPP_INFO_STREAM(nh_->get_logger(),
          "Replay run time: " << (tracker + timeDiff_ - bag_begin_time).seconds() << "s / "
                              << (bag_end_time - bag_begin_time).seconds() << " s");
        if ((tracker + timeDiff_ - bag_begin_time).seconds() >= slam_->bagReplayEndTime_) {
          break;
        }
      }
    }

    // Tf static
    else if (topic == tfStaticTopic_ && msg_type == "tf2_msgs/msg/TFMessage") {
      tf2_msgs::msg::TFMessage tf_msg;
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
      serializer.deserialize_message(&serialized_msg, &tf_msg);
      // staticTransformBroadcaster_.sendTransform(tf_msg.transforms);
    }

    // Tf
    else if (topic == tfTopic_ && msg_type == "tf2_msgs/msg/TFMessage") {
      tf2_msgs::msg::TFMessage tf_msg;
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
      serializer.deserialize_message(&serialized_msg, &tf_msg);
      // transformBroadcaster_.sendTransform(tf_msg.transforms);
    }

    if (!isBagReadyToPlay_) continue;

    // Pointcloud
    if (topic == cloudTopic_ && msg_type == "sensor_msgs/msg/PointCloud2") {
      sensor_msgs::msg::PointCloud2 pc_msg;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
      serializer.deserialize_message(&serialized_msg, &pc_msg);
      // Add point cloud to buffer, handle frame logic, etc.
      slamInputs->pointCloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(pc_msg);
      if (slamInputs->pointCloud_) {
        slam_->frames_.rangeSensorFrame = slamInputs->pointCloud_->header.frame_id;
      }
      if (!slam_->useSyncedPoses_) {
        if (slamInputs && slamInputs->pointCloud_) {
          slamInputsBuffer.emplace_back(std::move(slamInputs));
        }
      }
    }

    // Synced odometry pose
    if (slam_->useSyncedPoses_ && topic == poseStampedTopic_ && msg_type == "geometry_msgs/msg/PoseStamped") {
      geometry_msgs::msg::PoseStamped pose_msg;
      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
      serializer.deserialize_message(&serialized_msg, &pose_msg);
      slamInputs->odometryPose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_msg);
    }

    // Async odometry
    if (!slam_->useSyncedPoses_ && topic == slam_->asyncOdometryTopic_) {
      if (msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        geometry_msgs::msg::PoseWithCovarianceStamped odom_msg;
        rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serializer;
        serializer.deserialize_message(&serialized_msg, &odom_msg);
        // Process as in your logic...
        // If needed, convert, transform, add to buffer
      }
      else if (msg_type == "nav_msgs/msg/Odometry") {
        nav_msgs::msg::Odometry odom_msg;
        rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
        serializer.deserialize_message(&serialized_msg, &odom_msg);
        // Process as in your logic...
      }
      else {
        RCLCPP_WARN(nh_->get_logger(), "This msg type is not supported yet. Exiting.");
        return false;
      }
    }

    if (slam_->isUsingOdometryTopic() && slam_->useSyncedPoses_) {
      if (slamInputs && slamInputs->pointCloud_ && slamInputs->odometryPose_ &&
          (slamInputs->pointCloud_->header.stamp == slamInputs->odometryPose_->header.stamp)) {
        slamInputsBuffer.emplace_back(std::move(slamInputs));
      }
    }
    // Insert sleep/processing rate handling as needed (see original)
  }

  processBuffers(slamInputsBuffer);
  RCLCPP_INFO(nh_->get_logger(), "Finished running through the bag.");
  std::cout << "Rosbag processing finished. Rosbag duration: "
            << (bag_end_time - bag_begin_time).seconds()
            << " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
  slam_->offlineFinishProcessing();
  return true;
}

}  // namespace o3d_slam
