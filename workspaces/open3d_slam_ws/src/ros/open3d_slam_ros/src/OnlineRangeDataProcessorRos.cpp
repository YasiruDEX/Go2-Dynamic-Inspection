#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr nh)
    : BASE(nh), nh_(nh), tfBuffer_(nh->get_clock()), tfListener_(tfBuffer_) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  slam_->loadParametersAndInitialize();
}

bool OnlineRangeDataProcessorRos::readCalibrationIfNeeded() {
  static rclcpp::Time last_warning_time = nh_->now();
  static const rclcpp::Duration warning_interval(2, 0); // 2 seconds

  if (slam_->frames_.rangeSensorFrame == "default") {
    auto now = nh_->now();
    if ((now - last_warning_time) > warning_interval) {
      RCLCPP_WARN(nh_->get_logger(),
          "Range sensor frame is not set yet (cloud didn't arrive yet). Still waiting for the transformation...");
      last_warning_time = now;
    }
    return false;
  }

  if ((slam_->frames_.rangeSensorFrame == slam_->frames_.assumed_external_odometry_tracked_frame) ||
      !slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    return true;
  }

  if (!isStaticTransformFound_) {
    try {
      tf2::Duration timeout = tf2::durationFromSec(0.2);

      if (!tfBuffer_.canTransform(
              slam_->frames_.rangeSensorFrame,
              slam_->frames_.assumed_external_odometry_tracked_frame,
              tf2::TimePointZero,
              timeout)) {
        auto now = nh_->now();
        if ((now - last_warning_time) > warning_interval) {
          RCLCPP_WARN(nh_->get_logger(),
              "Still waiting for transform: [%s] to [%s]",
              slam_->frames_.rangeSensorFrame.c_str(),
              slam_->frames_.assumed_external_odometry_tracked_frame.c_str());
          last_warning_time = now;
        }
        return false;
      }

      auto T_L_sensorFrame = tfBuffer_.lookupTransform(
          slam_->frames_.rangeSensorFrame,
          slam_->frames_.assumed_external_odometry_tracked_frame,
          tf2::TimePointZero);

      RCLCPP_INFO(nh_->get_logger(), "Found the transform between %s and %s",
                  slam_->frames_.rangeSensorFrame.c_str(),
                  slam_->frames_.assumed_external_odometry_tracked_frame.c_str());

      slam_->setExternalOdometryFrameToCloudFrameCalibration(tf2::transformToEigen(T_L_sensorFrame));

      if (slam_->isIMUattitudeInitializationEnabled()) {
        if (!tfBuffer_.canTransform(
                slam_->frames_.rangeSensorFrame,
                slam_->frames_.imuFrame,
                tf2::TimePointZero,
                timeout)) {
          auto now = nh_->now();
          if ((now - last_warning_time) > warning_interval) {
            RCLCPP_WARN(nh_->get_logger(),
                "Still waiting for transform: [%s] to [%s]",
                slam_->frames_.rangeSensorFrame.c_str(), slam_->frames_.imuFrame.c_str());
            last_warning_time = now;
          }
          return false;
        }

        auto RangeSensorFrameToimuFrame =
            tfBuffer_.lookupTransform(
                slam_->frames_.rangeSensorFrame,
                slam_->frames_.imuFrame,
                tf2::TimePointZero);

        RCLCPP_INFO(nh_->get_logger(), "Found the transform between %s and %s",
                    slam_->frames_.rangeSensorFrame.c_str(), slam_->frames_.imuFrame.c_str());

        lidarToImu_.matrix() = tf2::transformToEigen(RangeSensorFrameToimuFrame).matrix();
      }
      return true;

    } catch (const tf2::TransformException& exception) {
      auto now = nh_->now();
      if ((now - last_warning_time) > warning_interval) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Caught exception while looking for the transform frame: %s to %s. %s",
                    slam_->frames_.rangeSensorFrame.c_str(),
                    slam_->frames_.assumed_external_odometry_tracked_frame.c_str(),
                    exception.what());
        last_warning_time = now;
      }
      return false;
    }
  } else {
    RCLCPP_WARN(nh_->get_logger(), "This is unexpected, something is off.");
    return false;
  }
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
if (!slam_->isUseExistingMapEnabled() && slam_->isUsingOdometryTopic()) {
  if (!slam_->isInitialTransformSet()) {
    auto cursedLambda = [this]() {
      RCLCPP_WARN_THROTTLE(
          nh_->get_logger(),
          *(nh_->get_clock()),
          1000,
          "Initial Transform not set yet, skipping the measurement. Throttled 1s" // Message
      );
    };
    return;
  }
}

  // Add the range scan to the pointcloud processing buffer. This is actually a buffer with size 1, so no queue.
  // The add range scan comes first since scan2scan odometry would create its own odometry measurements.
  if (!slam_->addRangeScan(cloud, timestamp)) {
    RCLCPP_WARN(nh_->get_logger(), "Failed to add range scan. This is unexpected. Skipping the measurement.");
    return;
  }

  if (slam_->isOdometryPoseBufferEmpty()) {
    RCLCPP_WARN(nh_->get_logger(), "Odometry Buffer is empty! But a point cloud has arrived and waiting to be processed. Skipping this cloud.");
    return;
  }

  if (slam_->isUsingOdometryTopic()) {
    if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
      RCLCPP_WARN(nh_->get_logger(),
          "Pointcloud is here, pose buffer is not empty but odometry with the right stamp not available yet. Skipping the measurement.");
      return;
    }
  }

  // TODO(TT) Is this the best place to do this? (ofc its not)
  // Get the latest registered point cloud and publish it.
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();

  if (std::get<0>(cloudTimePair).IsEmpty()) {
    RCLCPP_WARN(nh_->get_logger(), "Registered Cloud will not be published. Registration didn't take place yet.");
    return;
  }

  if (isTimeValid(std::get<1>(cloudTimePair))) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)),
                           registeredCloudPub_);

    // Transform the registered cloud to the map frame using the calculated transform
    auto mapTransform = std::get<2>(cloudTimePair);
    auto transformedCloud = std::get<0>(cloudTimePair);
    transformedCloud.Transform(mapTransform.matrix());

    // Publish the transformed cloud in the map frame
    o3d_slam::publishCloud(
      transformedCloud,
      slam_->frames_.mapFrame,
      toRos(std::get<1>(cloudTimePair)),
      alreadyTransformedCloudPub_);

    if (surfaceNormalPub_->get_subscription_count() > 0u) {
      auto surfaceNormalLineMarker{
          generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(std::get<1>(cloudTimePair)), colorMap_[ColorKey::kRed])};

      if (surfaceNormalLineMarker != std::nullopt) {
        surfaceNormalPub_->publish(surfaceNormalLineMarker.value());
      }
    }

  } else {
    RCLCPP_WARN(nh_->get_logger(), "Registered Cloud will be published with original stamp. Should only happen at start-up.");
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(timestamp), registeredCloudPub_);

    if (surfaceNormalPub_->get_subscription_count() > 0u) {
      auto surfaceNormalLineMarker{
          generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(timestamp), colorMap_[ColorKey::kRed])};

      if (surfaceNormalLineMarker != std::nullopt) {
        surfaceNormalPub_->publish(surfaceNormalLineMarker.value());
      }
    }

    return;
  }

  if ((!isTimeValid(std::get<1>(cloudTimePair)))) {
    RCLCPP_WARN(nh_->get_logger(), "Transform Time is not valid at processMeasurement level.");
    return;
  }

  // TODO [TT]
  // Functionize this stuff.
  Transform calculatedTransform = std::get<2>(cloudTimePair);

  geometry_msgs::msg::PoseStamped poseStamped;
  Eigen::Quaterniond rotation(calculatedTransform.rotation());

  poseStamped.header.stamp = toRos(std::get<1>(cloudTimePair));
  poseStamped.header.frame_id = "map_o3d";
  poseStamped.pose.position.x = calculatedTransform.translation().x();
  poseStamped.pose.position.y = calculatedTransform.translation().y();
  poseStamped.pose.position.z = calculatedTransform.translation().z();
  poseStamped.pose.orientation.w = rotation.w();
  poseStamped.pose.orientation.x = rotation.x();
  poseStamped.pose.orientation.y = rotation.y();
  poseStamped.pose.orientation.z = rotation.z();

  slam_->appendPoseToTrackedPath(poseStamped);

  std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();

  if ((!isTimeValid(std::get<0>(bestGuessTimePair)))) {
    RCLCPP_WARN(nh_->get_logger(), "bestGuessTimePair Transform Time is not valid at processMeasurement level.");
    return;
  }

  // Best guess path
  Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

  geometry_msgs::msg::PoseStamped bestGuessPoseStamped;
  Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

  // Until we identify the time issue with best guess use cloud time. These are supposed to be same since they are paired.
  bestGuessPoseStamped.header.stamp = toRos(std::get<0>(bestGuessTimePair));
  bestGuessPoseStamped.header.frame_id = "map_o3d";
  bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
  bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
  bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
  bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
  bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
  bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
  bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();

  slam_->appendPoseToBestGuessPath(bestGuessPoseStamped);
  return;
}

std::optional<visualization_msgs::msg::Marker> OnlineRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
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

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  // Main cloud subscriber
  // cloudSubscriber_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     cloudTopic_, rclcpp::SensorDataQoS(),
  //     std::bind(&OnlineRangeDataProcessorRos::cloudCallback, this, std::placeholders::_1));

  // ---- Point cloud subscriber (10 Hz, lossless, robust, reliable, big buffer) ----
  rclcpp::QoS cloud_qos(1);                // 5 seconds buffer at 10 Hz (increase if callback can stall longer)
  cloud_qos.reliable();                     // Guarantee delivery
  // // Optionally: 
  // cloud_qos.deadline(std::chrono::milliseconds(120)); // Detect 10 Hz publisher stalls

  // rclcpp::QoS cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1))
  //     .best_effort()           // Lower latency than reliable; use reliable only if loss is unacceptable
  //     .durability_volatile();  // Default, do not persist


  cloudSubscriber_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloudTopic_,
      cloud_qos,
      std::bind(&OnlineRangeDataProcessorRos::cloudCallback, this, std::placeholders::_1)
  );

  staticTfCallback_ = nh_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OnlineRangeDataProcessorRos::staticTfCallback, this));

  // poseStampedCovarianceSubscriber_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //     poseStampedWithCovarianceTopic_,
  //     40,
  //     std::bind(&OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback, this, std::placeholders::_1));

  // rclcpp::QoS pose_qos(400);                // 1 second buffer at 400 Hz
  // pose_qos.reliable();                      // Guarantee delivery
  // // Optionally: pose_qos.deadline(std::chrono::milliseconds(3)); // Detect stalls

  rclcpp::QoS pose_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()           // Lower latency than reliable; use reliable only if loss is unacceptable
      .durability_volatile();  // Default, do not persist

  poseStampedCovarianceSubscriber_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      poseStampedWithCovarianceTopic_,
      pose_qos,
      std::bind(&OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback, this, std::placeholders::_1)
  );

  odometrySubscriber_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
      odometryTopic_,
      pose_qos,
      std::bind(&OnlineRangeDataProcessorRos::odometryCallback, this, std::placeholders::_1)
  );

  // All dynamic pose topic discovery and odometry/pose_stamped topic subscriptions are REMOVED
  RCLCPP_INFO(nh_->get_logger(), "Open3d_slam subscribers initialized. Waiting for pose topic...");

  // rclcpp::spin(nh_);

  const int N_THREADS = std::max(2u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), N_THREADS);
  executor.add_node(nh_); // or whatever your node shared_ptr is called
  executor.spin();


  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::staticTfCallback() {
  if (!slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    staticTfCallback_->cancel();
    return;
  }

  if (readCalibrationIfNeeded()) {
    if (!slam_->isIMUattitudeInitializationEnabled()) {
      Eigen::Isometry3d T_L_sensorFrame = slam_->getExternalOdometryFrameToCloudFrameCalibration();
      if (!slam_->isUseExistingMapEnabled()) {
        slam_->setInitialTransform(T_L_sensorFrame.inverse().matrix());
      }
    }
    RCLCPP_INFO(nh_->get_logger(), "Static TF reader callback is terminated after successfully reading the transform.");
    staticTfCallback_->cancel();
  }
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_DEBUG(nh_->get_logger(), "A point cloud has arrived.");
  slam_->frames_.rangeSensorFrame = msg->header.frame_id;
  open3d::geometry::PointCloud cloud;
  if (!open3d_conversions::rosToOpen3d(msg, cloud, false, true)) {
    RCLCPP_ERROR(nh_->get_logger(), "Conversion Failed.");
  }
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

void OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!slam_->isUsingOdometryTopic()) {
    return;
  }

  geometry_msgs::msg::Pose odomPose = msg->pose.pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
  RCLCPP_DEBUG(nh_->get_logger(), "Pose with covariance callback is called.");
}

void OnlineRangeDataProcessorRos::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  if (!slam_->isUsingOdometryTopic()) {
    return;
  }

  const geometry_msgs::msg::Pose& odomPose = msg->pose.pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
  // RCLCPP_INFO(nh_->get_logger(), "Odometry message header timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
  // RCLCPP_INFO(nh_->get_logger(), "Odometry callback is called.");
}


void OnlineRangeDataProcessorRos::processOdometry(const Transform& transform, const Time& timestamp) {
  if (!slam_->isUsingOdometryTopic()) {
    // RCLCPP_WARN(nh_->get_logger(), "BIG WARNING: Odometry received but SLAM is not using odometry topic! This should NOT happen.");
    return;
  }

  if (!slam_->addOdometryPoseToBuffer(transform, timestamp)) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to add odometry pose to buffer. Exiting.");
    return;
  }

  if (!slam_->isIMUattitudeInitializationEnabled()) {
    isAttitudeInitialized_ = true;
  }

  if (!isAttitudeInitialized_) {
    // Correct the duration value for RCLCPP_WARN_THROTTLE
    // RCLCPP_WARN_THROTTLE(nh_->get_logger(), 1000, "Attitude not initialized yet, waiting for IMU measurements. Throttled 1s");
    RCLCPP_INFO(nh_->get_logger(), "Attitude not initialized yet, waiting for IMU measurements.");
    return;
  }

  // Convert timestamp to string (for logging)
  auto timestamp_seconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count();
  RCLCPP_DEBUG(nh_->get_logger(), "Processed odometry at time: %ld seconds", timestamp_seconds);
}

}  // namespace o3d_slam
