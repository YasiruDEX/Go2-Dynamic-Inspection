/*
 * DataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/DataProcessorRos.hpp"

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "open3d_slam/magic.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

DataProcessorRos::DataProcessorRos(rclcpp::Node::SharedPtr nh)
    : nh_(std::move(nh)) {}

void DataProcessorRos::initCommonRosStuff() {
  cloudTopic_                     = nh_->declare_parameter<std::string>("cloud_topic", "");
  odometryTopic_                  = nh_->declare_parameter<std::string>("odometry_topic", "");
  poseStampedTopic_               = nh_->declare_parameter<std::string>("pose_stamped_topic", "");
  poseStampedWithCovarianceTopic_ = nh_->declare_parameter<std::string>("pose_stamped_with_covariance_topic", "");

  std::cout << "Cloud topic:                     " << cloudTopic_  << '\n'
            << "Odometry topic:                  " << odometryTopic_ << '\n'
            << "PoseStamped topic:               " << poseStampedTopic_ << '\n'
            << "PoseWithCovarianceStamped topic: " << poseStampedWithCovarianceTopic_ << '\n';

  /* QoS chosen:
   * - depth 1
   * - transient local so late-joining RViz can see the last message   */
  // rclcpp::QoS latched_qos(1U);
  // const auto latched = latched_qos.transient_local();

  // rclcpp::QoS qos(1);
  // qos.best_effort();
  // qos.durability_volatile();  // Use .durability_volatile(), not .volatile_()

  auto qos = rclcpp::QoS(1).reliable();

  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  // rawCloudPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", qos, pub_options);
  alreadyTransformedCloudPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("already_transformed_cloud", qos, pub_options);
  registeredCloudPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_cloud", qos, pub_options);
  offlinePathPub_ = nh_->create_publisher<nav_msgs::msg::Path>("tracked_path", qos, pub_options);
  surfaceNormalPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("surface_normals", qos, pub_options);
  offlineDifferenceLinePub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("difference_lines", qos, pub_options);
  offlineBestGuessPathPub_ = nh_->create_publisher<nav_msgs::msg::Path>("best_guess_path", qos, pub_options);

}

/* -------------------------------------------------------------------------- */
/*  Default (no-op) virtuals – can be overridden by derived classes           */
/* -------------------------------------------------------------------------- */
void DataProcessorRos::processMeasurement(const PointCloud&, const Time&) {
  std::cout << "Warning: processMeasurement() not implemented by derived class.\n";
}

void DataProcessorRos::processOdometry(const Transform&, const Time&) {
  std::cout << "Warning: processOdometry() not implemented by derived class.\n";
}

/* -------------------------------------------------------------------------- */
std::shared_ptr<SlamWrapper> DataProcessorRos::getSlamPtr() { return slam_; }

/* -------------------------------------------------------------------------- */
void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud& cloud,
                                                     const Time& timestamp) {
  processMeasurement(cloud, timestamp);
}

void DataProcessorRos::processOdometryData(const Transform& transform,
                                           const Time& timestamp) {
  processOdometry(transform, timestamp);
}

}  // namespace o3d_slam
