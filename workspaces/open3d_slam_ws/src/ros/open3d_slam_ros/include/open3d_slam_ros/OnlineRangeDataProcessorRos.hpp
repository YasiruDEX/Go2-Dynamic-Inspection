#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <optional>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr nh);
  ~OnlineRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void processOdometry(const Transform& cloud, const Time& timestamp) override;

  bool initializeTheTransformBuffers_ = true;

  void staticTfCallback();

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool readCalibrationIfNeeded();
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void poseStampedWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::optional<visualization_msgs::msg::Marker> generateMarkersForSurfaceNormalVectors(
      const open3d::geometry::PointCloud& pointCloud,
      const rclcpp::Time& timestamp,
      const o3d_slam::RgbaColorMap::Values& color);

  // --- ROS2 node and comms ---
  rclcpp::Node::SharedPtr nh_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseStampedCovarianceSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseStampedSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;

  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr addedImuMeasPub_;

  rclcpp::TimerBase::SharedPtr staticTfCallback_;  // Declare the timer here

  // --- TF2 ---
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // --- State ---
  uint64_t counter_ = 0;
  bool poseStampedCallBackEnabled_ = false;
  bool odometryCallBackEnabled_ = false;
  bool poseStampedWithCovarianceCallBackEnabled_ = false;
  bool isAttitudeInitialized_ = false;
  bool isStaticTransformFound_ = false;
  bool poseSubscribed_ = false;

  // std::shared_ptr<ImuBuffer> imuBufferPtr_;
  Transform lidarToImu_ = Transform::Identity();
  o3d_slam::RgbaColorMap colorMap_;
};

}  // namespace o3d_slam
