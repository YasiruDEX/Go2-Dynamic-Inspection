#pragma once

#include <memory>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/convert.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_msgs/srv/save_map.hpp"
#include "open3d_slam_msgs/srv/save_submaps.hpp"

namespace o3d_slam {

class SlamWrapperRos : public SlamWrapper {
  using BASE = SlamWrapper;

 public:
  SlamWrapperRos(const rclcpp::Node::SharedPtr& nh);
  ~SlamWrapperRos() override;

  // ROS2 service callbacks
  bool saveMapCallback(
      const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Request> req,
      std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Response> res);
  bool saveSubmapsCallback(
      const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Request> req,
      std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Response> res);

  void loadParametersAndInitialize() override;
  void startWorkers() override;

  bool readLibpointmatcherConfig(const std::string& path);

  void offlineTfWorker() override;
  void offlineVisualizationWorker() override;

  void setThreadAffinityAndPriority(std::thread& t, int core_id, int prio);

  void drawLinesBetweenPoses(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2, const rclcpp::Time& stamp);

  geometry_msgs::msg::TransformStamped baseToLidarTransform_;
  bool isStaticTransformAttempted_ = true;

 private:
  void tfWorker();
  void odomPublisherWorker();
  void visualizationWorker();
  void publishMaps(const Time& time);
  void publishDenseMap(const Time& time);
  void publishMapToOdomTf(const Time& time);
  bool isPathValid(const nav_msgs::msg::Path& path) const;

  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

  // Publishers
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odometryInputPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mappingInputPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submapOriginsPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembledMapPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr denseMapPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submapsPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bestGuessPathPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trackedPathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr differenceLinePub_;
  // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr scan2scanTransformPublisher_;
  // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan2scanOdomPublisher_;
  // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr scan2mapTransformPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan2mapOdomPublisher_;

  // Services
  rclcpp::Service<open3d_slam_msgs::srv::SaveMap>::SharedPtr saveMapSrv_;
  rclcpp::Service<open3d_slam_msgs::srv::SaveSubmaps>::SharedPtr saveSubmapsSrv_;

  bool isVisualizationFirstTime_ = true;
  std::thread tfWorkerThread_, visualizationWorkerThread_, odomPublisherWorkerThread_;
  Time prevPublishedTimeScanToScan_, prevPublishedTimeScanToMap_;
  Time prevPublishedTimeScanToScanOdom_, prevPublishedTimeScanToMapOdom_;
};

}  // namespace o3d_slam
