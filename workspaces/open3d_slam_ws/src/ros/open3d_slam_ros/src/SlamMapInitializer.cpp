/*
 * SlamMapInitializer.cpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "open3d/io/PointCloudIO.h"

#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include <boost/filesystem.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace o3d_slam {

const double sqrt2 = std::sqrt(2.0);

SlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, rclcpp::Node::SharedPtr nh)
    : server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("initialization_pose", nh)), slamPtr_(slamPtr), nh_(nh) {}

SlamMapInitializer::~SlamMapInitializer() {
  if (initWorker_.joinable()) {
    initWorker_.join();
    std::cout << "Joined mapInitializer worker \n";
  }
}

std::string SlamMapInitializer::get_map_file_path(const std::string& package, const std::string& map_name) {
    // 1. Try installed share directory
    std::string installed_path = ament_index_cpp::get_package_share_directory(package) + "/data/" + map_name;
    if (boost::filesystem::exists(installed_path)) {
        return installed_path;
    }
    // 2. Try source tree (relative to CMakeLists.txt or known dev path)
    // You must know your repo layout for this to work. Example:
    std::string dev_path = std::string(PROJECT_SOURCE_DIR) + "/data/" + map_name; // If you define PROJECT_SOURCE_DIR
    if (boost::filesystem::exists(dev_path)) {
      return dev_path;
    }
    // 3. Optionally: try cwd or error
    throw std::runtime_error("Map file not found in install or source: " + map_name);
}

void SlamMapInitializer::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  Eigen::Isometry3d init_transform;
  tf2::fromMsg(msg->pose.pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}

bool SlamMapInitializer::initSlamCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
  res->success = true;
  return true;
}

void SlamMapInitializer::initialize(const MapInitializingParameters& params) {
  mapInitializerParams_ = params;

  if (mapInitializerParams_.pcdFilePath_.empty()) {
    throw std::runtime_error("Error: mapInitializerParams_.pcdFilePath_ is empty. Please provide a valid PCD file path.");
  }
  
  std::string map_name = mapInitializerParams_.pcdFilePath_;
  std::string pcdFile = get_map_file_path("open3d_slam_ros", map_name);

  initialized_.store(false);

  PointCloud raw_map;
  std::cout << "Loading pointloud from: " << pcdFile << "\n";
  if (!open3d::io::ReadPointCloud(pcdFile, raw_map)) {
    std::cerr << "[Error] Initialization pointcloud not loaded" << std::endl;
  }

  Transform initPose = params.initialPose_;
  slamPtr_->setInitialMap(raw_map);
  slamPtr_->setInitialTransform(initPose.matrix());
  std::cout << "Init pose within the given map: " << asString(initPose) << std::endl;
  if (params.isInitializeInteractively_) {
    initInteractiveMarker();
    initPoseSub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&SlamMapInitializer::initialPoseCallback, this, std::placeholders::_1));
    initializeSlamSrv_ = nh_->create_service<std_srvs::srv::Trigger>(
        "initialize_slam", std::bind(&SlamMapInitializer::initSlamCallback, this, std::placeholders::_1, std::placeholders::_2));
    cloudPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud_preview", 1);
    std::string cloudTopic = nh_->declare_parameter<std::string>("cloud_topic", "");
    std::cout << "Initializer subscribing to " << cloudTopic << std::endl;
    cloudSub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloudTopic, 1, std::bind(&SlamMapInitializer::pointcloudCallback, this, std::placeholders::_1));
    initWorker_ = std::thread([this]() { initializeWorker(); });
    std::cout << "started interactive marker worker \n";
  } else {
    std::cout << "Finished setting initial map! \n";
  }
}

void SlamMapInitializer::initializeWorker() {
  rclcpp::Rate r(20);
  const bool isMergeScansIntoMap = slamPtr_->getMapperParameters().isMergeScansIntoMap_;
  slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = false;
  slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = true;
  while (rclcpp::ok() && !initialized_.load()) {
    rclcpp::spin_some(nh_);
    r.sleep();
  }
  slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = isMergeScansIntoMap;
  usleep(1000000);
  slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = false;
  std::cout << "Finished setting initial map! \n";
}

void SlamMapInitializer::initInteractiveMarker() {
  menuHandler_.insert("Initialize SLAM map", std::bind(&SlamMapInitializer::initMapCallback, this, std::placeholders::_1));
  menuHandler_.insert("Set Pose", std::bind(&SlamMapInitializer::setPoseCallback, this, std::placeholders::_1));

  auto interactiveMarker = createInteractiveMarker();
  interactiveMarkerName_ = interactiveMarker.name;
  server_->insert(interactiveMarker);
  menuHandler_.apply(*server_, interactiveMarker.name);
  server_->applyChanges();
}

void SlamMapInitializer::setPoseCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) {
  Eigen::Isometry3d init_transform;
  tf2::fromMsg(msg->pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}

void SlamMapInitializer::initMapCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) {
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
}

void SlamMapInitializer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  visualization_msgs::msg::InteractiveMarker marker;
  server_->get(interactiveMarkerName_, marker);
  Eigen::Isometry3d markerPose;
  tf2::fromMsg(marker.pose, markerPose);
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(*msg, cloud, false, true);
  cloud.Transform(markerPose.matrix());
  o3d_slam::publishCloud(cloud, slamPtr_->frames_.mapFrame, rclcpp::Time(marker.header.stamp), cloudPub_);
}

visualization_msgs::msg::InteractiveMarker SlamMapInitializer::createInteractiveMarker() const {
  visualization_msgs::msg::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = mapInitializerParams_.frameId_;
  interactiveMarker.header.stamp = rclcpp::Clock().now();
  interactiveMarker.name = "Initial Pose";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to see options";

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
  geometry_msgs::msg::Pose poseMsg;
  Eigen::Isometry3d initialPose = mapInitializerParams_.initialPose_;

  // Set translation
  poseMsg.position.x = initialPose.translation().x();
  poseMsg.position.y = initialPose.translation().y();
  poseMsg.position.z = initialPose.translation().z();

  // Set rotation (convert Eigen::Quaterniond to geometry_msgs::msg::Quaternion)
  Eigen::Quaterniond rotation(initialPose.rotation());
  poseMsg.orientation.x = rotation.x();
  poseMsg.orientation.y = rotation.y();
  poseMsg.orientation.z = rotation.z();
  poseMsg.orientation.w = rotation.w();

  interactiveMarker.pose = poseMsg;

  // create a mesh marker
  const auto arrowMarker = []() {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.pose.position.x = -0.25;
    return marker;
  }();

  // create a non-interactive control which contains the mesh
  visualization_msgs::msg::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(arrowMarker);
  boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the mesh
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 1.0 / sqrt2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0;
  control.orientation.y = 1.0 / sqrt2;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1.0 / sqrt2;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

}  // namespace o3d_slam
