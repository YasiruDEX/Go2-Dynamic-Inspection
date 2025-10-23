/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <open3d/Open3D.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  using namespace o3d_slam;

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<rclcpp::Node>("open3d_slam", options);

  // You may need to declare parameters explicitly in ROS2
  node->declare_parameter<std::string>("parameter_folder_path", "");
  node->declare_parameter<std::string>("parameter_filename", "");
  node->declare_parameter<bool>("is_read_from_rosbag", false);

  const std::string paramFolderPath = node->get_parameter("parameter_folder_path").as_string();
  const std::string paramFilename = node->get_parameter("parameter_filename").as_string();

  // The LUA parameters are loaded twice. This is the first time. Soley because we need to know if we are using a map for initialization.
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params);

  const bool isProcessAsFastAsPossible = node->get_parameter("is_read_from_rosbag").as_bool();
  std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
  std::cout << "Is use a map for initialization: " << std::boolalpha << params.mapper_.isUseInitialMap_ << "\n";
  std::cout << "Is Map carving enabled: " << std::boolalpha << params.mapper_.isCarvingEnabled_ << "\n";

  // This is where the initial class is constructed and passed on.
  std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(node, isProcessAsFastAsPossible);
  dataProcessor->initialize();

  std::shared_ptr<SlamMapInitializer> slamMapInitializer;
  if (params.mapper_.isUseInitialMap_) {
    std::shared_ptr<SlamWrapper> slam = dataProcessor->getSlamPtr();
    slamMapInitializer = std::make_shared<SlamMapInitializer>(slam, node);
    slamMapInitializer->initialize(params.mapper_.mapInit_);
  }

  dataProcessor->startProcessing();

  rclcpp::shutdown();
  return 0;
}
