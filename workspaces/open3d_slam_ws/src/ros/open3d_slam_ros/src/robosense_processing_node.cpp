#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "open3d_slam_ros/RobosenseProcessorRos.hpp"

int main(int argc, char** argv) {
  using namespace robosense_processor;

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);

  auto robosenseProcessor = std::make_shared<RobosenseProcessorRos>();
  robosenseProcessor->initCommonRosStuff();
  robosenseProcessor->subscribeCloud();

  const int N_THREADS = std::max(2u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), N_THREADS);
  executor.add_node(robosenseProcessor);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
