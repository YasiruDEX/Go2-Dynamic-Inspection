#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "open3d_slam_ros/AiryProcessorRos.hpp"

int main(int argc, char** argv) {
  using namespace airy_processor;

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);

  auto airyProcessor = std::make_shared<AiryProcessorRos>();
  airyProcessor->initCommonRosStuff();
  airyProcessor->subscribeCloud();

  const int N_THREADS = std::max(2u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), N_THREADS);
  executor.add_node(airyProcessor);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
