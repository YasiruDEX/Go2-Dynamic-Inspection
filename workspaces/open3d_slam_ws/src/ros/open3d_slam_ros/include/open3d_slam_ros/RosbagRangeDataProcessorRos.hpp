/*
 * RosbagRangeDataProcessorRos.hpp
 *
 *  Created on: Apr 21 2022
 *      Author: jelavice
 *  Ported to ROS 2 Jazzy: May 31 2025
 */

#pragma once

/* ────────────────────────────────────────────────────────
 * Standard
 * ──────────────────────────────────────────────────────── */
#include <deque>
#include <fstream>
#include <memory>
#include <optional>
#include <chrono>
#include <string>
#include <tuple>
#include <vector>

/* ────────────────────────────────────────────────────────
 * ROS 2 core
 * ──────────────────────────────────────────────────────── */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>

#include <rosbag2_cpp/info.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_options.hpp>
/* ────────────────────────────────────────────────────────
 * Messages
 * ──────────────────────────────────────────────────────── */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>   // <- used in processBuffers()
#include <cmath>                                     // <- cos(), sin()


#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/empty.hpp>

/* ────────────────────────────────────────────────────────
 * TF 2
 * ──────────────────────────────────────────────────────── */
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/* ────────────────────────────────────────────────────────
 * Rosbag 2
 * ──────────────────────────────────────────────────────── */
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>


/* ────────────────────────────────────────────────────────
 * Project-specific
 * ──────────────────────────────────────────────────────── */
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam
{

/* ========================================================
 * Helper struct
 * ====================================================== */
struct SlamInputs
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr        pointCloud_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr      odometryPose_;

  SlamInputs() = default;
  SlamInputs(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc,
             const geometry_msgs::msg::PoseStamped::ConstSharedPtr& odom)
    : pointCloud_(pc), odometryPose_(odom) {}
};

/* ========================================================
 * Main class
 * ====================================================== */
class RosbagRangeDataProcessorRos : public DataProcessorRos
{
  using BASE             = DataProcessorRos;
  using SlamInputsBuffer = std::deque<std::unique_ptr<SlamInputs>>;

public:
  explicit RosbagRangeDataProcessorRos(const rclcpp::Node::SharedPtr& nh);
  ~RosbagRangeDataProcessorRos() override = default;

  rclcpp::Node::SharedPtr nh_;

  /* ---------- lifecycle -------------------------------------------------- */
  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& stamp) override;

  /* ---------- callbacks / helpers --------------------------------------- */
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

  bool  processBuffers(SlamInputsBuffer& buffer);
  void  logTiming(const rclcpp::Duration&   elapsed, const std::string& filepath);
  void  drawLinesBetweenPoses(const nav_msgs::msg::Path& path1,
                              const nav_msgs::msg::Path& path2,
                              const rclcpp::Time&        stamp);

  std::optional<visualization_msgs::msg::Marker>
         generateMarkersForSurfaceNormalVectors(const open3d::geometry::PointCloud& cloud,
                                                const rclcpp::Time&                 stamp,
                                                const o3d_slam::RgbaColorMap::Values& color);

  bool  validateTopicsInRosbag(const std::string&               bag_path,
                               const std::vector<std::string>&  mandatoryTopics);

  bool  readCalibrationIfNeeded();
  bool  run();

  std::tuple<rclcpp::Duration, rclcpp::Duration, rclcpp::Duration>
         usePairForRegistration();

private:
  /* ---------- internal I/O --------------------------------------------- */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  bool processRosbag();

  /* ---------- utility --------------------------------------------------- */
  std::string buildUpLogFilename(const std::string& suffix,
                                 const std::string& ext = ".txt");
  bool createOutputDirectory();

  visualization_msgs::msg::MarkerArray convertPathToMarkerArray(const nav_msgs::msg::Path& path);
  visualization_msgs::msg::Marker      createLineStripMarker();

  o3d_slam::PointCloud lineStripToPointCloud(const visualization_msgs::msg::MarkerArray& markers,
                                             int num_samples);

  void calculateSurfaceNormals(o3d_slam::PointCloud& cloud);
  void processRosbagForIMU();
  void exportIMUData();

  /* ---------- configuration / parameters -------------------------------- */
  std::string   rosbagFilename_;
  Parameters    parameters_;

  /* ---------- state ----------------------------------------------------- */
  nav_msgs::msg::Path                         trackedPath_;
  nav_msgs::msg::Path                         bestGuessPath_;
  std::deque<geometry_msgs::msg::PoseStamped> registeredPoses_;

  rclcpp::Time      tracker;
  rclcpp::Duration  timeDiff_{0, 0};          //  ⟵  **FIX** (valid default-ctor)

  bool   isFirstMessage_         {true};
  bool   isStaticTransformFound_ {false};
  bool   isBagReadyToPlay_       {false};
  bool   isFirstWrite            {true};
  double maxProcessingRate_      {-0.1};

  /* ---------- publishers / services ------------------------------------- */
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr       clockPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   inputPointCloudPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometryPosePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr registeredPosePublisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr              sleepServer_;

  /* ---------- TF -------------------------------------------------------- */
  std::shared_ptr<tf2_ros::TransformBroadcaster>        transformBroadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>  staticTransformBroadcaster_;
  std::shared_ptr<tf2_ros::Buffer>                      tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener>           tfListener_;
  geometry_msgs::msg::TransformStamped                  baseToLidarTransform_;
  std::vector<geometry_msgs::msg::TransformStamped>     staticTransforms_;

  /* ---------- rosbag 2 writer ------------------------------------------ */
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;

  /* ---------- misc ------------------------------------------------------ */
  std::ofstream           poseFile_;
  std::ofstream           imuFile_;
  std::string             asyncOdometryFrame_;
  std::string             odometryHeader_{"/bestHeaderThereis"};
  std::string             tfTopic_{"/tf"};
  std::string             tfStaticTopic_{"/tf_static"};
  std::string             clockTopic_{"/clock"};
  o3d_slam::RgbaColorMap  colorMap_;

  /* ========  TIME-CONVERSION HELPERS  ======== */
  /* Convert builtin_interfaces::msg::Time → internal o3d_slam::Time  */
  inline o3d_slam::Time fromRos2(const builtin_interfaces::msg::Time& stamp) {
    using namespace std::chrono;
    const auto ns = seconds(stamp.sec) + nanoseconds(stamp.nanosec);
    return o3d_slam::Time(duration_cast<o3d_slam::Time::duration>(ns));
  }

  /* Convert internal o3d_slam::Time → builtin_interfaces::msg::Time  */
  inline builtin_interfaces::msg::Time toRos2(const o3d_slam::Time& t) {
    using namespace std::chrono;
    const int64_t ns_total =
        duration_cast<nanoseconds>(t.time_since_epoch()).count();
    builtin_interfaces::msg::Time out;
    out.sec     = static_cast<int32_t>(ns_total / 1000000000LL);
    out.nanosec = static_cast<uint32_t>(ns_total % 1000000000LL);
    return out;
  }

  /* Overload kept for the (few) places where we really have an rclcpp::Time */
  inline builtin_interfaces::msg::Time toRos2(const rclcpp::Time& t) {
    builtin_interfaces::msg::Time out;
    out.sec     = static_cast<int32_t>(t.seconds());
    out.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL);
    return out;
  }

  /* Convert internal o3d_slam::Time → rclcpp::Time (needed for RViz etc.) */
  inline rclcpp::Time toRclcpp(const o3d_slam::Time& t) {
    using namespace std::chrono;
    return rclcpp::Time(
        duration_cast<nanoseconds>(t.time_since_epoch()).count(),
        RCL_ROS_TIME);
  }

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

}  // namespace o3d_slam