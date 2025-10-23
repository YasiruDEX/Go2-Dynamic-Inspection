/*
 * DataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <unordered_map>
#include <string>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

enum class ColorKey {
  kWhite = 0, kRed, kGreen, kBlue, kCyan,
  kYellow, kGold, kGrey, kLavender, kOrange, kBlack
};

struct RgbaColorMap {
  using Values = std::vector<float>;
  Values operator[](ColorKey id) const { return rgb_.at(id); }

  const std::unordered_map<ColorKey, Values> rgb_ = {
      {ColorKey::kWhite,    {1.0f, 1.0f, 1.0f, 1.0f}},
      {ColorKey::kBlue,     {0.0f, 0.0f, 1.0f, 1.0f}},
      {ColorKey::kCyan,     {0.0f, 1.0f, 1.0f, 1.0f}},
      {ColorKey::kRed,      {1.0f, 0.0f, 0.0f, 1.0f}},
      {ColorKey::kGreen,    {0.0f, 1.0f, 0.0f, 1.0f}},
      {ColorKey::kGrey,     {0.705f, 0.674f, 0.678f, 1.0f}},
      {ColorKey::kLavender, {0.560f, 0.501f, 0.674f, 1.0f}},
      {ColorKey::kYellow,   {1.0f, 1.0f, 0.2f, 1.0f}},
      {ColorKey::kGold,     {0.898f, 0.784f, 0.462f, 1.0f}},
      {ColorKey::kOrange,   {1.0f, 0.501f, 0.0f, 1.0f}},
      {ColorKey::kBlack,    {0.0f, 0.0f, 0.0f, 1.0f}}
  };
};

class DataProcessorRos {
 public:
  explicit DataProcessorRos(rclcpp::Node::SharedPtr nh);
  virtual ~DataProcessorRos() = default;

  virtual void initialize()                            = 0;
  virtual void startProcessing()                       = 0;
  virtual void processMeasurement(const PointCloud& cloud,
                                  const Time& timestamp);
  virtual void processOdometry(const Transform& transform,
                               const Time& timestamp);

  void accumulateAndProcessRangeData(const PointCloud& cloud,
                                     const Time& timestamp);
  void processOdometryData(const Transform& transform,
                           const Time& timestamp);

  void initCommonRosStuff();
  std::shared_ptr<SlamWrapper> getSlamPtr();

 protected:
  /* --- runtime counters -------------------------------------------------- */
  size_t numAccumulatedRangeDataCount_   = 0;
  size_t numPointCloudsReceived_         = 0;
  size_t numAccumulatedRangeDataDesired_ = 1;

  PointCloud accumulatedCloud_;

  /* --- publishers -------------------------------------------------------- */
  // using PointCloudPubT = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawCloudPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registeredCloudPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr alreadyTransformedCloudPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr offlinePathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr surfaceNormalPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr offlineDifferenceLinePub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr offlineBestGuessPathPub_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr addedImuMeasPub_;

  /* --- topic names ------------------------------------------------------- */
  std::string cloudTopic_;
  std::string odometryTopic_;
  std::string poseStampedWithCovarianceTopic_;
  std::string poseStampedTopic_;
  std::string imuTopic_;

  /* --- core -------------------------------------------------------------- */
  std::shared_ptr<SlamWrapper>   slam_;
  rclcpp::Node::SharedPtr        nh_;
};

}  // namespace o3d_slam
