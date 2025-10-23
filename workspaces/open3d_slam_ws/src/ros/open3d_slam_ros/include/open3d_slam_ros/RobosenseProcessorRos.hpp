#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include <omp.h>

namespace robosense_processor {

enum class ColorKey : uint8_t { kWhite = 0, kRed, kGreen, kBlue, kCyan, kYellow, kGold, kGrey, kLavender, kOrange, kBlack };

struct RgbaColorMap {
  using Values = std::vector<float>;
  const std::unordered_map<ColorKey, Values> rgb_ = {{ColorKey::kWhite, {1, 1, 1, 1}},
                                                     {ColorKey::kBlue, {0, 0, 1, 1}},
                                                     {ColorKey::kCyan, {0, 1, 1, 1}},
                                                     {ColorKey::kRed, {1, 0, 0, 1}},
                                                     {ColorKey::kGreen, {0, 1, 0, 1}},
                                                     {ColorKey::kGrey, {0.705, 0.674, 0.678, 1}},
                                                     {ColorKey::kLavender, {0.560, 0.501, 0.674, 1}},
                                                     {ColorKey::kYellow, {1, 1, 0.2, 1}},
                                                     {ColorKey::kGold, {0.898, 0.784, 0.462, 1}},
                                                     {ColorKey::kOrange, {1, 0.501, 0, 1}},
                                                     {ColorKey::kBlack, {0, 0, 0, 1}}};
  Values operator[](ColorKey id) const { return rgb_.at(id); }
};

// ───────────────────────────────── pose ring-buffer ─────────────────────────────
struct TimedPose {
  rclcpp::Time stamp;
  Eigen::Isometry3d pose;
};

class PoseBuffer {
 public:
  void add(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  std::vector<TimedPose> snapshot() const;
  static bool query(const std::vector<TimedPose>& buf, const rclcpp::Time& stamp, Eigen::Isometry3d& out_pose);

 private:
  std::deque<TimedPose> buffer_;
  mutable std::shared_mutex mtx_;
  const size_t max_size_ = 500;
};

// ─────────────────────────────── ROS2 wrapper class ─────────────────────────────
class RobosenseProcessorRos : public rclcpp::Node {
 public:
  explicit RobosenseProcessorRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RobosenseProcessorRos() = default;

  void initCommonRosStuff();
  void subscribeCloud();

 protected:
  sensor_msgs::msg::PointCloud2 deskewPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  void isom3d_to_mat33_vec3(const Eigen::Isometry3d& iso, double R[3][3], double t[3]);

  inline void mat3x3_transpose(const double in[3][3], double out[3][3]) {
    out[0][0] = in[0][0];
    out[0][1] = in[1][0];
    out[0][2] = in[2][0];
    out[1][0] = in[0][1];
    out[1][1] = in[1][1];
    out[1][2] = in[2][1];
    out[2][0] = in[0][2];
    out[2][1] = in[1][2];
    out[2][2] = in[2][2];
  }

  inline void mat3x3_vec3_mult(const double M[3][3], const double v[3], double out[3]) {
    out[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
    out[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
    out[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
  }
  inline void mat3x3_vec3_mult(const double M[3][3], const float v[3], double out[3]) {
    out[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
    out[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
    out[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
  }

  inline void mat3x3_mult(const double A[3][3], const double B[3][3], double out[3][3]) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out[r][c] = A[r][0] * B[0][c] +
                    A[r][1] * B[1][c] +
                    A[r][2] * B[2][c];
      }
    }
  }

  struct FieldOffsets {
    int x = -1, y = -1, z = -1, t = -1;
  };
  FieldOffsets computeFieldOffsets(const sensor_msgs::msg::PointCloud2& msg) const;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processedPub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robosenseSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  std::string cloud_topic_;
  std::string pose_topic_;

  PoseBuffer pose_buffer_;
  Eigen::Isometry3d T_base_rslidar_ = Eigen::Isometry3d::Identity();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float distance_cutoff_ = 0.5f;  // meters
  double R_base_lidar_[3][3], t_base_lidar_[3];
};

}  // namespace airy_processor
