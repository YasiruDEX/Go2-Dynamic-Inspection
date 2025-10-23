#include "open3d_slam_ros/RobosenseProcessorRos.hpp"
#include <algorithm>
#include <atomic>
#include <iostream>
#include <numeric>
#include <vector>
#include <chrono>

using std::chrono::steady_clock;

namespace robosense_processor {

struct DeskewPoint {
  size_t idx;
  float* px;
  float* py;
  float* pz;
  double timestamp;
  uint8_t* src_ptr;
};

void PoseBuffer::add(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  Eigen::Isometry3d pose = Eigen::Translation3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) *
                           Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                              msg->pose.pose.orientation.z);

  {
    std::unique_lock lock(mtx_);
    buffer_.push_back({msg->header.stamp, pose});
    if (buffer_.size() > max_size_) buffer_.pop_front();
  }
}

std::vector<TimedPose> PoseBuffer::snapshot() const {
  std::shared_lock lock(mtx_);
  return {buffer_.begin(), buffer_.end()};
}

bool PoseBuffer::query(const std::vector<TimedPose>& buf, const rclcpp::Time& stamp, Eigen::Isometry3d& out) {
  if (buf.size() < 2) return false;

  auto it = std::lower_bound(buf.begin(), buf.end(), stamp, [](const TimedPose& p, const rclcpp::Time& t) { return p.stamp < t; });

  const TimedPose* p1;
  const TimedPose* p2;
  double tau;

  if (it == buf.begin()) {
    p1 = &buf[0];
    p2 = &buf[1];
  } else if (it == buf.end()) {
    p1 = &buf[buf.size() - 2];
    p2 = &buf.back();
  } else {
    p2 = &*it;
    p1 = &*(it - 1);
  }

  const double t1 = p1->stamp.seconds();
  const double t2 = p2->stamp.seconds();
  if (t1 == t2) return false;

  tau = (stamp.seconds() - t1) / (t2 - t1);

  Eigen::Quaterniond q1(p1->pose.rotation());
  Eigen::Quaterniond q2(p2->pose.rotation());
  Eigen::Quaterniond q = q1.slerp(tau, q2);
  Eigen::Vector3d tr = (1.0 - tau) * p1->pose.translation() + tau * p2->pose.translation();

  out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = tr;
  return true;
}

RobosenseProcessorRos::RobosenseProcessorRos(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robosense_processor_ros", options),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {}

void RobosenseProcessorRos::initCommonRosStuff() {
  cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/rslidar_points");
  pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/graph_msf/est_odometry_odom_imu");
  distance_cutoff_ = this->declare_parameter<float>("distance_cutoff", 0.5);

  std::cout << "[RobosenseProcessor] cloud_topic param: " << cloud_topic_ << '\n';
  std::cout << "[RobosenseProcessor] pose_topic param: " << pose_topic_ << '\n';
  std::cout << "[RobosenseProcessor] distance_cutoff param: " << distance_cutoff_ << '\n';

  processedPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("robosense_deskewed_cloud", rclcpp::QoS(1).transient_local());
}

void RobosenseProcessorRos::subscribeCloud() {
  constexpr double TF_TIMEOUT = 1.0;
  constexpr double WAIT_INTERVAL = 0.5;
  constexpr double WARN_INTERVAL = 5.0;

  rclcpp::Time start_time = this->now();
  rclcpp::Duration warn_duration = rclcpp::Duration::from_seconds(WARN_INTERVAL);

  while (rclcpp::ok()) {
  bool ready = tf_buffer_->canTransform("imu_link", "rslidar", tf2::TimePointZero, tf2::durationFromSec(TF_TIMEOUT));

    if (ready) {
      std::cout << "\033[1;32m[RobosenseProcessorRos] Found static TF imu_link↔rslidar.\033[0m" << std::endl;
      break;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(WAIT_INTERVAL * 1000)));

    rclcpp::Time now = this->now();
    if ((now - start_time) >= warn_duration) {
      std::cout << "\033[1;33m[RobosenseProcessorRos] Waiting for static TF base↔rslidar...\033[0m" << std::endl;
      start_time = now;
    }
  }

  // NOTE: The following block will intentionally throw an exception if the static TF base↔rslidar is missing.
  // This is by design! We want to fail fast and loudly if the transform is not available,
  // so the user knows exactly what's wrong. This is not a bug, but a deliberate choice.
  try {
    auto tf_msg = tf_buffer_->lookupTransform("imu_link", "rslidar", tf2::TimePointZero, tf2::durationFromSec(TF_TIMEOUT));
    Eigen::Quaterniond q(tf_msg.transform.rotation.w, tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    T_base_rslidar_.linear() = q.toRotationMatrix();
    T_base_rslidar_.translation() =
        Eigen::Vector3d(tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z);

    isom3d_to_mat33_vec3(T_base_rslidar_, R_base_lidar_, t_base_lidar_);

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "\033[1;33m[RobosenseProcessorRos] This exception is thrown on purpose if the transform is missing!\033[0m");
    throw std::runtime_error(std::string("Cannot start: base↔rslidar TF missing: ") + ex.what());
  }

  robosenseSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RobosenseProcessorRos::cloudCallback, this, std::placeholders::_1));

  std::cout << "\033[1;32m[RobosenseProcessorRos] Subscribed to cloud topic successfully.\033[0m" << std::endl;

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 400, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) { pose_buffer_.add(msg); });

  std::cout << "\033[1;32m[RobosenseProcessorRos] Subscribed to pose topic successfully.\033[0m" << std::endl;
}

void RobosenseProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  if (processedPub_->get_subscription_count() == 0) {
    return;
  }
  auto deskewed = deskewPointCloud(msg);
  processedPub_->publish(deskewed);
}

RobosenseProcessorRos::FieldOffsets RobosenseProcessorRos::computeFieldOffsets(const sensor_msgs::msg::PointCloud2& msg) const {
  FieldOffsets o;
  for (const auto& f : msg.fields) {
    if (f.name == "x")
      o.x = f.offset;
    else if (f.name == "y")
      o.y = f.offset;
    else if (f.name == "z")
      o.z = f.offset;
    else if (f.name == "timestamp" || f.name == "time")
      o.t = f.offset;
  }
  return o;
}

void RobosenseProcessorRos::isom3d_to_mat33_vec3(const Eigen::Isometry3d& iso, double R[3][3], double t[3]) {
  auto m = iso.linear();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R[r][c] = m(r, c);
  auto v = iso.translation();
  t[0] = v.x();
  t[1] = v.y();
  t[2] = v.z();
}

sensor_msgs::msg::PointCloud2 RobosenseProcessorRos::deskewPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  using clock = std::chrono::steady_clock;

  sensor_msgs::msg::PointCloud2 ros_msg = *msg;

  const FieldOffsets off = computeFieldOffsets(ros_msg);
  if (off.x < 0 || off.y < 0 || off.z < 0 || off.t < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Deskew: required fields missing");
    return ros_msg;
  }

  const size_t n_pts = ros_msg.width * ros_msg.height;
  const size_t step = ros_msg.point_step;
  uint8_t* base = ros_msg.data.data();

  std::vector<uint8_t> valid(n_pts, 0);
  std::vector<double> timestamps(n_pts, 0.0);

  int timestamp_field_type = -1;
  for (const auto& f : ros_msg.fields) {
    if (f.offset == off.t) {
      timestamp_field_type = f.datatype;
      break;
    }
  }
  if (timestamp_field_type != sensor_msgs::msg::PointField::FLOAT64 && timestamp_field_type != sensor_msgs::msg::PointField::FLOAT32) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Deskew: timestamp field not FLOAT64 or FLOAT32");
    return ros_msg;
  }

#pragma omp parallel for schedule(static, 4096)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    uint8_t* ptr = base + i * step;
    float* pz = reinterpret_cast<float*>(ptr + off.z);
    float range = *pz;
    if (range < distance_cutoff_) continue;

    double t_val;
    if (timestamp_field_type == sensor_msgs::msg::PointField::FLOAT64)
      t_val = *reinterpret_cast<const double*>(ptr + off.t);
    else
      t_val = static_cast<double>(*reinterpret_cast<const float*>(ptr + off.t));

    valid[i] = 1;
    timestamps[i] = t_val;
  }

  std::vector<size_t> output_idx(n_pts, 0);
  size_t n_kept = 0;
  for (size_t i = 0; i < n_pts; ++i) {
    output_idx[i] = n_kept;
    n_kept += valid[i];
  }
  if (n_kept == 0) {
    ros_msg.data.clear();
    ros_msg.width = 0;
    ros_msg.height = 1;
    ros_msg.row_step = 0;
    return ros_msg;
  }

  std::vector<TimedPose> pose_snap = pose_buffer_.snapshot();

  std::vector<double> all_times;
  all_times.reserve(n_kept);
  for (size_t i = 0; i < n_pts; ++i)
    if (valid[i]) all_times.push_back(timestamps[i]);
  std::sort(all_times.begin(), all_times.end());
  auto last = std::unique(all_times.begin(), all_times.end());
  all_times.erase(last, all_times.end());
  size_t n_times = all_times.size();

  std::vector<Eigen::Isometry3d> pose_results(n_times);

#pragma omp parallel for schedule(static, 1024)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_times); ++i) {
    double t = all_times[i];
    rclcpp::Time stamp(t * 1e9);  // seconds to nanoseconds

    auto it =
        std::lower_bound(pose_snap.begin(), pose_snap.end(), stamp, [](const TimedPose& p, const rclcpp::Time& t) { return p.stamp < t; });

    const TimedPose* p1;
    const TimedPose* p2;
    if (it == pose_snap.begin()) {
      p1 = &pose_snap[0];
      p2 = &pose_snap[1];
    } else if (it == pose_snap.end()) {
      p1 = &pose_snap[pose_snap.size() - 2];
      p2 = &pose_snap.back();
    } else {
      p2 = &*it;
      p1 = &*(it - 1);
    }

    double t1 = p1->stamp.seconds();
    double t2 = p2->stamp.seconds();
    double tau = (t2 != t1) ? ((t - t1) / (t2 - t1)) : 0.0;

    Eigen::Quaterniond q1(p1->pose.rotation());
    Eigen::Quaterniond q2(p2->pose.rotation());
    Eigen::Quaterniond q = q1.slerp(tau, q2);
    Eigen::Vector3d tr = (1.0 - tau) * p1->pose.translation() + tau * p2->pose.translation();

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.toRotationMatrix();
    result.translation() = tr;
    pose_results[i] = result;
  }

  std::vector<size_t> pose_idx(n_pts, 0);
#pragma omp parallel for schedule(static, 4096)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    if (!valid[i]) continue;
    double t = timestamps[i];
    size_t idx = std::lower_bound(all_times.begin(), all_times.end(), t) - all_times.begin();
    pose_idx[i] = idx;
  }

  Eigen::Isometry3d T_odom_base_ref;
  if (!PoseBuffer::query(pose_snap, rclcpp::Time(msg->header.stamp), T_odom_base_ref)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No reference pose for deskewing");
    return ros_msg;
  }

  double R_base_ref[3][3], t_base_ref[3];
  isom3d_to_mat33_vec3(T_odom_base_ref, R_base_ref, t_base_ref);
  double R_odom_rslidar_ref[3][3], t_odom_rslidar_ref[3];
  mat3x3_mult(R_base_ref, R_base_lidar_, R_odom_rslidar_ref);
  double tmp[3];
  mat3x3_vec3_mult(R_base_ref, t_base_lidar_, tmp);
  t_odom_rslidar_ref[0] = tmp[0] + t_base_ref[0];
  t_odom_rslidar_ref[1] = tmp[1] + t_base_ref[1];
  t_odom_rslidar_ref[2] = tmp[2] + t_base_ref[2];

  double R_ref_inv[3][3];
  mat3x3_transpose(R_odom_rslidar_ref, R_ref_inv);
  double t_ref_inv[3];
  mat3x3_vec3_mult(R_ref_inv, t_odom_rslidar_ref, tmp);
  t_ref_inv[0] = -tmp[0];
  t_ref_inv[1] = -tmp[1];
  t_ref_inv[2] = -tmp[2];

  std::vector<uint8_t> compacted(n_kept * step);

#pragma omp parallel for schedule(static, 2048)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(n_pts); ++i) {
    if (!valid[i]) continue;
    uint8_t* ptr = base + i * step;
    float* px = reinterpret_cast<float*>(ptr + off.x);
    float* py = reinterpret_cast<float*>(ptr + off.y);
    float* pz = reinterpret_cast<float*>(ptr + off.z);

    size_t idx = pose_idx[i];
    const Eigen::Isometry3d& pose = pose_results[idx];

    double R_base_pt[3][3], t_base_pt[3];
    isom3d_to_mat33_vec3(pose, R_base_pt, t_base_pt);

    double R_pt[3][3];
    mat3x3_mult(R_base_pt, R_base_lidar_, R_pt);

    mat3x3_vec3_mult(R_base_pt, t_base_lidar_, tmp);
    double t_pt[3] = {tmp[0] + t_base_pt[0], tmp[1] + t_base_pt[1], tmp[2] + t_base_pt[2]};

    float p_rsl_f[3] = {*px, *py, *pz};

    double p_tmp[3];
    mat3x3_vec3_mult(R_pt, p_rsl_f, p_tmp);
    p_tmp[0] += t_pt[0];
    p_tmp[1] += t_pt[1];
    p_tmp[2] += t_pt[2];

    double p_ref[3];
    mat3x3_vec3_mult(R_ref_inv, p_tmp, p_ref);
    p_ref[0] += t_ref_inv[0];
    p_ref[1] += t_ref_inv[1];
    p_ref[2] += t_ref_inv[2];

    size_t out_idx = output_idx[i];
    uint8_t* dst = compacted.data() + out_idx * step;
    std::memcpy(dst, ptr, step);
    float* out_px = reinterpret_cast<float*>(dst + off.x);
    float* out_py = reinterpret_cast<float*>(dst + off.y);
    float* out_pz = reinterpret_cast<float*>(dst + off.z);
    *out_px = static_cast<float>(p_ref[0]);
    *out_py = static_cast<float>(p_ref[1]);
    *out_pz = static_cast<float>(p_ref[2]);
  }

  ros_msg.data = std::move(compacted);
  ros_msg.width = n_kept;
  ros_msg.height = 1;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.header.stamp = msg->header.stamp;

  return ros_msg;
}
}  // namespace airy_processor
