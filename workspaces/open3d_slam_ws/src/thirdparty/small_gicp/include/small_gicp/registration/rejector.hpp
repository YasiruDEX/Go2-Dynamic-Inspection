// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

/// @brief Null correspondence rejector. This class accepts all input correspondences.
struct NullRejector {
  template <typename TargetPointCloud, typename SourcePointCloud>
  bool operator()(const TargetPointCloud& target, const SourcePointCloud& source, const Eigen::Isometry3d& T, size_t target_index, size_t source_index, double sq_dist) const {
    return false;
  }
};

/// @brief Rejecting correspondences with large distances.
struct DistanceRejector {
  DistanceRejector() : max_dist_sq(1.0) {}

  template <typename TargetPointCloud, typename SourcePointCloud>
  bool operator()(const TargetPointCloud& target, const SourcePointCloud& source, const Eigen::Isometry3d& T, size_t target_index, size_t source_index, double sq_dist) const {
    return sq_dist > max_dist_sq;
  }

  double max_dist_sq;  ///< Maximum squared distance between corresponding points
};


/// @brief Compound correspondence rejector: filters by squared distance and surface normal angle.
struct CompoundRejector {
  /// @brief Constructor
  /// @param max_sq_dist       Maximum allowed squared distance between corresponding points
  /// @param max_angle_deg     Maximum allowed angle (in degrees) between surface normals
  CompoundRejector(double max_sq_dist = 1.0, double max_angle_deg = 45.0)
      : max_dist_sq(max_sq_dist), max_angle_cos(std::cos(max_angle_deg * M_PI / 180.0)) {}

  template <typename TargetPointCloud, typename SourcePointCloud>
  bool operator()(const TargetPointCloud& target,
                  const SourcePointCloud& source,
                  const Eigen::Isometry3d& T,
                  size_t target_index,
                  size_t source_index,
                  double sq_dist) const {
    // Reject based on distance
    if (sq_dist > max_dist_sq) {
      return true;
    }

    using traits::has_normals;
    using traits::normal;

    // Reject based on angle between surface normals
    if (has_normals(target) && has_normals(source)) {
      const auto& n_target_v4 = normal(target, target_index);
      const auto& n_source_v4 = normal(source, source_index);

      const Eigen::Vector3d n_target = n_target_v4.template head<3>().normalized();
      const Eigen::Vector3d n_source = (T.linear() * n_source_v4.template head<3>()).normalized();

      const double dot = std::abs(n_target.dot(n_source));
      if (dot < max_angle_cos) {
        return true;  // reject if angle between normals is too large
      }

      // const double dot = n_target.dot(n_source);
      // constexpr double eps = 1e-12;
      // if (dot + eps < max_angle_cos)      // reject if angle > θ
      //     return true;

    }else{
      // Raise an error if normals are required but not available
      throw std::runtime_error("CompoundRejector requires point clouds with normals");
    }

    return false;
  }

  double max_dist_sq;
  double max_angle_cos;
};

}  // namespace small_gicp
