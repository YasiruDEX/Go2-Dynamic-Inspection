#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

struct SymmetricPointToPlaneICPFactor {
  struct Setting {};

  explicit SymmetricPointToPlaneICPFactor(const Setting& = Setting())
      : target_index(std::numeric_limits<size_t>::max()),
        source_index(std::numeric_limits<size_t>::max()) {}

  template <typename TargetPointCloud,
            typename SourcePointCloud,
            typename TargetTree,
            typename CorrespondenceRejector>
  bool linearize(const TargetPointCloud&  target,
                 const SourcePointCloud&  source,
                 const TargetTree&        target_tree,
                 const Eigen::Isometry3d& T,
                 size_t                   source_idx,
                 const CorrespondenceRejector& rejector,
                 Eigen::Matrix<double, 6, 6>* H,
                 Eigen::Matrix<double, 6, 1>* b,
                 double*                   e)
  {
    constexpr size_t k_neighbors = 7; //3 works              
    constexpr double degenerate_threshold = 1e-6;  // Threshold for ‖n_s + n_t‖²

    source_index = source_idx;

    const Eigen::Vector4d p_s_v4    = traits::point(source, source_idx);
    const Eigen::Vector4d p_s_v4_tr = T * p_s_v4;
    const Eigen::Vector3d p_s       = p_s_v4.template head<3>();
    const Eigen::Vector3d p_s_tr    = p_s_v4_tr.template head<3>();
    const Eigen::Vector3d n_s       = traits::normal(source, source_idx).template head<3>();

    std::array<size_t, k_neighbors>  k_indices;
    std::array<double, k_neighbors>  k_sq_dists;

    if (traits::knn_search(target_tree, p_s_v4_tr, k_neighbors, k_indices.data(), k_sq_dists.data()) != k_neighbors) {
      return false;
    }

    Eigen::Matrix<double, 6, 6> H_acc = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b_acc = Eigen::Matrix<double, 6, 1>::Zero();
    double e_acc = 0.0;
    bool valid = false;

    const Eigen::Matrix3d& R = T.linear();

    for (size_t i = 0; i < k_neighbors; ++i) {
      size_t k_index = k_indices[i];
      double sq_dist = k_sq_dists[i];

      if (rejector(target, source, T, k_index, source_idx, sq_dist)) continue;

      this->target_index = k_index;

      const Eigen::Vector3d p_t = traits::point(target, k_index).template head<3>();
      Eigen::Vector3d n_t       = traits::normal(target, k_index).template head<3>();

      // Ensure normals are in the same hemisphere for averaging
      if (n_s.dot(n_t) < 0.0) {
        n_t = -n_t;
      }

      Eigen::Vector3d n_avg = n_s + n_t;
      n_avg.normalize();

      // J = [ nᵀ (-R skew(p_s)) | nᵀ R ]   // 1×6
      // ^ 3 rot cols        ^ 3 trans cols

      // ---- Residual and Jacobian, using correct right-mult. convention
      const Eigen::Vector3d r = p_s_tr - p_t;
      double err = n_avg.dot(r);

      Eigen::Matrix<double, 1, 6> J;
      J.leftCols<3>()  = n_avg.transpose() * (-R * skew(p_s));
      J.rightCols<3>() = n_avg.transpose() *  R;

      // Eigen::Matrix<double,6,6> H_r = Eigen::Matrix<double,6,6>::Zero();
      // // H_r = Jᵀ J +   ->> this is the 2nd order term. err * H_r
      // Eigen::Matrix3d skew_ps = skew(p_s);
      // Eigen::RowVector3d nR = n_avg.transpose() * R;
      // Eigen::Matrix3d rot_rot = - (nR.transpose() * p_s.transpose()) * skew_ps;
      // // Top-right and bottom-left: rot-trans, trans-rot
      // Eigen::Matrix3d rot_trans = Eigen::Matrix3d::Zero();
      // // Bottom-right: trans-trans
      // Eigen::Matrix3d trans_trans = Eigen::Matrix3d::Zero();
      
      // H_r.block<3,3>(0,0) = rot_rot;
      // H_r.block<3,3>(0,3) = rot_trans;
      // H_r.block<3,3>(3,0) = rot_trans.transpose();
      // H_r.block<3,3>(3,3) = trans_trans;


      H_acc.noalias() += J.transpose() * J ;//+ err * H_r;
      b_acc.noalias() += J.transpose() * err;
      e_acc          += 0.5 * err * err;

      valid = true;
    }

    if (valid) {
      *H = H_acc;
      *b = b_acc;
      *e = e_acc;
    }

    return valid;
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud& target,
               const SourcePointCloud& source,
               const Eigen::Isometry3d& T) const {
    if (target_index == std::numeric_limits<size_t>::max()) {
      return 0.0;
    }

    const Eigen::Vector3d p_s_tr = (T * traits::point(source, source_index)).template head<3>();
    const Eigen::Vector3d p_t    = traits::point(target, target_index).template head<3>();
    const Eigen::Vector3d r      = p_s_tr - p_t;

    const Eigen::Vector3d n_s = traits::normal(source, source_index).template head<3>();
    const auto n_t_ref = traits::normal(target, target_index).template head<3>();

    Eigen::Vector3d n_avg = Eigen::Vector3d::Zero();
    if (n_s.dot(n_t_ref) < 0.0){
      n_avg = n_s - n_t_ref;
    }
    else{
      n_avg = n_s + n_t_ref;
    }
    n_avg.normalize();

    double err = n_avg.dot(r);
    return 0.5 * err * err;
  }

  bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }
  void reset_inlier() { target_index = std::numeric_limits<size_t>::max();        }

  size_t target_index;
  size_t source_index;
};



// struct SymmetricPointToPlaneICPFactor {
//   struct Setting {};

//   explicit SymmetricPointToPlaneICPFactor(const Setting& = Setting())
//       : target_index(std::numeric_limits<size_t>::max()),
//         source_index(std::numeric_limits<size_t>::max()) {}

//   template <typename TargetPointCloud,
//             typename SourcePointCloud,
//             typename TargetTree,
//             typename CorrespondenceRejector>
//   bool linearize(const TargetPointCloud&  target,
//                  const SourcePointCloud&  source,
//                  const TargetTree&        target_tree,
//                  const Eigen::Isometry3d& T,
//                  size_t                   source_idx,
//                  const CorrespondenceRejector& rejector,
//                  Eigen::Matrix<double, 6, 6>* H,
//                  Eigen::Matrix<double, 6, 1>* b,
//                  double*                   e)
//   {
//     // constexpr size_t k_neighbors = 7;              
//     constexpr double degenerate_threshold = 1e-6;  // Threshold for ‖n_s + n_t‖²

//     source_index = source_idx;

//     const Eigen::Vector4d p_s_v4    = traits::point(source, source_idx);
//     const Eigen::Vector4d p_s_v4_tr = T * p_s_v4;
//     const Eigen::Vector3d p_s       = p_s_v4.template head<3>();
//     const Eigen::Vector3d p_s_tr    = p_s_v4_tr.template head<3>();
//     const Eigen::Vector3d n_s       = traits::normal(source, source_idx).template head<3>();

//     std::array<size_t, k_neighbors>  k_indices;
//     std::array<double, k_neighbors>  k_sq_dists;

//     if (traits::knn_search(target_tree, p_s_v4_tr, k_neighbors, k_indices.data(), k_sq_dists.data()) != k_neighbors) {
//       return false;
//     }

//     for (size_t i = 0; i < k_neighbors; ++i) {
//       neighbor_target_indices[i] = k_indices[i];
//       neighbor_sq_dists[i] = k_sq_dists[i];
//     }
      
//     // // Find the index with the smallest squared distance
//     // size_t min_idx = 0;
//     // double min_dist = k_sq_dists[0];
//     // for (size_t i = 1; i < k_neighbors; ++i) {
//     //   if (k_sq_dists[i] < min_dist) {
//     //   min_dist = k_sq_dists[i];
//     //   min_idx = i;
//     //   }
//     // }
//     // this->target_index = k_indices[min_idx];

//     Eigen::Matrix<double, 6, 6> H_acc = Eigen::Matrix<double, 6, 6>::Zero();
//     Eigen::Matrix<double, 6, 1> b_acc = Eigen::Matrix<double, 6, 1>::Zero();
//     double e_acc = 0.0;
//     bool valid = false;

//     const Eigen::Matrix3d& R = T.linear();

//     for (size_t i = 0; i < k_neighbors; ++i) {
//       size_t k_index = k_indices[i];
//       double sq_dist = k_sq_dists[i];

//       if (rejector(target, source, T, k_index, source_idx, sq_dist)) continue;

//       const Eigen::Vector3d p_t = traits::point(target, k_index).template head<3>();
//       Eigen::Vector3d n_t       = traits::normal(target, k_index).template head<3>();

//       if (n_s.dot(n_t) < 0.0) n_t = -n_t;

//       // ---- Averaged normal, only if not degenerate
//       Eigen::Vector3d n_avg = n_s + n_t;
//       // double norm2 = n_avg.squaredNorm();
//       // if (norm2 < degenerate_threshold) continue;
//       n_avg.normalize();

//       // J = [ nᵀ (-R skew(p_s)) | nᵀ R ]   // 1×6
//       // ^ 3 rot cols        ^ 3 trans cols

//       // ---- Residual and Jacobian, using correct right-mult. convention
//       const Eigen::Vector3d r = p_s_tr - p_t;
//       double err = n_avg.dot(r);

//       Eigen::Matrix<double, 1, 6> J;
//       J.leftCols<3>()  = n_avg.transpose() * (-R * skew(p_s));
//       J.rightCols<3>() = n_avg.transpose() *  R;

//       Eigen::Matrix<double,6,6> H_r = Eigen::Matrix<double,6,6>::Zero();

//       Eigen::Matrix3d skew_ps = skew(p_s);
//       Eigen::RowVector3d nR = n_avg.transpose() * R;
//       Eigen::Matrix3d rot_rot = - (nR.transpose() * p_s.transpose()) * skew_ps;
//       // Top-right and bottom-left: rot-trans, trans-rot
//       Eigen::Matrix3d rot_trans = Eigen::Matrix3d::Zero();
//       // Bottom-right: trans-trans
//       Eigen::Matrix3d trans_trans = Eigen::Matrix3d::Zero();
      
//       H_r.block<3,3>(0,0) = rot_rot;
//       H_r.block<3,3>(0,3) = rot_trans;
//       H_r.block<3,3>(3,0) = rot_trans.transpose();
//       H_r.block<3,3>(3,3) = trans_trans;


//       H_acc.noalias() += J.transpose() * J + err * H_r;
//       b_acc.noalias() += J.transpose() * err;
//       e_acc          += 0.5 * err * err;

//       valid = true;
//     }

//     if (valid) {
//       *H = H_acc;
//       *b = b_acc;
//       *e = e_acc;
//     }

//     return valid;
//   }

//   template <typename TargetPointCloud, typename SourcePointCloud>
//     double error(const TargetPointCloud& target,
//       const SourcePointCloud& source,
//       const Eigen::Isometry3d& T) const
//   {
//   if (source_index == std::numeric_limits<size_t>::max()) return 0.0;

//   const Eigen::Vector3d p_s_tr = (T * traits::point(source, source_index)).template head<3>();
//   const Eigen::Vector3d n_s = traits::normal(source, source_index).template head<3>();

//   double total_error = 0.0;
//   double weight_sum = 0.0;

//   for (size_t i = 0; i < k_neighbors; ++i) {
//   size_t tgt_idx = neighbor_target_indices[i];
//   if (tgt_idx == std::numeric_limits<size_t>::max()) continue; // invalid neighbor

//   const Eigen::Vector3d p_t = traits::point(target, tgt_idx).template head<3>();
//   const Eigen::Vector3d n_t_unflipped = traits::normal(target, tgt_idx).template head<3>();
//   Eigen::Vector3d n_t = n_t_unflipped;
//   if (n_s.dot(n_t) < 0.0) n_t = -n_t;

//   Eigen::Vector3d n_avg = n_s + n_t;
//   double norm2 = n_avg.squaredNorm();
//   if (norm2 < 1e-6) continue;
//   n_avg.normalize();

//   const Eigen::Vector3d r = p_s_tr - p_t;
//   double err = n_avg.dot(r);

//   // Example: inverse distance weighting
//   double weight = 1.0 / (neighbor_sq_dists[i] + 1e-8);
//   total_error += weight * 0.5 * err * err;
//   weight_sum += weight;
//   }

//   if (weight_sum > 0.0)
//   return total_error / weight_sum; // weighted average
//   return 0.0;
//   }

//   bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }
//   void reset_inlier() { target_index = std::numeric_limits<size_t>::max();        }

//   static constexpr size_t k_neighbors = 7; // use the same value as linearize
//   std::array<size_t, k_neighbors> neighbor_target_indices;
//   std::array<double, k_neighbors> neighbor_sq_dists;

//   size_t target_index;
//   size_t source_index;
// };
/**
 * Symmetric point-to-plane ICP factor with consistent frames
 *
 * – All normals are expressed in the **target** frame.
 * – Residual:   e = n̄ᵀ (R p_s + t − p_t)
 * – Jacobian:   J = [ (R p_s × n̄)ᵀ ┊ n̄ᵀ ]
 */
struct SymmetricPointToPlaneICPFactor2 {
  struct Setting {};

  explicit SymmetricPointToPlaneICPFactor2(const Setting& = Setting())
      : target_index(std::numeric_limits<size_t>::max()),
        source_index(std::numeric_limits<size_t>::max()) {}

  // --------------------------------------------------------------------------
  template <typename TargetPointCloud,
            typename SourcePointCloud,
            typename TargetTree,
            typename CorrespondenceRejector>
  bool linearize(const TargetPointCloud&        target,
                 const SourcePointCloud&        source,
                 const TargetTree&              target_tree,
                 const Eigen::Isometry3d&       T,          // source → target
                 size_t                         source_idx,
                 const CorrespondenceRejector&  rejector,
                 Eigen::Matrix<double, 6, 6>*   H,
                 Eigen::Matrix<double, 6, 1>*   b,
                 double*                        e)
  {
    constexpr std::size_t k_neighbors           = 7;
    constexpr double      k_degenerate_threshold = 1e-6;

    source_index = source_idx;
    target_index = std::numeric_limits<size_t>::max();

    /* source point / normal (source frame) --------------------------------- */
    const Eigen::Vector3d p_s = traits::point  (source,  source_index).template head<3>();
    const Eigen::Vector3d n_s = traits::normal (source,  source_index).template head<3>();

    const Eigen::Matrix3d& R = T.linear();          // source → target
    const Eigen::Vector3d& t = T.translation();

    const Eigen::Vector3d Rp_s   = R * p_s;          // source point in target frame
    const Eigen::Vector3d p_s_tr = Rp_s + t;         // transformed source point

    /* k-NN search around transformed source point (target frame) ----------- */
    std::array<std::size_t, k_neighbors> k_indices;
    std::array<double,       k_neighbors> k_sq_dists;

    if (traits::knn_search(target_tree, p_s_tr,               // 3-D query
                           k_neighbors,
                           k_indices.data(),
                           k_sq_dists.data()) != k_neighbors) {
      return false;
    }

    Eigen::Matrix<double, 6, 6> H_acc = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b_acc = Eigen::Matrix<double, 6, 1>::Zero();
    double                       e_acc = 0.0;
    bool                         valid = false;

    for (std::size_t i = 0; i < k_neighbors; ++i) {
      const std::size_t k_index = k_indices[i];
      const double      sq_dist = k_sq_dists[i];

      if (rejector(target, source, T, k_index, source_idx, sq_dist))
        continue;

      if (target_index == std::numeric_limits<size_t>::max())
        target_index = k_index;

      /* target point / normal (target frame) ------------------------------ */
      const Eigen::Vector3d p_t = traits::point (target, k_index).template head<3>();
      Eigen::Vector3d       n_t = traits::normal(target, k_index).template head<3>();

      /* source normal → target frame, then average ------------------------ */
      const Eigen::Vector3d n_s_t = R * n_s;                  // rotate only

      if (n_s_t.dot(n_t) < 0.0) n_t = -n_t;                   // hemisphere

      Eigen::Vector3d n_avg = n_s_t + n_t;
      const double    norm2 = n_avg.squaredNorm();
      if (norm2 < k_degenerate_threshold) {
        continue;
      }
      n_avg /= std::sqrt(norm2);

      const Eigen::Vector3d r_vec    = Rp_s + t - p_t;
      const double          err      = n_avg.dot(r_vec);

      const Eigen::Vector3d rot_term = Rp_s.cross(n_avg);

      Eigen::Matrix<double, 1, 6> J;
      J.leftCols<3>()  = rot_term.transpose();
      J.rightCols<3>() = n_avg.transpose();

      H_acc.noalias() += J.transpose() * J;
      b_acc.noalias() += J.transpose() * err;
      e_acc          += 0.5 * err * err;

      valid = true;
    }

    if (valid) {
      *H = H_acc;
      *b = b_acc;
      *e = e_acc;
    }
    return valid;
  }

  // --------------------------------------------------------------------------
  template <typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud&  target,
               const SourcePointCloud&  source,
               const Eigen::Isometry3d& T) const
  {
    if (target_index == std::numeric_limits<std::size_t>::max() ||
        source_index == std::numeric_limits<std::size_t>::max())
      return 0.0;

    const Eigen::Vector3d p_s = traits::point  (source, source_index).template head<3>();
    const Eigen::Vector3d n_s = traits::normal (source, source_index).template head<3>();
    const Eigen::Vector3d p_t = traits::point  (target, target_index).template head<3>();
    Eigen::Vector3d       n_t = traits::normal (target, target_index).template head<3>();

    const Eigen::Matrix3d& R = T.linear();
    const Eigen::Vector3d& t = T.translation();

    const Eigen::Vector3d n_s_t = R * n_s;
    if (n_s_t.dot(n_t) < 0.0) {
      n_t = -n_t;
    }

    Eigen::Vector3d n_avg = n_s_t + n_t;
    const double    norm2 = n_avg.squaredNorm();
    // Check for degenerate case: if the averaged normal is too small, return zero error
    constexpr double k_degenerate_threshold = 1e-6;
    if (norm2 < k_degenerate_threshold) {
      return 0.0;
    }
    n_avg /= std::sqrt(norm2);

    const double err = n_avg.dot(R * p_s + t - p_t);
    return 0.5 * err * err;
  }

  /* helpers --------------------------------------------------------------- */
  bool  inlier()       const { return target_index != std::numeric_limits<std::size_t>::max(); }
  void  reset_inlier()       { target_index  = std::numeric_limits<std::size_t>::max();
                               source_index  = std::numeric_limits<std::size_t>::max(); }

  std::size_t target_index;
  std::size_t source_index;
};


}  // namespace small_gicp
