#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <cstring>  // for std::memset

namespace small_gicp {

#ifndef _OPENMP
#warning "OpenMP is not available. Parallel reduction will be disabled."
inline int omp_get_thread_num() {
  return 0;
}
#endif

/// @brief Parallel reduction with OpenMP backend (SIMD-aligned)
struct ParallelReductionOMP {
  ParallelReductionOMP() : num_threads(8) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree, typename CorrespondenceRejector, typename Factor>
  std::tuple<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>, double> linearize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const CorrespondenceRejector& rejector,
    const Eigen::Isometry3d& T,
    std::vector<Factor>& factors) const {

    using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    // Aligned storage
    std::vector<Matrix6d, Eigen::aligned_allocator<Matrix6d>> Hs(num_threads, Matrix6d::Zero());
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> bs(num_threads, Vector6d::Zero());
    std::vector<double> es(num_threads, 0.0);

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(factors.size()); i++) {
      Eigen::Matrix<double, 6, 6> H_i;
      Eigen::Matrix<double, 6, 1> b_i;
      double e_i;

      if (!factors[i].linearize(target, source, target_tree, T, i, rejector, &H_i, &b_i, &e_i)) {
        continue;
      }

      const int tid = omp_get_thread_num();
      Hs[tid].noalias() += H_i;
      bs[tid].noalias() += b_i;
      es[tid] += e_i;
    }

    // Combine all thread-local accumulations
    for (int i = 1; i < num_threads; ++i) {
      Hs[0].noalias() += Hs[i];
      bs[0].noalias() += bs[i];
      es[0] += es[i];
    }

    return {Hs[0], bs[0], es[0]};
  }

  template <typename TargetPointCloud, typename SourcePointCloud, typename Factor>
  double error(const TargetPointCloud& target,
               const SourcePointCloud& source,
               const Eigen::Isometry3d& T,
               std::vector<Factor>& factors) const {
    double sum_e = 0.0;

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8) reduction(+ : sum_e)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(factors.size()); ++i) {
      sum_e += factors[i].error(target, source, T);
    }
    return sum_e;
  }

  int num_threads;
};

}  // namespace small_gicp
