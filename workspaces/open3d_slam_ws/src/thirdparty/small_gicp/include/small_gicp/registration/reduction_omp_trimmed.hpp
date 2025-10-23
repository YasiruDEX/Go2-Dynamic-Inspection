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
struct ParallelReductionOMPTrimmed {
  double trim_ratio; // e.g., 0.7 = keep 70% best inliers

  ParallelReductionOMPTrimmed(int threads = 8, double trim = 0.95) : num_threads(threads), trim_ratio(trim) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree, typename CorrespondenceRejector, typename Factor>
  std::tuple<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>, double> linearize(
      const TargetPointCloud& target,
      const SourcePointCloud& source,
      const TargetTree& target_tree,
      const CorrespondenceRejector& rejector,
      const Eigen::Isometry3d& T,
      std::vector<Factor>& factors) const
  {
      struct TrimmedEntry {
          double error;
          Eigen::Matrix<double, 6, 6> H;
          Eigen::Matrix<double, 6, 1> b;
          size_t idx;
          bool valid;
      };

      std::vector<TrimmedEntry> entries(factors.size());

      #pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
      for (std::int64_t i = 0; i < static_cast<std::int64_t>(factors.size()); ++i) {
          Eigen::Matrix<double, 6, 6> H_i;
          Eigen::Matrix<double, 6, 1> b_i;
          double e_i;
          TrimmedEntry entry;
          if (factors[i].linearize(target, source, target_tree, T, i, rejector, &H_i, &b_i, &e_i)) {
              entry.error = e_i;
              entry.H = H_i;
              entry.b = b_i;
              entry.idx = i;
              entry.valid = true;
          } else {
              entry.valid = false;
          }
          entries[i] = entry;
      }

      std::vector<TrimmedEntry*> valid_entries;
      for (auto& entry : entries)
          if (entry.valid)
              valid_entries.push_back(&entry);

      std::sort(valid_entries.begin(), valid_entries.end(),
          [](const TrimmedEntry* a, const TrimmedEntry* b) { return a->error < b->error; });

      size_t num_trimmed = static_cast<size_t>(trim_ratio * valid_entries.size());
      if (num_trimmed == 0) num_trimmed = 1;

      Eigen::Matrix<double, 6, 6> H_total = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b_total = Eigen::Matrix<double, 6, 1>::Zero();
      double e_total = 0.0;

      for (size_t i = 0; i < num_trimmed; ++i) {
          H_total.noalias() += valid_entries[i]->H;
          b_total.noalias() += valid_entries[i]->b;
          e_total += valid_entries[i]->error;
      }

      return {H_total, b_total, e_total};
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
