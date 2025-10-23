/*
 * helpers.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/helpers.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include <open3d/Open3D.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
// #include <tbb/blocked_range.h>
// #include <tbb/concurrent_unordered_map.h>
// #include <tbb/parallel_reduce.h>
#include <algorithm>  // Required for std::transform
#include <execution>  // Required for parallel STL

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;

class AccumulatedPoint {
 public:
  void AddPoint(const open3d::geometry::PointCloud& cloud, int index) {
    point_ += cloud.points_[index];
    if (cloud.HasNormals()) {
      if (!std::isnan(cloud.normals_[index](0)) && !std::isnan(cloud.normals_[index](1)) && !std::isnan(cloud.normals_[index](2))) {
        normal_ += cloud.normals_[index];
      }
    }

    if (cloud.HasColors() && isValidColor(cloud.colors_[index])) {
      color_ = cloud.colors_[index];  //+= cloud.colors_[index];
    }

    if (cloud.HasCovariances()) {
      covariance_ += cloud.covariances_[index];
    }

    num_of_points_++;
  }

  Eigen::Vector3d GetAveragePoint() const { return point_ / double(num_of_points_); }

  Eigen::Vector3d GetAverageNormal() const {
    // Call NormalizeNormals() afterwards if necessary
    return normal_ / double(num_of_points_);
  }

  Eigen::Vector3d GetAverageColor() const {
    return color_;  // / double(num_of_points_);
  }

  Eigen::Matrix3d GetAverageCovariance() const {
    // Call NormalizeNormals() afterwards if necessary
    return covariance_ / double(num_of_points_);
  }

  void Merge(const AccumulatedPoint& other) {
    point_ += other.point_;
    normal_ += other.normal_;
    // For color: keep latest, or weighted average. Here: keep if other has any points
    if (other.num_of_points_ > 0) color_ = other.color_;  // You can change logic if you want sum/mean.
    covariance_ += other.covariance_;
    num_of_points_ += other.num_of_points_;
  }

 public:
  int num_of_points_ = 0;
  Eigen::Vector3d point_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d color_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Zero();
};

class point_cubic_id {
 public:
  size_t point_id;
  int cubic_id;
};

}  // namespace

bool isValidColor(const Eigen::Vector3d& c) {
  return (c.array().all() >= 0.0) && (c.array().all() <= 1.0);
}

double informationMatrixMaxCorrespondenceDistance(double mappingVoxelSize) {
  return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (1.5 * mappingVoxelSize);
}

double icpMaxCorrespondenceDistance(double mappingVoxelSize) {
  return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (2.0 * mappingVoxelSize);
}

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud* pcl) {
  open3d::geometry::KDTreeSearchParamKNN param(numNearestNeighbours);
  pcl->EstimateNormals(param);
}

void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud* pcl) {
  if (downSamplingRatio >= 1.0) {
    return;
  }
  auto downSampled = pcl->RandomDownSample(downSamplingRatio);
  *pcl = std::move(*downSampled);
}

void voxelize(double voxelSize, open3d::geometry::PointCloud* pcl) {
  if (voxelSize <= 0) {
    return;
  }
  auto voxelized = pcl->VoxelDownSample(voxelSize);
  *pcl = std::move(*voxelized);
  return;
}

Eigen::Vector3d getColorFromIndex(size_t idx, size_t total) {
  // Evenly spaced HSV, convert to RGB
  double h = (static_cast<double>(idx) / std::max(total, size_t(1)));
  double s = 0.85, v = 0.85;

  // HSV to RGB
  double r, g, b;
  int i = int(h * 6);
  double f = h * 6 - i;
  double p = v * (1 - s);
  double q = v * (1 - f * s);
  double t = v * (1 - (1 - f) * s);
  switch (i % 6) {
    case 0:
      r = v, g = t, b = p;
      break;
    case 1:
      r = q, g = v, b = p;
      break;
    case 2:
      r = p, g = v, b = t;
      break;
    case 3:
      r = p, g = q, b = v;
      break;
    case 4:
      r = t, g = p, b = v;
      break;
    case 5:
      r = v, g = p, b = q;
      break;
  }
  return Eigen::Vector3d(r, g, b);
}

std::shared_ptr<open3d::geometry::PointCloud> voxelizeWithinCroppingVolume(double voxel_size, const CroppingVolume& croppingVolume,
                                                                           const open3d::geometry::PointCloud& cloud) {
  using namespace open3d::geometry;
  using Vec3i = Eigen::Vector3i;
  using PointCloudPtr = std::shared_ptr<PointCloud>;

  constexpr size_t parallelThreshold = 10000;  // auto-tuning threshold
  PointCloudPtr output = std::make_shared<PointCloud>();

  if (voxel_size <= 0.0) {
    *output = cloud;
    return output;
  }

  const Eigen::Vector3d voxelSize = Eigen::Vector3d::Constant(voxel_size);
  const InverseVoxelSize invVoxelSize = fromVoxelSize(voxelSize);

  const bool has_normals = cloud.HasNormals();
  const bool has_colors = cloud.HasColors();
  const bool has_covariances = cloud.HasCovariances();

  output->points_.reserve(cloud.points_.size());
  if (has_normals) output->normals_.reserve(cloud.points_.size());
  if (has_colors) output->colors_.reserve(cloud.points_.size());
  if (has_covariances) output->covariances_.reserve(cloud.points_.size());

  if (cloud.points_.size() < parallelThreshold) {
    std::unordered_map<Vec3i, AccumulatedPoint, o3d_slam::EigenVec3iHash> voxel_map;

    for (size_t i = 0; i < cloud.points_.size(); ++i) {
      const auto& pt = cloud.points_[i];
      if (croppingVolume.isWithinVolume(pt)) {
        const Vec3i voxelIdx = getVoxelIdx(pt, invVoxelSize);
        voxel_map[voxelIdx].AddPoint(cloud, static_cast<int>(i));
      } else {
        output->points_.emplace_back(pt);
        if (has_normals) output->normals_.emplace_back(cloud.normals_[i]);
        if (has_colors) output->colors_.emplace_back(cloud.colors_[i]);
        if (has_covariances) output->covariances_.emplace_back(cloud.covariances_[i]);
      }
    }

    for (const auto& kv : voxel_map) {
      const auto& acc = kv.second;
      output->points_.emplace_back(acc.GetAveragePoint());
      if (has_normals) output->normals_.emplace_back(acc.GetAverageNormal().normalized());
      if (has_colors) output->colors_.emplace_back(acc.GetAverageColor());
      if (has_covariances) output->covariances_.emplace_back(acc.GetAverageCovariance());
    }

  } else {
    using VoxelMap = std::unordered_map<Vec3i, AccumulatedPoint, o3d_slam::EigenVec3iHash>;
    const long N = static_cast<long>(cloud.points_.size());
    int max_threads = omp_get_max_threads();

    // Thread-local voxel maps and passthrough indices
    std::vector<VoxelMap> thread_voxel_maps(max_threads);
    std::vector<std::vector<size_t>> thread_passthrough_idxs(max_threads);

#pragma omp parallel
    {
      int tid = omp_get_thread_num();
      auto& voxel_map = thread_voxel_maps[tid];
      auto& passthrough = thread_passthrough_idxs[tid];

#pragma omp for schedule(static)
      for (long i = 0; i < N; ++i) {
        const auto& pt = cloud.points_[i];
        if (croppingVolume.isWithinVolume(pt)) {
          const Vec3i voxelIdx = getVoxelIdx(pt, invVoxelSize);
          voxel_map[voxelIdx].AddPoint(cloud, static_cast<int>(i));
        } else {
          passthrough.push_back(i);
        }
      }
    }

    // Merge thread-local voxel maps into a single map
    VoxelMap merged_voxel_map;
    for (auto& local_map : thread_voxel_maps) {
      for (auto& kv : local_map) {
        merged_voxel_map[kv.first].Merge(kv.second);  // Merge AccumulatedPoint logic
      }
    }

    // Collect all passthrough indices
    std::vector<size_t> passthrough_idxs;
    size_t total_passthrough = 0;
    for (const auto& v : thread_passthrough_idxs) total_passthrough += v.size();
    passthrough_idxs.reserve(total_passthrough);
    for (const auto& v : thread_passthrough_idxs) passthrough_idxs.insert(passthrough_idxs.end(), v.begin(), v.end());

    // Output assembly (could parallelize this if needed)
    for (const auto& kv : merged_voxel_map) {
      const auto& acc = kv.second;
      output->points_.emplace_back(acc.GetAveragePoint());
      if (has_normals) output->normals_.emplace_back(acc.GetAverageNormal().normalized());
      if (has_colors) output->colors_.emplace_back(acc.GetAverageColor());
      if (has_covariances) output->covariances_.emplace_back(acc.GetAverageCovariance());
    }
    for (size_t i : passthrough_idxs) {
      output->points_.emplace_back(cloud.points_[i]);
      if (has_normals) output->normals_.emplace_back(cloud.normals_[i]);
      if (has_colors) output->colors_.emplace_back(cloud.colors_[i]);
      if (has_covariances) output->covariances_.emplace_back(cloud.covariances_[i]);
    }
  }

  return output;
}

std::pair<std::vector<double>, std::vector<size_t>> computePointCloudDistance(const open3d::geometry::PointCloud& reference,
                                                                              const open3d::geometry::PointCloud& cloud,
                                                                              const std::vector<size_t>& idsInReference) {
  std::vector<double> distances(idsInReference.size());
  std::vector<int> indices(idsInReference.size());
  open3d::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(cloud);  // fast cca 1 ms

#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < idsInReference.size(); i++) {
    const size_t idx = idsInReference[i];
    const int knn = 1;
    std::vector<int> ids(knn);
    std::vector<double> dists(knn);
    //			if (kdtree.SearchHybrid(reference.points_[idx], 2.0, knn, ids, dists) != 0) {
    if (kdtree.SearchKNN(reference.points_[idx], knn, ids, dists) != 0) {
      distances[i] = std::sqrt(dists[0]);
      indices[i] = idx;
    } else {
      distances[i] = -1.0;
      indices[i] = -1;
      //				std::cout << "could not find a nearest neighbour \n";
    }
  }  // end for

  // remove distances/ids for which no neighbor was found
  std::vector<double> distsRet;
  distsRet.reserve(distances.size());
  std::copy_if(distances.begin(), distances.end(), std::back_inserter(distsRet), [](double x) { return x >= 0; });
  std::vector<size_t> idsRet;
  idsRet.reserve(indices.size());
  std::copy_if(indices.begin(), indices.end(), std::back_inserter(idsRet), [](int x) { return x >= 0; });
  return {distsRet, idsRet};
}

void removeByIds(const std::vector<size_t>& ids, open3d::geometry::PointCloud* cloud) {
  if (ids.empty()) {
    return;
  }
  const bool isInvertSelection = true;
  auto trimmedCloud = cloud->SelectByIndex(ids, isInvertSelection);
  *cloud = std::move(*trimmedCloud);
}

std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud& scan, const open3d::geometry::PointCloud& cloud,
                                          const Eigen::Vector3d& sensorPosition, const SpaceCarvingParameters& param) {
  std::vector<size_t> subsetIdxs(cloud.points_.size(), 0);
  std::iota(subsetIdxs.begin(), subsetIdxs.end(), 0);
  return getIdxsOfCarvedPoints(scan, cloud, sensorPosition, subsetIdxs, param);
}

std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud& scan, const open3d::geometry::PointCloud& cloud,
                                          const Eigen::Vector3d& sensorPosition, const std::vector<size_t>& cloudIdxsSubset,
                                          const SpaceCarvingParameters& param) {
  const double stepSize = param.voxelSize_;
  const std::string layer = "layer";
  VoxelMap voxelMap(Eigen::Vector3d::Constant(param.voxelSize_));
  voxelMap.insertCloud(layer, cloud, cloudIdxsSubset);
  std::unordered_set<size_t> setOfIdsToRemove;
  setOfIdsToRemove.reserve(scan.points_.size());
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < scan.points_.size(); ++i) {
    const Eigen::Vector3d& p = scan.points_[i];
    const double length = (p - sensorPosition).norm();
    const Eigen::Vector3d direction = (p - sensorPosition) / length;
    double distance = 0.0;
    const double maximalPathTraveled = std::max(param.voxelSize_, std::min(length - param.truncationDistance_, param.maxRaytracingLength_));
    while (distance < maximalPathTraveled) {
      const Eigen::Vector3d currentPosition = distance * direction + sensorPosition;
      auto ids = voxelMap.getIndicesInVoxel(layer, currentPosition);
      for (const auto id : ids) {
        bool isRemoveId = true;
        if (cloud.HasNormals()) {
          const auto n = cloud.normals_[id].normalized();
          isRemoveId = std::abs(direction.dot(n)) > param.minDotProductWithNormal_;
        }
        if (isRemoveId) {
#pragma omp critical
          setOfIdsToRemove.insert(id);
        }
      }
      distance += stepSize;
    }
  }
  std::vector<size_t> vecOfIdsToRemove;
  vecOfIdsToRemove.insert(vecOfIdsToRemove.end(), setOfIdsToRemove.begin(), setOfIdsToRemove.end());
  return vecOfIdsToRemove;
}

std::shared_ptr<open3d::geometry::PointCloud> transform(const Eigen::Matrix4d& T, const open3d::geometry::PointCloud& cloud) {
  using namespace open3d::geometry;

  auto out = std::make_shared<PointCloud>();
  const bool is_identity = (T - Eigen::Matrix4d::Identity()).array().abs().maxCoeff() < 1e-4;
  if (is_identity) {
    *out = cloud;
    return out;
  }

  const bool has_normals = cloud.HasNormals();
  const bool has_covariances = cloud.HasCovariances();
  const bool has_colors = cloud.HasColors();

  out->points_.resize(cloud.points_.size());
  if (has_normals) out->normals_.resize(cloud.points_.size());
  if (has_covariances) out->covariances_.resize(cloud.points_.size());
  if (has_colors) out->colors_ = cloud.colors_;  // Direct copy (unchanged by transform)

  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);

  // Transform points
  std::transform(std::execution::par_unseq, cloud.points_.begin(), cloud.points_.end(), out->points_.begin(),
                 [&](const Eigen::Vector3d& p) { return R * p + t; });

  // Normals
  if (has_normals) {
    std::transform(std::execution::par_unseq, cloud.normals_.begin(), cloud.normals_.end(), out->normals_.begin(),
                   [&](const Eigen::Vector3d& n) { return (R * n).normalized(); });
  }

  // Covariances
  if (has_covariances) {
    std::transform(std::execution::par_unseq, cloud.covariances_.begin(), cloud.covariances_.end(), out->covariances_.begin(),
                   [&](const Eigen::Matrix3d& cov) { return R * cov * R.transpose(); });
  }

  return out;
}

void computeIndicesOfOverlappingPoints(const open3d::geometry::PointCloud& source, const open3d::geometry::PointCloud& target,
                                       const Transform& sourceToTarget, double voxelSize, size_t minNumPointsPerVoxel,
                                       std::vector<size_t>* idxsSource, std::vector<size_t>* idxsTarget) {
  assert_ge<size_t>(minNumPointsPerVoxel, 1);
  const std::string targetLayer = "target";
  const std::string sourceLayer = "source";
  VoxelMap voxelMap(Eigen::Vector3d::Constant(voxelSize));
  voxelMap.insertCloud(targetLayer, target);
  auto sourceTransformed = source;
  sourceTransformed.Transform(sourceToTarget.matrix());
  voxelMap.insertCloud(sourceLayer, sourceTransformed);
  idxsSource->clear();
  idxsSource->reserve(source.points_.size());
  idxsTarget->clear();
  idxsTarget->reserve(target.points_.size());
  const auto& voxels = voxelMap.voxels_;
  for (auto it = voxels.cbegin(); it != voxels.cend(); ++it) {
    const auto voxelKey = it->first;
    const auto sourceIdxs = voxelMap.getIndicesInVoxel(sourceLayer, voxelKey);
    const auto targetIdxs = voxelMap.getIndicesInVoxel(targetLayer, voxelKey);
    if (sourceIdxs.size() >= minNumPointsPerVoxel && targetIdxs.size() >= minNumPointsPerVoxel) {
      idxsTarget->insert(idxsTarget->end(), targetIdxs.begin(), targetIdxs.end());
      idxsSource->insert(idxsSource->end(), sourceIdxs.begin(), sourceIdxs.end());
    }
  }
}

Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud& cloud, const std::vector<size_t>& idxs) {
  assert_gt<size_t>(idxs.size(), 0, "you're trying to compute center of a empty pointcloud");
  Eigen::Vector3d center(0.0, 0.0, 0.0);
  for (const auto idx : idxs) {
    center += cloud.points_.at(idx);
  }
  return center / static_cast<double>(idxs.size());
}

// Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud& cloud, const std::vector<size_t>& idxs) {
//   assert(!idxs.empty() && "Trying to compute center of an empty point set.");

//   struct Accumulator {
//     const open3d::geometry::PointCloud& cloud_;
//     const std::vector<size_t>& idxs_;
//     Eigen::Vector3d sum_;
//     size_t count_;

//     Accumulator(const open3d::geometry::PointCloud& cloud, const std::vector<size_t>& idxs)
//         : cloud_(cloud), idxs_(idxs), sum_(Eigen::Vector3d::Zero()), count_(0) {}

//     Accumulator(Accumulator& other, tbb::split) : cloud_(other.cloud_), idxs_(other.idxs_), sum_(Eigen::Vector3d::Zero()), count_(0) {}

//     void operator()(const tbb::blocked_range<size_t>& range) {
//       for (size_t i = range.begin(); i < range.end(); ++i) {
//         sum_ += cloud_.points_[idxs_[i]];
//         ++count_;
//       }
//     }

//     void join(const Accumulator& rhs) {
//       sum_ += rhs.sum_;
//       count_ += rhs.count_;
//     }
//   };

//   Accumulator body(cloud, idxs);
//   tbb::parallel_reduce(tbb::blocked_range<size_t>(0, idxs.size()), body);

//   return body.sum_ / static_cast<double>(body.count_);
// }

// Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud& cloud, const std::vector<size_t>& idxs) {
//   assert(!idxs.empty() && "You're trying to compute center of an empty pointcloud.");
//   Eigen::Vector3d center(0.0, 0.0, 0.0);

// #pragma omp parallel
//   {
//     Eigen::Vector3d local_center(0.0, 0.0, 0.0);
// #pragma omp for schedule(static)
//     for (int i = 0; i < static_cast<int>(idxs.size()); ++i) {
//       local_center += cloud.points_[idxs[i]];
//     }
// #pragma omp critical
//     center += local_center;
//   }

//   return center / static_cast<double>(idxs.size());
// }

double getMapVoxelSize(const MapBuilderParameters& p, double valueIfZero) {
  return std::abs(p.mapVoxelSize_) <= 1e-3 ? valueIfZero : p.mapVoxelSize_;
}

std::vector<Eigen::Vector3i> getKeysOfCarvedPoints(const open3d::geometry::PointCloud& scan, const VoxelizedPointCloud& cloud,
                                                   const Eigen::Vector3d& sensorPosition, const SpaceCarvingParameters& param) {
  const double stepSize = 2.0 * param.neighborhoodRadiusDenseMap_;
  std::unordered_set<Eigen::Vector3i, EigenVec3iHash> setOfIdsToRemove;
  setOfIdsToRemove.reserve(scan.points_.size());
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < scan.points_.size(); ++i) {
    const Eigen::Vector3d& p = scan.points_[i];
    const double length = (p - sensorPosition).norm();
    const Eigen::Vector3d direction = (p - sensorPosition) / length;
    double distance = 0.0;
    const double maximalPathTraveled = std::max(stepSize, std::min(length - param.truncationDistance_, param.maxRaytracingLength_));
    while (distance < maximalPathTraveled) {
      const Eigen::Vector3d currentPosition = distance * direction + sensorPosition;
      const auto voxelsToBeFlushed =
          getVoxelsWithinPointNeighborhood(currentPosition, param.neighborhoodRadiusDenseMap_, cloud.getVoxelSize());
      // todo also check the dot product
      for (const auto& key : voxelsToBeFlushed) {
        if (cloud.hasVoxelWithKey(key)) {
#pragma omp critical
          setOfIdsToRemove.insert(key);
        }
      }

      distance += stepSize;
    }
  }
  std::vector<Eigen::Vector3i> vecOfIdsToRemove;
  vecOfIdsToRemove.insert(vecOfIdsToRemove.end(), setOfIdsToRemove.begin(), setOfIdsToRemove.end());
  return vecOfIdsToRemove;
}

Eigen::Vector3d computeCenter(const VoxelizedPointCloud& voxels) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  int n = 0;
  for (const auto& voxel : voxels.voxels_) {
    if (voxel.second.numAggregatedPoints_ > 0) {
      center += voxel.second.getAggregatedPosition();
      ++n;
    }
  }
  return center / static_cast<double>(n + 1e-6);
}

std::shared_ptr<PointCloud> removePointsWithNonFiniteValues(const PointCloud& cloud) {
  std::shared_ptr<PointCloud> filtered = std::make_shared<PointCloud>();
  *filtered = cloud;
  filtered->RemoveNonFinitePoints();
  return filtered;
}

} /* namespace o3d_slam */
