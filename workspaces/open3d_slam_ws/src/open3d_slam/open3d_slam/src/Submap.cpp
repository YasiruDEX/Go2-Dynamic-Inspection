/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Submap.hpp"
#include <Eigen/Core>
#include "open3d_slam/assert.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam/typedefs.hpp"

#include <omp.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <thread>
#include <utility>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const std::string voxelMapLayer = "layer";
}  // namespace

Submap::Submap(size_t id, size_t parentId) : id_(id), parentId_(parentId) {
  update(params_);
}

size_t Submap::getId() const {
  return id_;
}

size_t Submap::getParentId() const {
  return parentId_;
}

o3d_slam::Submap::~Submap() {
  if (voxelizationFuture_.valid()) {
    voxelizationFuture_.wait();  // Wait for async voxelization to complete
  }
}

bool Submap::insertScan(const PointCloud& rawScan, const PointCloud& preProcessedScan, const Transform& mapToRangeSensor, const Time& time,
                        bool /*isPerformCarving*/) {
  if (preProcessedScan.IsEmpty()) return true;

  mapToRangeSensor_ = mapToRangeSensor;

  if (params_.isUseInitialMap_ && mapCloud_.IsEmpty()) {
    std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
    mapCloud_ = preProcessedScan;
    voxelize(params_.mapBuilder_.mapVoxelSize_, &mapCloud_);
    return true;
  }

  const auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), preProcessedScan);

  {
    std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
    mapCloud_ += *transformedCloud;
    mapBuilderCropper_->setPose(mapToRangeSensor);
  }

  // // Print the state of carvingEnabled and its components in a bright orange block
  // std::cout << "\033[38;5;208m[Carving Status]\n"
  //           << "  params_.isCarvingEnabled_: " << (params_.isCarvingEnabled_ ? "true" : "false") << "\n"
  //           << "  nScansInsertedMap_: " << nScansInsertedMap_ << "\n"
  //           << "  carveSpaceEveryNscans_: " << params_.mapBuilder_.carving_.carveSpaceEveryNscans_ << "\n"
  //           << "  (nScansInsertedMap_ % carveSpaceEveryNscans_ == 0): "
  //           << ((nScansInsertedMap_ % params_.mapBuilder_.carving_.carveSpaceEveryNscans_ == 0) ? "true" : "false") << "\n"
  //           << "  carvingEnabled: "
  //           << ((params_.isCarvingEnabled_ && (nScansInsertedMap_ % params_.mapBuilder_.carving_.carveSpaceEveryNscans_ == 0)) ? "true"
  //                                                                                                                              : "false")
  //           << "\033[0m" << std::endl;

  const bool carvingEnabled = params_.isCarvingEnabled_ && (nScansInsertedMap_ % params_.mapBuilder_.carving_.carveSpaceEveryNscans_ == 0);

  if (carvingEnabled) {
    carvingStatisticsTimer_.startStopwatch();
    {
      std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
      carve(rawScan, mapToRangeSensor, *mapBuilderCropper_, params_.mapBuilder_.carving_, &mapCloud_);
    }
    const double elapsedMs = carvingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
    carvingStatisticsTimer_.addMeasurementMsec(elapsedMs);
    if (params_.isPrintTimingStatistics_) {
      std::cout << "Space carving took: \033[92m" << elapsedMs << " ms\033[0m\n";
    }
  }

  const int voxelizeEvery = std::max(1, voxelizeEveryNscans_);
  if (nScansInsertedMap_ % voxelizeEvery == 0) {
    bool expected = false;
    if (voxelizationRunning_.compare_exchange_strong(expected, true)) {
      PointCloudPtr mapCopy;
      {
        std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
        mapCopy = std::make_shared<PointCloud>(mapCloud_);
      }

      auto cropperCopy = *mapBuilderCropper_;
      auto voxelParams = params_.mapBuilder_;

      voxelizationFuture_ = std::async(std::launch::async, [this, mapCopy, cropperCopy, voxelParams]() mutable {
        auto voxelized = voxelizeWithinCroppingVolume(voxelParams.mapVoxelSize_, cropperCopy, *mapCopy);
        {
          std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
          mapCloud_ = std::move(*voxelized);
        }
        voxelizationRunning_ = false;
      });
    }
  }

  ++nScansInsertedMap_;
  return true;
}

bool Submap::insertScanDenseMap(const PointCloud& rawScan, const Transform& mapToRangeSensor, const Time& time, bool isPerformCarving) {
  denseMapCropper_->setPose(Transform::Identity());
  auto cropped = denseMapCropper_->crop(rawScan);
  auto validColors = colorCropper_.crop(*cropped);
  auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), *validColors);
  {
    std::lock_guard<std::mutex> lck(denseMapMutex_);
    denseMap_.insert(*transformedCloud);
  }
  if (isPerformCarving) {
    std::lock_guard<std::mutex> lck(denseMapMutex_);
    carve(rawScan, mapToRangeSensor.translation(), params_.denseMapBuilder_.carving_, &denseMap_);
  }
  ++nScansInsertedDenseMap_;
  return true;
}

void Submap::transform(const Transform& T) {
  const Eigen::Matrix4d mat(T.matrix());
  sparseMapCloud_.Transform(mat);
  {
    std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
    mapCloud_.Transform(mat);
  }
  {
    std::lock_guard<std::mutex> lck(denseMapMutex_);
    denseMap_.transform(T);
  }
  mapToRangeSensor_ = mapToRangeSensor_ * T;
  submapCenter_ = T * submapCenter_;
}

void Submap::carve(const PointCloud& rawScan, const Transform& mapToRangeSensor, const CroppingVolume& cropper,
                   const SpaceCarvingParameters& params, PointCloud* map) {
  if (map->points_.empty() || !(nScansInsertedMap_ % params.carveSpaceEveryNscans_ == 0)) {
    return;
  }
  //	Timer timer("carving");
  auto scan = o3d_slam::transform(mapToRangeSensor.matrix(), rawScan);
  //	auto croppedScan = removeDuplicatePointsWithinSameVoxels(*scan, Eigen::Vector3d::Constant(params_.mapBuilder_.mapVoxelSize_));
  const auto wideCroppedIdxs = cropper.getIndicesWithinVolume(*map);
  auto idxsToRemove = std::move(getIdxsOfCarvedPoints(*scan, *map, mapToRangeSensor.translation(), wideCroppedIdxs, params));
  toRemove_ = std::move(*(map->SelectByIndex(idxsToRemove)));
  scanRef_ = std::move(*scan);
  std::cout << "\033[91mWould remove: " << idxsToRemove.size() << "\033[0m" << std::endl;
  removeByIds(idxsToRemove, map);
}

void Submap::carve(const PointCloud& scan, const Eigen::Vector3d& sensorPosition, const SpaceCarvingParameters& param,
                   VoxelizedPointCloud* cloud) {
  if (cloud->empty() || !(nScansInsertedDenseMap_ % param.carveSpaceEveryNscans_ == 1)) {
    return;
  }
  const PointCloudPtr croppedScanPtr =
      removeDuplicatePointsWithinSameVoxels(scan, Eigen::Vector3d::Constant(params_.denseMapBuilder_.mapVoxelSize_));
  std::vector<Eigen::Vector3i> keysToRemove = getKeysOfCarvedPoints(*croppedScanPtr, *cloud, sensorPosition, param);
  for (const auto& k : keysToRemove) {
    cloud->removeKey(k);
  }
}

void Submap::voxelizeInsideCroppingVolume(const CroppingVolume& cropper, const MapBuilderParameters& param, PointCloud* map) const {
  if (param.mapVoxelSize_ > 0.0) {
    //			Timer timer("voxelize_map",true);
    auto voxelizedMap = voxelizeWithinCroppingVolume(param.mapVoxelSize_, cropper, *map);
    *map = *voxelizedMap;
  } else {
    std::cerr << "Map voxel size is zero. Not voxelizing the map." << std::endl;
  }
}

void Submap::setParameters(const MapperParameters& mapperParams) {
  params_ = mapperParams;
  update(mapperParams);
}

Submap::Submap(const Submap& other) : Submap(other.id_, other.parentId_) {
  colorCropper_ = other.colorCropper_;
  denseMap_ = other.denseMap_;
  voxelMap_ = other.voxelMap_;
  scanCounter_ = other.scanCounter_;
  carvingStatisticsTimer_ = other.carvingStatisticsTimer_;
  parentId_ = other.parentId_;
  isCenterComputed_ = other.isCenterComputed_;
  id_ = other.id_;
  feature_ = other.feature_;
  nScansInsertedDenseMap_ = other.nScansInsertedDenseMap_;
  nScansInsertedMap_ = other.nScansInsertedMap_;
  featureTimer_ = other.featureTimer_;
  params_ = other.params_;
  denseMapCropper_ = other.denseMapCropper_;
  mapBuilderCropper_ = other.mapBuilderCropper_;
  submapCenter_ = other.submapCenter_;
  mapToRangeSensor_ = other.mapToRangeSensor_;
  mapToSubmap_ = other.mapToSubmap_;
  mapCloud_ = other.mapCloud_;
  sparseMapCloud_ = other.sparseMapCloud_;

  //	update(params_);
}

const Transform& Submap::getMapToSubmapOrigin() const {
  return mapToSubmap_;
}

Eigen::Vector3d Submap::getMapToSubmapCenter() const {
  return isCenterComputed_ ? submapCenter_ : mapToSubmap_.translation();
}

const Submap::PointCloud& Submap::getMapPointCloud() const {
  return mapCloud_;
}
PointCloud Submap::getMapPointCloudCopy() const {
  std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
  auto copy = mapCloud_;
  return std::move(copy);
}
const VoxelizedPointCloud& Submap::getDenseMap() const {
  return denseMap_;
}

VoxelizedPointCloud Submap::getDenseMapCopy() const {
  std::lock_guard<std::mutex> lck(denseMapMutex_);
  auto copy = denseMap_;
  return std::move(copy);
}

const Submap::PointCloud& Submap::getSparseMapPointCloud() const {
  return sparseMapCloud_;
}

void Submap::setMapToSubmapOrigin(const Transform& T) {
  mapToSubmap_ = T;
}

void Submap::update(const MapperParameters& p) {
  mapBuilderCropper_ = croppingVolumeFactory(p.mapBuilder_.cropper_);
  denseMapCropper_ = croppingVolumeFactory(p.denseMapBuilder_.cropper_);
  denseMap_ = std::move(VoxelizedPointCloud(Eigen::Vector3d::Constant(p.denseMapBuilder_.mapVoxelSize_)));

  voxelMap_ =
      std::move(VoxelMap(Eigen::Vector3d::Constant(magic::voxelExpansionFactorAdjacencyBasedRevisiting * p.mapBuilder_.mapVoxelSize_)));
}

bool Submap::isEmpty() const {
  return mapCloud_.points_.empty();
}

std::size_t Submap::getNbPoints() const {
  return mapCloud_.points_.size();
}

const VoxelMap& Submap::getVoxelMap() const {
  return voxelMap_;
}

void Submap::computeFeatures() {
  if (feature_ != nullptr && featureTimer_.elapsedSec() < params_.submaps_.minSecondsBetweenFeatureComputation_) {
    return;
  }

  std::thread computeVoxelMapThread([this]() {
    //		Timer t("compute_voxel_submap");
    voxelMap_.clear();
    voxelMap_.insertCloud(voxelMapLayer, mapCloud_);
  });

  auto mapCopy = getMapPointCloudCopy();
  const auto& p = params_.placeRecognition_;
  sparseMapCloud_ = *(mapCopy.VoxelDownSample(p.featureVoxelSize_));
  sparseMapCloud_.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(p.normalEstimationRadius_, p.normalKnn_));
  sparseMapCloud_.NormalizeNormals();
  sparseMapCloud_.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
  feature_ = registration::ComputeFPFHFeature(sparseMapCloud_, open3d::geometry::KDTreeSearchParamHybrid(p.featureRadius_, p.featureKnn_));
  computeVoxelMapThread.join();
  featureTimer_.reset();
}

const Submap::Feature& Submap::getFeatures() const {
  assert_nonNullptr(feature_, "Feature ptr is nullptr");
  return *feature_;
}

void Submap::computeSubmapCenter() {
  auto mapCopy = getMapPointCloudCopy();
  submapCenter_ = mapCopy.GetCenter();
  // submapCenter_ = ComputeCenterCustom(mapCopy.points_);

  isCenterComputed_ = true;
}

Eigen::Vector3d Submap::ComputeCenterCustom(const std::vector<Eigen::Vector3d>& points) {
  if (points.empty()) return Eigen::Vector3d::Zero();

  const size_t N = points.size();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();

  int n_threads = 1;
#pragma omp parallel
  {
    int tid = omp_get_thread_num();
#pragma omp single
    n_threads = omp_get_num_threads();
  }

  std::vector<Eigen::Vector3d> local_sums(n_threads, Eigen::Vector3d::Zero());

#pragma omp parallel
  {
    int tid = omp_get_thread_num();
    Eigen::Vector3d& local_sum = local_sums[tid];

#pragma omp for schedule(static)
    for (int i = 0; i < static_cast<int>(N); ++i) {
      local_sum += points[i];
    }
  }

  for (int i = 0; i < n_threads; ++i) {
    sum += local_sums[i];
  }

  return sum / static_cast<double>(N);
}

Eigen::Vector3d Submap::ComputeCenterSIMD(const std::vector<Eigen::Vector3d>& points) {
  if (points.empty()) return Eigen::Vector3d::Zero();

  // Map the raw data into a Matrix3Xd
  const double* raw_ptr = reinterpret_cast<const double*>(points.data());
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor>> mat(raw_ptr, 3, points.size());

  // Sum columns and divide
  return mat.rowwise().mean();
}

}  // namespace o3d_slam
