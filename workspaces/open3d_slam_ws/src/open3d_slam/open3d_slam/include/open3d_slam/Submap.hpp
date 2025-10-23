/*
 * Submap.hpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Feature.h>
#include <Eigen/Dense>
#include <atomic>
#include <future>
#include <mutex>
#include <thread>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Transform.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/time.hpp"

namespace o3d_slam {

struct TimestampedSubmapId {
  size_t submapId_;
  Time time_;
};

class Submap {
 public:
  using PointCloud = open3d::geometry::PointCloud;
  using Feature = open3d::pipelines::registration::Feature;
  using SubmapId = size_t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Submap(size_t id, size_t parentId);
  ~Submap();

  void setParameters(const MapperParameters& mapperParams);
  bool insertScan(const PointCloud& rawScan, const PointCloud& preProcessedScan, const Transform& transform, const Time& time,
                  bool isPerformCarving);
  bool insertScanDenseMap(const PointCloud& rawScan, const Transform& transform, const Time& time, bool isPerformCarving);

  const Transform& getMapToSubmapOrigin() const;
  Eigen::Vector3d getMapToSubmapCenter() const;
  inline void setSubmapCenter(const Eigen::Vector3d& c) { submapCenter_ = c; }
  void setMapToSubmapOrigin(const Transform& T);
  const PointCloud& getMapPointCloud() const;
  PointCloud getMapPointCloudCopy() const;
  const VoxelizedPointCloud& getDenseMap() const;
  VoxelizedPointCloud getDenseMapCopy() const;
  bool isEmpty() const;
  std::size_t getNbPoints() const;
  const Feature& getFeatures() const;
  const PointCloud& getSparseMapPointCloud() const;
  void computeSubmapCenter();
  Eigen::Vector3d ComputeCenterCustom(const std::vector<Eigen::Vector3d>& points);
  Eigen::Vector3d ComputeCenterSIMD(const std::vector<Eigen::Vector3d>& points);
  void computeFeatures();
  size_t getId() const;
  size_t getParentId() const;
  void transform(const Transform& T);
  const VoxelMap& getVoxelMap() const;
  mutable PointCloud toRemove_;
  mutable PointCloud scanRef_;

  Submap(const Submap& other);

 private:
  void carve(const PointCloud& scan, const Eigen::Vector3d& sensorPosition, const SpaceCarvingParameters& param,
             VoxelizedPointCloud* cloud);
  void update(const MapperParameters& mapperParams);
  void carve(const PointCloud& rawScan, const Transform& mapToRangeSensor, const CroppingVolume& cropper,
             const SpaceCarvingParameters& params, PointCloud* map);
  void voxelizeInsideCroppingVolume(const CroppingVolume& cropper, const MapBuilderParameters& param, PointCloud* map) const;

  PointCloud sparseMapCloud_, mapCloud_;
  Transform mapToSubmap_ = Transform::Identity();
  Transform mapToRangeSensor_ = Transform::Identity();
  Eigen::Vector3d submapCenter_ = Eigen::Vector3d::Zero();
  std::shared_ptr<CroppingVolume> denseMapCropper_, mapBuilderCropper_;
  MapperParameters params_;
  Timer featureTimer_;
  size_t nScansInsertedMap_ = 0;
  size_t nScansInsertedDenseMap_ = 0;
  std::shared_ptr<Feature> feature_;
  size_t id_ = 0;
  bool isCenterComputed_ = false;
  size_t parentId_ = 0;
  Timer carvingStatisticsTimer_;
  Timer voxelizeAndCropTimer;
  int scanCounter_ = 0;
  VoxelMap voxelMap_;
  VoxelizedPointCloud denseMap_;
  ColorRangeCropper colorCropper_;
  mutable std::mutex denseMapMutex_;
  mutable std::mutex mapPointCloudMutex_;
  std::future<void> voxelizationFuture_;
  std::atomic<bool> voxelizationRunning_ = false;
  int voxelizeEveryNscans_ = 10;
};

}  // namespace o3d_slam
