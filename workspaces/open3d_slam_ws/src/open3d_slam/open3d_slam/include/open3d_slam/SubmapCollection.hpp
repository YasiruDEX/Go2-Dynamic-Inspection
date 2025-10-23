/*
 * SubmapCollection.hpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>
#include <mutex>
#include <unordered_map>
#include "open3d_slam/AdjacencyMatrix.hpp"
#include "open3d_slam/CircularBuffer.hpp"
#include "open3d_slam/Constraint.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/PlaceRecognition.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/ThreadSafeBuffer.hpp"
#include "open3d_slam/croppers.hpp"

namespace o3d_slam {

class SubmapCollection {
  struct ScanTimeTransform {
    PointCloud cloud_;
    Time timestamp_;
    Transform mapToRangeSensor_;
  };

  struct CellKey {
    int x, y, z;
    bool operator==(const CellKey& other) const { return x == other.x && y == other.y && z == other.z; }
  };

  struct CellKeyHash {
    size_t operator()(const CellKey& k) const noexcept {
      size_t h = std::hash<int>()(k.x);
      h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };
  inline CellKey keyFromPoint(const Eigen::Vector3d& p, double inv_cell) {
    return {static_cast<int>(std::floor(p.x() * inv_cell)), static_cast<int>(std::floor(p.y() * inv_cell)),
            static_cast<int>(std::floor(p.z() * inv_cell))};
  }

  struct Pair {
    double d;
    size_t i;
  };

  struct SubmapOverlap {
    size_t submap_idx;
    int overlap_count;
  };

  struct ConsistencyCheckCache {
    Eigen::Vector3d last_position;
    double last_fitness;
    bool valid = false;
  };

 public:
  using SubmapId = Submap::SubmapId;
  using TimestampedSubmapIds = std::vector<TimestampedSubmapId>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SubmapCollection();
  ~SubmapCollection() = default;

  void setMapToRangeSensor(const Transform& T);
  const Submap& getActiveSubmap() const;
  bool insertScan(const PointCloud& rawScan, const PointCloud& preProcessedScan, const Transform& mapToRangeSensor, const Time& timestamp);
  void setParameters(const MapperParameters& p);
  bool isEmpty() const;
  Submap* getSubmapPtr(SubmapId idx);
  const Submap& getSubmap(SubmapId idx) const;
  size_t getNumSubmaps() const;
  size_t getTotalNumPoints() const;

  bool setInitialMap(const PointCloud& initialMap);
  bool chunkTheInitialMapIntoSubmaps(const PointCloud& initialMap);
  bool setInitialMapIntoSingleSubmap(const PointCloud& initialMap);

  void computeFeatures(const TimestampedSubmapIds& ids);
  bool isComputingFeatures() const;
  TimestampedSubmapIds popFinishedSubmapIds();
  size_t numFinishedSubmaps() const;

  Constraints buildLoopClosureConstraints(const TimestampedSubmapIds& ids);
  size_t numLoopClosureCandidates() const;
  TimestampedSubmapIds popLoopClosureCandidates();

  std::vector<size_t> selectActiveMostOverlapAndKClosest(const PointCloud& rawScan, const Transform& T, size_t k,
                                                         size_t active_submap_idx) const;
  std::vector<size_t> findKMostOverlappingSubmaps(const PointCloud& scan, const Transform& T, size_t k, size_t exclude) const;

  bool dumpToFile(const std::string& folderPath, const std::string& filename, const bool& isDenseMap) const;
  void transform(const OptimizedTransforms& transformIncrements);
  void updateAdjacencyMatrix(const Constraints& loopClosureConstraints);
  const Constraints& getOdometryConstraints() const;

  const MapperParameters& getParameters() const;
  void setFolderPath(const std::string& folderPath);

  void forceNewSubmapCreation();
  void createNewSubmap(const Transform& mapToSubmap);
  PointCloudPtr getCachedCompositeSubmapFromMulti(const std::vector<size_t>& neighbor_idxs) const;
  PointCloudPtr getCachedCompositeSubmapFromVoxel(const std::vector<size_t>& neighbor_idxs) const;

  std::vector<size_t> findKClosestSubmaps(const Transform& mapToRS, size_t k, size_t exclude_idx = SIZE_MAX) const;

  PointCloud buildCompositeSubmapSingle(const std::vector<size_t>& idxs, double voxel_size_downsample = 0.0) const;
  PointCloud buildCompositeSubmapFromMulti(const std::vector<size_t>& idxs) const;
  PointCloud buildCompositeSubmapFromVoxel(const std::vector<size_t>& idxs) const;
  size_t activeSubmapIdx_ = 0;

  mutable std::vector<size_t> cached_neighbor_idxs_;
  mutable PointCloudPtr cached_neighbor_cloud_;
  mutable size_t cached_active_submap_idx_ = SIZE_MAX;
  mutable std::mutex composite_cache_mutex_;
  mutable PointCloudPtr cached_static_neighbors_cloud_;      // neighbours - active
  mutable std::vector<size_t> cached_static_neighbor_idxs_;  // neighbours - active

  mutable PointCloudPtr cached_static_cloud_;       // neighbours only
  mutable std::vector<size_t> cached_static_idxs_;  // neighbour ids
  mutable size_t cached_static_point_count_ = 0;    // for resize checks

  mutable PointCloudPtr cached_combined_cloud_;   // neighbours + active
  mutable size_t cached_active_point_count_ = 0;  // active size last time

 private:
  bool isSwitchingSubmapsConsistant(const PointCloud& scan, size_t newActiveSubmapCandidate, const Transform& mapToRangeSensor) const;
  void insertBufferedScans(Submap* submap);
  void addScanToBuffer(const PointCloud& scan, const Transform& mapToRangeSensor, const Time& timestamp);
  void updateActiveSubmap(const Transform& mapToRangeSensor, const PointCloud& scan);

  // size_t findClosestSubmap(const Transform& mapToRangesensor) const;
  size_t findClosestSubmap(const Transform& mapToRangeSensor, size_t exclude_idx) const;
  std::vector<size_t> getAllSubmapIdxs() const;

  Transform mapToRangeSensor_ = Transform::Identity();
  Time timestamp_;
  std::vector<Submap> submaps_;

  MapperParameters params_;
  size_t numScansMergedInActiveSubmap_ = 0;
  size_t lastFinishedSubmapIdx_ = 0;
  std::mutex featureComputationMutex_;
  bool isComputingFeatures_ = false;
  std::mutex constraintBuildMutex_;
  AdjacencyMatrix adjacencyMatrix_;
  size_t submapId_ = 0;
  PlaceRecognition placeRecognition_;
  ThreadSafeBuffer<TimestampedSubmapId> loopClosureCandidatesIdxs_, finishedSubmapsIdxs_;
  Constraints odometryConstraints_;
  CircularBuffer<ScanTimeTransform> overlapScansBuffer_;
  std::string savingDataFolderPath_;
  bool isForceNewSubmapCreation_ = false;
  std::vector<Eigen::Vector3d> lastVisitPosition_;
  mutable std::vector<ConsistencyCheckCache> consistency_cache_;
  mutable std::mutex consistency_cache_mtx_;
  mutable VoxelMap consistencyCheckNeighbourVoxCopy_;
  mutable size_t lastConsistencyCandidateSubmapIdx_ = std::numeric_limits<size_t>::max();

  // TODO MAGIC
  // const double minDistanceToReturnToRecentSubmap_ = 30.0;  // in meters
  const double kConsistencyCheckSpatialThresh = 5.0;  // meters
};

}  // namespace o3d_slam
