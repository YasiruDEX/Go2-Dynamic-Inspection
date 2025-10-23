/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include "open3d_conversions/open3d_conversions.hpp"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
#include <open3d/utility/Helper.h>
#include "open3d_slam/ProfilerScopeGuard.hpp"

#pragma omp declare reduction(vec3d_plus : Eigen::Vector3d : omp_out += omp_in) initializer(omp_priv = Eigen::Vector3d::Zero())

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const bool isCheckTransformChainingAndPrintResult = false;
}  // namespace

Mapper::Mapper(const TransformInterpolationBuffer& odomToRangeSensorBuffer, std::shared_ptr<SubmapCollection> submaps)
    : odomToRangeSensorBuffer_(odomToRangeSensorBuffer), submaps_(submaps) {
  // `updates` with default parameters
  update(params_);
  double max_correspondence_distance = 2.0;
  small_registration_.reduction.num_threads = 8;
  small_registration_.rejector.max_dist_sq = max_correspondence_distance * max_correspondence_distance;
  small_registration_.criteria.rotation_eps = 0.005 * M_PI / 180.0;  // 0.001;
  small_registration_.criteria.translation_eps = 1e-3;               // 0.0008;
  small_registration_.optimizer.max_iterations = 30;
  small_registration_.optimizer.verbose = false;

  base_max_corr_dist_ = static_cast<float>(max_correspondence_distance);
}

void Mapper::computeSpaciousness(const PointCloud& scan) {
  if (scan.IsEmpty()) return;

  std::vector<float> r;
  r.reserve(scan.points_.size());
  for (const auto& p : scan.points_) r.emplace_back(std::hypot(p(0), p(1)));

  const size_t mid = r.size() / 2;
  std::nth_element(r.begin(), r.begin() + mid, r.end());
  const float median = r[mid];

  constexpr float alpha = 0.95f;  // LPF coefficient
  metrics_.spaciousness = (metrics_.spaciousness == 0.0f) ? median : alpha * metrics_.spaciousness + (1.0f - alpha) * median;
}

float Mapper::computeDensity(const PointCloud& scan, float v = 0.10f) {
  if (scan.IsEmpty()) {
    std::cout << "[Mapper] Input scan is empty. Returning density 0.0." << std::endl;
    return 0.0f;
  }

  struct Key {
    int x, y, z;
    bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct Hash {
    std::size_t operator()(const Key& k) const {
      return (std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) ^ (std::hash<int>()(k.z) << 2);
    }
  };
  std::unordered_set<Key, Hash> vox;
  vox.reserve(scan.points_.size());

  for (const auto& p : scan.points_) {
    Key k{int(std::floor(p(0) / v)), int(std::floor(p(1) / v)), int(std::floor(p(2) / v))};
    vox.insert(k);
  }
  double voxels = static_cast<double>(vox.size());
  double volume = voxels * v * v * v;  // m³
  double points = static_cast<double>(scan.points_.size());
  return static_cast<float>(points / volume);  // points per m³
}

void Mapper::setAdaptiveParams() {
  if (metrics_.spaciousness == 0.0f || metrics_.density == 0.0f) return;

  /* --- map metrics → scale ∈ [0.25 … 2.0] -------------------------- */
  const float sp_low = 2.0f;   // m  (tight corridor)
  const float sp_high = 8.0f;  // m  (open space)
  float sp_scale = std::clamp((metrics_.spaciousness - sp_low) / (sp_high - sp_low), 0.0f, 1.0f);
  sp_scale = 0.25f + sp_scale * (2.0f - 0.25f);

  const float den_low = 100.0f;   // pts / m³  (sparse)
  const float den_high = 800.0f;  // pts / m³  (very dense)
  float den_scale = std::clamp((den_high - metrics_.density) / (den_high - den_low), 0.0f, 1.0f);
  den_scale = 0.25f + den_scale * (2.0f - 0.25f);

  /* choose the larger (safer) of the two scales                    */
  const float scale = std::clamp(std::max(sp_scale, den_scale), 0.25f, 5.0f);

  /* desired correspondence distance in metres                      */
  const float target_dist = base_max_corr_dist_ * scale;  // 0.1 … 2.0
  const float target_dist_sq = target_dist * target_dist;

  /* smooth the output                                              */
  constexpr float beta = 0.80f;  // 0 → instant, 1 → no change
  small_registration_.rejector.max_dist_sq = beta * small_registration_.rejector.max_dist_sq + (1.0f - beta) * target_dist_sq;

  std::cout << "\033[38;5;208m[Mapper] max_dist_sq = " << small_registration_.rejector.max_dist_sq << "\033[0m\n";
}

void Mapper::updateRejectorFromOdometryMotion(const Transform& odometryMotion) {
  /* ------------------------------------------------------------------
   * 1.  Compute a scalar motion metric for this frame pair
   *     ‖Δt‖  +  w·Δθ      (metres + radians·w)
   * ------------------------------------------------------------------*/
  const double trans = odometryMotion.translation().norm();               // m
  const double rot = Eigen::AngleAxisd(odometryMotion.linear()).angle();  // rad
  const double metric = trans + 0.5 * rot;                                // w = 0.5

  /* ------------------------------------------------------------------
   * 2.  Maintain a sliding window of recent metrics
   * ------------------------------------------------------------------*/
  pushOdometryMetric(metric);
  if (odometryMotionHistory_.size() < 5)  // wait until we have ≥5 samples
    return;

  /* ------------------------------------------------------------------
   * 3.  Standard deviation of the window  →  jitter proxy
   * ------------------------------------------------------------------*/
  const double mean = std::accumulate(odometryMotionHistory_.begin(), odometryMotionHistory_.end(), 0.0) / odometryMotionHistory_.size();

  double var = 0.0;
  for (double m : odometryMotionHistory_) var += (m - mean) * (m - mean);
  const double stddev = std::sqrt(var / odometryMotionHistory_.size());

  /* ------------------------------------------------------------------
   * 4.  Map  stddev  ∈ [low … high]  →  s ∈ [0 … 1]
   *     low  = “calm”;  high = “very jumpy”
   * ------------------------------------------------------------------*/
  constexpr double low = 0.02;   // tolerant region (m-equivalent)
  constexpr double high = 0.25;  // extreme jitter
  const double s = std::clamp((stddev - low) / (high - low), 0.0, 1.0);

  /* ------------------------------------------------------------------
   * 5.  Interpolate max_dist_sq in   [0.16 … 1.5]   using s
   *     (These are squared distances: 0.16 = 0.40², 1.5 ≈ 1.225²)
   * ------------------------------------------------------------------*/
  constexpr double dist_min_sq = 0.16;  // lower bound  (0.40 m)
  constexpr double dist_max_sq = 1.50;  // upper bound ~(1.22 m)
  const double target_sq = dist_min_sq + s * (dist_max_sq - dist_min_sq);

  /* ------------------------------------------------------------------
   * 6.  Low-pass filter the update so it changes smoothly
   * ------------------------------------------------------------------*/
  constexpr double beta = 0.80;  // 0 → instant, 1 → frozen
  auto& current = small_registration_.rejector.max_dist_sq;
  current = beta * current + (1.0 - beta) * target_sq;
}

void Mapper::setParameters(const MapperParameters& p) {
  params_ = p;
  update(p);
}

MapperParameters* Mapper::getParametersPtr() {
  return &params_;
}

void Mapper::setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform) {
  // Not thread safe?
  calibration_ = transform.matrix();
  isCalibrationSet_ = true;
  return;
}

bool Mapper::isExternalOdometryFrameToCloudFrameCalibrationSet() {
  return isCalibrationSet_;
}

void Mapper::loopClosureUpdate(const Transform& loopClosureCorrection) {
  mapToRangeSensor_ = loopClosureCorrection * mapToRangeSensor_;
  mapToRangeSensorPrev_ = loopClosureCorrection * mapToRangeSensorPrev_;
}

bool Mapper::hasProcessedMeasurements() const {
  return !mapToRangeSensorBuffer_.empty();
}

void Mapper::update(const MapperParameters& p) {
  scan2MapReg_ = scanToMapRegistrationFactory(p);
  submaps_->setParameters(p);
}

Transform Mapper::getMapToOdom(const Time& timestamp) const {
  const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
  const Transform mapToRangeSensor = getTransform(timestamp, mapToRangeSensorBuffer_);
  return mapToRangeSensor * odomToRangeSensor.inverse();
}

Transform Mapper::getMapToRangeSensor(const Time& timestamp) const {
  return getTransform(timestamp, mapToRangeSensorBuffer_);
}

bool Mapper::isRegistrationBestGuessBufferEmpty() const {
  return bestGuessBuffer_.empty();
}

Transform Mapper::getRegistrationBestGuess(const Time& timestamp) const {
  return getTransform(timestamp, bestGuessBuffer_);
}

const SubmapCollection& Mapper::getSubmaps() const {
  return *submaps_;
}

const Submap& Mapper::getActiveSubmap() const {
  return submaps_->getActiveSubmap();
}

const TransformInterpolationBuffer& Mapper::getMapToRangeSensorBuffer() const {
  return mapToRangeSensorBuffer_;
}

SubmapCollection* Mapper::getSubmapsPtr() {
  return submaps_.get();
}

void Mapper::setMapToRangeSensor(const Transform& t) {
  mapToRangeSensor_ = t;
}
void Mapper::setMapToRangeSensorInitial(const Transform& t) {
  if (isNewValueSetMapper_) {
    return;
  }

  std::cout << " Setting initial value transform at mapper: "
            << "\033[92m" << asString(t) << " \n"
            << "\033[0m";

  mapToRangeSensorPrev_ = t;
  mapToRangeSensor_ = t;
  isNewValueSetMapper_ = true;
  isInitialTransformSet_ = true;
}

const PointCloud& Mapper::getPreprocessedScan() const {
  return preProcessedScan_;
}

const ScanToMapRegistration& Mapper::getScanToMapRegistration() const {
  return *scan2MapReg_;
}

bool Mapper::setInitialMap(const PointCloud& initialMap) {
  ProfilerScopeGuard scope("Mapper::setInitialMap", "/tmp/slam_profile.csv");

  if (!params_.isUseInitialMap_) {
    std::cerr << "Mapper is not configured to use an initial map." << std::endl;
    return false;
  }
  std::cout << "Setting initial map in Mapper." << std::endl;
  const bool ok = submaps_->setInitialMap(initialMap);

  return ok;
}

bool Mapper::addRangeMeasurement(const Mapper::PointCloud& rawScan, const Time& timestamp) {
  ProfilerScopeGuard main_scope("Mapper::addRangeMeasurement", "/tmp/slam_profile.csv",
                                "timestamp=" + std::to_string(toUniversal(timestamp)));

  if (!params_.isUseInitialMap_) {
    ProfilerScopeGuard scope("checkCalibration", "/tmp/slam_profile.csv");
    if (!isCalibrationSet_) {
      std::cerr << "Calibration is not set. Returning from mapping." << std::endl;
      return false;
    }
  }

  {
    ProfilerScopeGuard scope("setMapToRangeSensor", "/tmp/slam_profile.csv");
    submaps_->setMapToRangeSensor(mapToRangeSensor_);
  }

  if (submaps_->getNumSubmaps() == 0) {
    ProfilerScopeGuard scope("insertInitialScan", "/tmp/slam_profile.csv");

    if (params_.isUseInitialMap_) {
      // assert_true(scan2MapReg_->isMergeScanValid(rawScan), "Init map invalid!!!!");
      // submaps_->insertScan(rawScan, rawScan, mapToRangeSensor_, timestamp);
    } else {
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_, true);

      {
        ProfilerScopeGuard scope_source_init("sourceInit", "/tmp/slam_profile.csv",
                                             "size=" + std::to_string(processed.merge_->points_.size()));

        source_ = std::make_shared<small_gicp::PointCloud>(processed.merge_->points_);
        // c_s = computeCentroid(*source_);
        // translatePointCloud(*source_, -c_s);
      }

      {
        ProfilerScopeGuard scope_source_kdtree("sourceKdTree", "/tmp/slam_profile.csv");
        source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_, small_gicp::KdTreeBuilderOMP(4));
      }

      {
        ProfilerScopeGuard scope_source_normals("sourceNormals", "/tmp/slam_profile.csv");

        small_gicp::PointCloud& dst = *source_;  // always small‑gicp
        const auto& src_ptr = processed.merge_;  // OPEN3D ptr
        // const std::size_t N = dst.size();

        const int num_threads = (small_registration_.reduction.num_threads > 0) ? small_registration_.reduction.num_threads
                                                                                : static_cast<int>(std::thread::hardware_concurrency());

        estimate_normals_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);
        // estimate_covariances_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);

        const size_t N = dst.normals.size();
        if (processed.merge_->normals_.size() != N) processed.merge_->normals_.resize(N);

        constexpr size_t OMP_THRESHOLD = 8192;
        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            // Efficient: Copy first 3 doubles from dst.normals[i] to processed.match_->normals_[i]
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        } else {
          for (size_t i = 0; i < N; ++i) {
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        }
      }

      submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
      mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
      bestGuessBuffer_.push(timestamp, mapToRangeSensor_);
    }
    return true;
  }

  if (timestamp <= lastMeasurementTimestamp_) {
    ProfilerScopeGuard scope("handleOutOfOrder", "/tmp/slam_profile.csv");
    std::cerr << "\n\n !!!!! MAPPER WARNING: Measurements came out of order!!!! \n\n";

    int64_t uts_timestamp = toUniversal(timestamp);
    int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " Current Timestamp: \033[92m" << ns_since_unix_epoch << " \n\033[0m";

    int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
    int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " Last Measurement Timestamp: \033[92m" << ns_since_unix_epoch_prev << " \n\033[0m";

    int64_t diff_millisecond_since_unix_epoch = (ns_since_unix_epoch - ns_since_unix_epoch_prev) / 1000000ll;
    std::cout << " Difference in Millisecond: \033[92m" << diff_millisecond_since_unix_epoch << " \n\033[0m";

    Time arbitraryLatestTime = odomToRangeSensorBuffer_.latest_time();
    Transform odomToRangeSensor = getTransform(arbitraryLatestTime, odomToRangeSensorBuffer_) * calibration_.inverse();
    Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
    Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
    Transform backupTransform = mapToRangeSensorPrev_ * odometryMotion;

    mapToRangeSensor_.matrix() = backupTransform.matrix();
    mapToRangeSensorBuffer_.push(arbitraryLatestTime, mapToRangeSensor_);
    bestGuessBuffer_.push(arbitraryLatestTime, mapToRangeSensorPrev_);
    submaps_->setMapToRangeSensor(mapToRangeSensor_);
    mapToRangeSensorPrev_ = mapToRangeSensor_;
    return true;
  }

  {
    ProfilerScopeGuard scope("checkTransformBuffer", "/tmp/slam_profile.csv");
    bool isOdomOkay = odomToRangeSensorBuffer_.has_query(timestamp);
    if (!isOdomOkay) {
      std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
      std::cerr << "Going to attempt the scan to map refinement anyway. \n";
    }
    checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);
  }

  Transform mapToRangeSensorEstimate = mapToRangeSensorPrev_;

  {
    ProfilerScopeGuard scope("odometryEstimation", "/tmp/slam_profile.csv");
    if (!isNewValueSetMapper_ && !isIgnoreOdometryPrediction_ && odomToRangeSensorBuffer_.has(timestamp)) {
      Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_) * calibration_.inverse();
      // Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();

      Transform odomToRangeSensorPrev = odomToRangeSensor;
      if (lastMeasurementTimestamp_ >= odomToRangeSensorBuffer_.earliest_time()) {
        odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
      } else {
        std::cerr << "Set odomToRangeSensorPrev = odomToRangeSensor so odometryMotion = I because "
            "lastMeasurementTimestamp_ >= odomToRangeSensorBuffer_.earliest_time(). May happen on first iteration.\n";
      
        std::cout << "[Mapper][DEBUG] mapToRangeSensorEstimate:\n" << mapToRangeSensorEstimate.matrix() << std::endl;
        std::cout << "[Mapper][DEBUG] odomToRangeSensor:\n" << odomToRangeSensor.matrix() << std::endl;
        std::cout << "[Mapper][DEBUG] odomToRangeSensorPrev:\n" << odomToRangeSensorPrev.matrix() << std::endl;
        std::cout << "[Mapper][DEBUG] odometryMotionMemory_:\n" << odometryMotionMemory_.matrix() << std::endl;
        
          }

      Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;

      // // Check odometry motion for large jumps
      // double odom_translation = odometryMotion.translation().norm();
      // double odom_rotation = Eigen::AngleAxisd(odometryMotion.linear()).angle();

      // const double max_odom_translation = 0.15;              // 20 cm
      // const double max_odom_rotation = 15.0 * M_PI / 180.0;  // 15 degrees in radians

      // if (odom_translation > max_odom_translation || odom_rotation > max_odom_rotation) {
      //   std::cerr << "\n\033[1;35;105m"
      //             << "[Mapper] Odometry motion too large (" << odom_translation << " m, " << (odom_rotation * 180.0 / M_PI) << " deg). "
      //             << "Using last valid odometry motion instead."
      //             << "\033[0m\n";
      //   odometryMotion = odometryMotionMemory_;
      // }

      odometryMotionMemory_ = odometryMotion;
      // updateRejectorFromOdometryMotion(odometryMotion);
      mapToRangeSensorEstimate = mapToRangeSensorPrev_ * odometryMotion;
    }
  }

  isIgnoreOdometryPrediction_ = false;
  ProcessedScans processed;
  {
    ProfilerScopeGuard scope("scanPreprocessing", "/tmp/slam_profile.csv");
    processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_, false);
  }

  Transform correctedTransform_o3d;
  {
    // Compute time since last reference update
    double passedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;

    // Compute relative motion since last reference update
    Transform relative = lastReferenceInitializationPose_.inverse() * mapToRangeSensor_;
    double translation = relative.translation().norm();
    double rotation = Eigen::AngleAxisd(relative.linear()).angle();

    // Configurable thresholds
    const double minTrans = 0.5;  // meters
    // Convert degrees to radians for minRot parameter
    const double minRot = 5.0 * M_PI / 180.0;                                           // degrees to radians
    const double minInterval = params_.scanMatcher_.icp_.referenceCloudSettingPeriod_;  // seconds

    bool enoughMotion = (translation >= minTrans) || (rotation >= minRot);
    bool enoughTime = (passedTime >= minInterval);
    bool updateReference = isNewValueSetMapper_ || (enoughMotion && enoughTime) || (firstCall_);

    if (updateReference) {
      firstCall_ = false;
      PointCloudPtr mapPatch;
      {
        ProfilerScopeGuard scope_crop("cropSubmap", "/tmp/slam_profile.csv");

        if (submaps_->getNumSubmaps() == 1) {
          // Only one submap, crop from the active submap
          mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_, false);
        } else {
          size_t active = submaps_->activeSubmapIdx_;
          std::vector<size_t> nbrs = submaps_->findKClosestSubmaps(mapToRangeSensor_,
                                                                   /*k=*/0,  // number of neighbours you want
                                                                   active);  // <-- exclude only for search

          nbrs.push_back(active);
          mapPatch = submaps_->getCachedCompositeSubmapFromMulti(nbrs);

          // std::vector<size_t> nbrs = submaps_->findKMostOverlappingSubmaps(rawScan, mapToRangeSensor_,
          //                                                                  /*k=*/2,  // number of neighbours you want
          //                                                                  active);  // <-- exclude only for search

          // std::vector<size_t> nbrs = submaps_->selectActiveMostOverlapAndKClosest(rawScan, mapToRangeSensor_, 2, active);
          // mapPatch = submaps_->getCachedCompositeSubmapFromMulti(nbrs);

          // TODO needs work.
          // mapPatch = submaps_->getCachedCompositeSubmapFromVoxel(nbrs);
        }
      }

      if (mapPatch->IsEmpty()) {
        std::cerr << "\033[1;31m[Mapper] mapPatch is empty! Skipping scan matching.\033[0m" << std::endl;
        return false;
      }

      std::lock_guard<std::mutex> lck(mapManipulationMutex_);
      lastReferenceInitializationTimestamp_ = timestamp;
      lastReferenceInitializationPose_ = mapToRangeSensor_;  // save pose for next time

      // Experimental.
      // {
      //   ProfilerScopeGuard scope("targetPipeline", "/tmp/slam_profile.csv", "size=" + std::to_string(mapPatch->points_.size()));

      //   target_ = cropPreparePointCloud(*mapPatch,
      //                                   mapToRangeSensorEstimate,  // centre pose
      //                                   /*radius     =*/60.0,
      //                                   /*normal_knn =*/10,
      //                                   /*num_threads=*/8);
      // }

      {
        ProfilerScopeGuard scope_target_init("targetInit", "/tmp/slam_profile.csv", "size=" + std::to_string(mapPatch->points_.size()));
        target_ = std::make_shared<small_gicp::PointCloud>(mapPatch->points_);
      }

      {
        ProfilerScopeGuard scope_target_kdtree("targetKdTree", "/tmp/slam_profile.csv");
        target_tree_ =
            std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target_, small_gicp::KdTreeBuilderOMP(8));
      }

      {
        ProfilerScopeGuard scope_target_cov("targetNormals", "/tmp/slam_profile.csv");
        copyOrEstimateNormals(*mapPatch, *target_, *target_tree_, /*knn=*/10, /*num_threads=*/8);
      }
    }

    {
      ProfilerScopeGuard scope_source_init("sourceInit", "/tmp/slam_profile.csv",
                                           "size=" + std::to_string(processed.merge_->points_.size()));

      source_ = std::make_shared<small_gicp::PointCloud>(processed.merge_->points_);
      // c_s = computeCentroid(*source_);
      // translatePointCloud(*source_, -c_s);
    }

    {
      ProfilerScopeGuard scope_source_normals("sourceNormals", "/tmp/slam_profile.csv");

      small_gicp::PointCloud& dst = *source_;
      const auto& src_ptr = processed.merge_;
      const std::size_t N = dst.size();

      if (src_ptr->HasNormals() && src_ptr->normals_.size() == N) {
        if (dst.normals.size() != N) dst.normals.resize(N);

        assert((reinterpret_cast<uintptr_t>(dst.normals.data()) & 0xF) == 0 && "dst.normals is not 16‑byte aligned");

        constexpr std::size_t OMP_THRESHOLD = 8192;

        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            std::memcpy(dst.normals[i].data(), src_ptr->normals_[i].data(), 3 * sizeof(double));
            dst.normals[i][3] = 0.0;
          }
        } else {
          for (std::size_t i = 0; i < N; ++i) {
            std::memcpy(dst.normals[i].data(), src_ptr->normals_[i].data(), 3 * sizeof(double));
            dst.normals[i][3] = 0.0;
          }
        }
      }
      /* --------------------------------------------------------------- */
      else {
        ProfilerScopeGuard scope_source_normals("sourceNormalsCalculatedAndCopied", "/tmp/slam_profile.csv");
        const int num_threads = (small_registration_.reduction.num_threads > 0) ? small_registration_.reduction.num_threads
                                                                                : static_cast<int>(std::thread::hardware_concurrency());

        {
          // Source tree is only needed for the normals calculation
          ProfilerScopeGuard scope_source_kdtree("sourceKdTree", "/tmp/slam_profile.csv");
          source_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_, small_gicp::KdTreeBuilderOMP(4));
        }

        estimate_normals_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);
        // estimate_covariances_omp(dst, *source_tree_, /*knn=*/10, /*threads=*/num_threads);

        const size_t N = dst.normals.size();
        if (processed.merge_->normals_.size() != N) processed.merge_->normals_.resize(N);

        constexpr size_t OMP_THRESHOLD = 8192;
        if (N >= OMP_THRESHOLD) {
#pragma omp parallel for schedule(static)
          for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
            // Efficient: Copy first 3 doubles from dst.normals[i] to processed.match_->normals_[i]
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        } else {
          for (size_t i = 0; i < N; ++i) {
            std::memcpy(processed.merge_->normals_[i].data(), dst.normals[i].data(), 3 * sizeof(double));
          }
        }
      }
    }

    {
      ProfilerScopeGuard scope_icp_align("icpAlign", "/tmp/slam_profile.csv");
      std::lock_guard<std::mutex> lck(mapManipulationMutex_);

      // Initial attemps to make parameters adaptive
      // computeSpaciousness(rawScan);
      // const float dens_now = computeDensity(*processed.merge_);
      // metrics_.density = (metrics_.density == 0.0f) ? dens_now : 0.90f * metrics_.density + 0.10f * dens_now;
      // setAdaptiveParams();

      // Eigen::Isometry3d init_T_target_source =
      //     Eigen::Translation3d(-c_t) * Eigen::Isometry3d(mapToRangeSensorEstimate.matrix()) * Eigen::Translation3d(c_s);

      Eigen::Isometry3d init_T_target_source(mapToRangeSensorEstimate.matrix());
      // Print the 4x4 initial transformation matrix
      // std::cout << "[Mapper] Initial T_target_source:\n" << init_T_target_source.matrix() << std::endl;

      // // Print sizes of target and source clouds
      // std::cout << "[Mapper] Target cloud size: " << target_->size() << std::endl;
      // std::cout << "[Mapper] Source cloud size: " << source_->size() << std::endl;

      // // Print first 10 points from target cloud
      // std::cout << "[Mapper] First 10 points from target cloud:" << std::endl;
      // for (size_t i = 0; i < std::min<size_t>(10, target_->size()); ++i) {
      //   const auto& pt = target_->point(i);
      //   std::cout << "  [" << i << "]: " << pt.transpose() << std::endl;
      // }

      // // Print first 10 points from source cloud
      // std::cout << "[Mapper] First 10 points from source cloud:" << std::endl;
      // for (size_t i = 0; i < std::min<size_t>(10, source_->size()); ++i) {
      //   const auto& pt = source_->point(i);
      //   std::cout << "  [" << i << "]: " << pt.transpose() << std::endl;
      // }
      auto result = small_registration_.align(*target_, *source_, *target_tree_, init_T_target_source);
      // result.T_target_source = Eigen::Translation3d(c_t) * result.T_target_source * Eigen::Translation3d(-c_s);
      correctedTransform_o3d.matrix() = result.T_target_source.matrix();

      // // Check if the result transform differs too much from the initial transform
      // double translation_diff = (result.T_target_source.translation() - init_T_target_source.translation()).norm();
      // double rotation_diff = Eigen::AngleAxisd(result.T_target_source.linear().transpose() * init_T_target_source.linear()).angle();

      // const double max_translation_diff = 2.0;               // meters
      // const double max_rotation_diff = 50.0 * M_PI / 180.0;  // 30 degrees in radians

      // if (translation_diff > max_translation_diff || rotation_diff > max_rotation_diff) {
      //   std::cerr << "\n\n\033[1;41;97m"
      //             << "!!!!!!! ICP result transform differs too much from initial guess: "
      //             << "translation diff = " << translation_diff << ", rotation diff (deg) = " << (rotation_diff * 180.0 / M_PI)
      //             << ". Using initial guess instead! "
      //             << "\033[0m\n\n";
      //   correctedTransform_o3d.matrix() = init_T_target_source.matrix();
      // }
    }
  }

  if (isNewValueSetMapper_) {
    if (isTimeValid(timestamp)) initTime_ = timestamp;

    mapToRangeSensorPrev_ = mapToRangeSensor_;
    mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
    bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
    isNewValueSetMapper_ = false;
    isIgnoreOdometryPrediction_ = true;
    return true;
  }

  preProcessedScan_ = *processed.merge_;
  // preProcessedScan_ = *processed.match_;
  mapToRangeSensor_.matrix() = correctedTransform_o3d.matrix();
  mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
  bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
  submaps_->setMapToRangeSensor(mapToRangeSensor_);

  {
    ProfilerScopeGuard scope("scanInsertionCheck", "/tmp/slam_profile.csv");
    double timeSinceInit = toSecondsSinceFirstMeasurement(timestamp) - toSecondsSinceFirstMeasurement(initTime_);
    if ((params_.isUseInitialMap_ && !params_.isMergeScansIntoMap_) ||
        (timeSinceInit < params_.mapMergeDelayInSeconds_ && params_.isUseInitialMap_ && params_.isMergeScansIntoMap_)) {
      lastMeasurementTimestamp_ = timestamp;
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      return true;
    }
  }

  {
    ProfilerScopeGuard scope("scanInsertion(mapper)", "/tmp/slam_profile.csv");
    // scanInsertionTimer_.startStopwatch();

    Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
    if (sensorMotion.translation().norm() >= params_.minMovementBetweenMappingSteps_) {
      submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
      mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
    }

    lastMeasurementTimestamp_ = timestamp;
    mapToRangeSensorPrev_ = mapToRangeSensor_;

    // double insertiontimeElapsed = scanInsertionTimer_.elapsedMsecSinceStopwatchStart();
    // scanInsertionTimer_.addMeasurementMsec(insertiontimeElapsed);
  }

  return true;
}

std::shared_ptr<small_gicp::PointCloud> Mapper::cropPreparePointCloud(const open3d::geometry::PointCloud& input,
                                                                      const Eigen::Isometry3d& center_pose, double radius, int normal_knn,
                                                                      int num_threads) const {
  const std::size_t N = input.points_.size();
  if (N == 0) throw std::runtime_error("cropPreparePointCloud: input point cloud is empty");

  const bool has_normals = input.HasNormals() && input.normals_.size() == N;

  if (has_normals) {
    std::cout << "\033[38;5;208m[Mapper] Input cloud HAS normals.\033[0m" << std::endl;
  } else {
    std::cout << "\033[38;5;208m[Mapper] Input cloud does NOT have normals.\033[0m" << std::endl;
  }

  //-------------------------------------------------------------------
  // 1. Select indices that fall inside the radius
  //-------------------------------------------------------------------
  const Eigen::Vector3d centre = center_pose.translation();
  const double radius2 = radius * radius;

  std::vector<std::size_t> selected;
  selected.reserve(N);

  constexpr std::size_t kParallelThreshold = 10'000;
  if (N >= kParallelThreshold) {
    //-----------------------------------------------------------------
    // Parallel selection – avoid false sharing by writing
    // into per-thread buffers first.
    //-----------------------------------------------------------------
    int nThreads = 1;
#ifdef _OPENMP
    nThreads = 8;
#endif
    std::vector<std::vector<std::size_t>> thread_sel(nThreads);

#pragma omp parallel
    {
      int tid = 0;
#ifdef _OPENMP
      tid = 8;
#endif
      auto& local = thread_sel[tid];
      local.reserve(N / nThreads + 64);

#pragma omp for nowait schedule(static)
      for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
        const Eigen::Vector3d& p = input.points_[i];
        if ((p - centre).squaredNorm() <= radius2) local.push_back(static_cast<std::size_t>(i));
      }
    }

    // Flatten per-thread buffers
    std::size_t total = 0;
    for (const auto& v : thread_sel) total += v.size();
    selected.resize(total);

    std::size_t pos = 0;
    for (const auto& v : thread_sel) {
      std::copy(v.begin(), v.end(), selected.begin() + pos);
      pos += v.size();
    }
  } else {
    // Serial fall-back
    for (std::size_t i = 0; i < N; ++i) {
      const Eigen::Vector3d& p = input.points_[i];
      if ((p - centre).squaredNorm() <= radius2) selected.emplace_back(i);
    }
  }

  //-------------------------------------------------------------------
  // 2. Allocate destination cloud
  //-------------------------------------------------------------------
  const std::size_t M = selected.size();
  auto cropped = std::make_shared<small_gicp::PointCloud>();
  cropped->resize(M);  // alloc points / normals / cov

  //-------------------------------------------------------------------
  // 3. Copy points (+ normals if present) in ONE pass
  //-------------------------------------------------------------------
  const Eigen::Vector3d* src_points = input.points_.data();
  const Eigen::Vector3d* src_normals = has_normals ? input.normals_.data() : nullptr;
  Eigen::Vector4d* dst_points = cropped->points.data();
  Eigen::Vector4d* dst_normals = cropped->normals.data();

  if (M >= kParallelThreshold) {
#pragma omp parallel for schedule(static)
    for (std::int64_t j = 0; j < static_cast<std::int64_t>(M); ++j) {
      const std::size_t i = selected[j];

      // point xyz → xyz1
      dst_points[j].head<3>() = src_points[i];
      dst_points[j][3] = 1.0;

      if (has_normals) {
        std::memcpy(dst_normals[j].data(), src_normals[i].data(), 3 * sizeof(double));
        dst_normals[j][3] = 0.0;
      } else {
        dst_normals[j].setZero();  // filled later if we estimate
      }
      // cropped->cov(j).setZero();       // if you store covariances
    }
  } else {
    for (std::size_t j = 0; j < M; ++j) {
      const std::size_t i = selected[j];

      dst_points[j].head<3>() = src_points[i];
      dst_points[j][3] = 1.0;

      if (has_normals) {
        std::memcpy(dst_normals[j].data(), src_normals[i].data(), 3 * sizeof(double));
        dst_normals[j][3] = 0.0;
      } else {
        dst_normals[j].setZero();
      }
      // cropped->cov(j).setZero();
    }
  }

  return cropped;
}

void Mapper::copyOrEstimateNormals(const open3d::geometry::PointCloud& src, small_gicp::PointCloud& dst,
                                   small_gicp::KdTree<small_gicp::PointCloud>& dst_tree, int normal_knn, int num_threads) {
  const std::size_t N = dst.size();
  assert(N == src.points_.size());

  constexpr std::size_t OMP_THRESHOLD = 16384;  // Use single-threaded below this
  constexpr std::size_t BLOCK_SIZE = 256;       // Cache-aware block size for OMP scheduling

  if (src.HasNormals() && src.normals_.size() == N) {
    if (dst.normals.size() != N) dst.normals.resize(N);

    assert((reinterpret_cast<uintptr_t>(dst.normals.data()) & 0xF) == 0 && "dst.normals is not 16-byte aligned");

    const Eigen::Vector3d* src_ptr = src.normals_.data();
    Eigen::Vector4d* dst_ptr = dst.normals.data();

    if (N < OMP_THRESHOLD || (num_threads == 1)) {
      // Single-threaded copy for small clouds
      for (std::size_t i = 0; i < N; ++i) {
        std::memcpy(dst_ptr[i].data(), src_ptr[i].data(), 3 * sizeof(double));
        dst_ptr[i][3] = 0.0;
      }
    } else {
      if (num_threads <= 0) num_threads = static_cast<int>(std::thread::hardware_concurrency());
// Multi-threaded, cache-blocked copy
#pragma omp parallel for schedule(static, BLOCK_SIZE) num_threads(num_threads)
      for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
        std::memcpy(dst_ptr[i].data(), src_ptr[i].data(), 3 * sizeof(double));
        dst_ptr[i][3] = 0.0;
      }
    }
  } else {
    // Fallback: estimate normals if Open3D cloud lacks them
    // If Open3D cloud lacks normals, estimate them using small_gicp
    if (num_threads <= 0) {
      num_threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    estimate_normals_omp(dst, dst_tree, normal_knn, num_threads);
    // estimate_covariances_omp(dst, dst_tree, normal_knn, num_threads);
    std::cout << "\033[35m[Mapper] Normals were estimated (not copied from Open3D cloud).\033[0m" << std::endl;
  }
}

Eigen::Vector3d Mapper::computeCentroid(const small_gicp::PointCloud& pc) {
  if (pc.empty()) {
    throw std::runtime_error("computeCentroid: input point cloud is empty");
  }

  constexpr std::size_t kParallelThreshold = 8'192 * 2;  // tweak if needed

  const std::size_t N = pc.size();
  if (N >= kParallelThreshold) {
    Eigen::Vector3d global_sum = Eigen::Vector3d::Zero();

#pragma omp parallel
    {
      Eigen::Vector3d local_sum = Eigen::Vector3d::Zero();

#pragma omp for nowait schedule(static)
      for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
        local_sum += pc.point(static_cast<std::size_t>(i)).head<3>();
      }

#pragma omp atomic
      global_sum.x() += local_sum.x();
#pragma omp atomic
      global_sum.y() += local_sum.y();
#pragma omp atomic
      global_sum.z() += local_sum.z();
    }
    return global_sum / static_cast<double>(N);
  }

  // ── Serial fallback (fastest for small clouds)
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < N; ++i) sum += pc.point(i).head<3>();

  return sum / static_cast<double>(N);
}

void Mapper::translatePointCloud(small_gicp::PointCloud& pc, const Eigen::Vector3d& t) {
  if (t.isZero(0)) {
    return;
  }

  const std::size_t N = pc.size();
  constexpr std::size_t kParallelThreshold = 8'192 * 2;  // adjust for your CPU

  if (N >= kParallelThreshold) {
#pragma omp parallel for schedule(static)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(N); ++i) {
      pc.point(static_cast<std::size_t>(i)).head<3>() += t;
    }
    return;
  }

  for (std::size_t i = 0; i < N; ++i) pc.point(i).head<3>() += t;
}

Mapper::PointCloud Mapper::getAssembledMapPointCloudVisualization() const {
  const size_t nSubmaps = submaps_->getNumSubmaps();
  if (nSubmaps == 0) {
    std::cerr << "No submaps. Returning empty cloud." << std::endl;
    return PointCloud();
  }

  std::vector<size_t> per_submap_sizes(nSubmaps, 0);
  std::vector<std::vector<Eigen::Vector3d>> thread_points(nSubmaps);

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(nSubmaps); ++i) {
    const auto& submap = submaps_->getSubmap(i);
    const auto& voxel_map = submap.getVoxelMap();
    auto& points_out = thread_points[i];

    // Check if voxel_map has any occupied voxels
    size_t voxel_count = 0;
    for (const auto& kv : voxel_map.voxels_) {
      if (!kv.second.idxs_.empty()) ++voxel_count;
    }

    if (voxel_count > 0) {
      // Use voxel map centers
      points_out.reserve(voxel_count);
      for (const auto& kv : voxel_map.voxels_) {
        if (!kv.second.idxs_.empty()) {
          points_out.push_back(voxel_map.getCenterFromKey(kv.first));
        }
      }
    } else {
      // Fallback: use raw point cloud
      const auto& pc = submap.getMapPointCloud();
      points_out.assign(pc.points_.begin(), pc.points_.end());
    }
    per_submap_sizes[i] = points_out.size();
  }

  // Print verbose info
  size_t total_points = 0;
  for (size_t i = 0; i < nSubmaps; ++i) {
    // std::cout << "[AssembledMap] Submap " << i << " has " << per_submap_sizes[i] << " points." << std::endl;
    total_points += per_submap_sizes[i];
  }
  // std::cout << "[AssembledMap] Total points in assembled map: " << total_points << std::endl;

  // Merge results
  PointCloud cloud;
  cloud.points_.reserve(total_points);
  for (auto& pts : thread_points) {
    cloud.points_.insert(cloud.points_.end(), std::make_move_iterator(pts.begin()), std::make_move_iterator(pts.end()));
  }
  return cloud;
}

Mapper::PointCloud Mapper::getAssembledMapPointCloud() const {
  const int nPoints = submaps_->getTotalNumPoints();

  if (nPoints == 0) {
    std::cerr << "No points in the assembled map. Returning empty cloud." << std::endl;
    return PointCloud();
  }

  PointCloud cloud;
  const Submap& activeSubmap = getActiveSubmap();
  cloud.points_.reserve(nPoints);
  if (activeSubmap.getMapPointCloud().HasColors()) {
    cloud.colors_.reserve(nPoints);
  }
  if (activeSubmap.getMapPointCloud().HasNormals()) {
    cloud.normals_.reserve(nPoints);
  }

  for (size_t j = 0; j < submaps_->getNumSubmaps(); ++j) {
    const PointCloud submap = submaps_->getSubmap(j).getMapPointCloudCopy();
    for (size_t i = 0; i < submap.points_.size(); ++i) {
      cloud.points_.push_back(submap.points_.at(i));
      if (submap.HasColors()) {
        cloud.colors_.push_back(submap.colors_.at(i));
      }
      if (submap.HasNormals()) {
        cloud.normals_.push_back(submap.normals_.at(i));
      }
    }
  }

  return cloud;
}

void Mapper::checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const {
  if (isCheckTransformChainingAndPrintResult && odomToRangeSensorBuffer_.size() > 70 && mapToRangeSensorBuffer_.size() > 70) {
    /*const auto odom1 = odomToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto odom2 = odomToRangeSensorBuffer_.latest_measurement(20).transform_;
    const auto start = mapToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto gt = mapToRangeSensorBuffer_.latest_measurement(20).transform_;
    const Transform mapMotion = start.inverse() * gt;
    const Transform odomMotion = odom1.inverse() * odom2;
    std::cout << "start      :  " << asString(start) << "\n";
    std::cout << "gt         :  " << asString(gt) << "\n";
    std::cout << "gt computed:  " << asString(start * mapMotion) << "\n";
    std::cout << "est        : " << asString(start * odomMotion) << "\n\n";*/
  }
}

} /* namespace o3d_slam */
