/*
 * Transform.cpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Transform.hpp"
#include <glog/logging.h>
#include <iostream>
#include <string>
#include "Eigen/Geometry"

namespace o3d_slam {

TimestampedTransform interpolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& query) {
  /* --- basic validation -------------------------------------------------- */
  if (start.time_ > end.time_) throw std::runtime_error("interpolate(): start.time_ > end.time_");
  if (query < start.time_ || query > end.time_) throw std::runtime_error("interpolate(): query time is outside [start,end]");

  /* --- trivial cases ----------------------------------------------------- */
  if (query == start.time_) return start;
  if (query == end.time_) return end;

  /* --- compute blend factor --------------------------------------------- */
  constexpr double kEps = 1e-12;
  const double duration = toSeconds(end.time_ - start.time_);
  if (duration < kEps)  // practically the same stamp
    return {query, start.transform_};

  const double factor = toSeconds(query - start.time_) / duration;  // ∈ (0,1)

  /* --- blend translation ------------------------------------------------- */
  const Eigen::Vector3d p = start.transform_.translation() + factor * (end.transform_.translation() - start.transform_.translation());

  /* --- blend rotation (SLERP) ------------------------------------------- */
  const Eigen::Quaterniond q_start(start.transform_.rotation());
  const Eigen::Quaterniond q_end(end.transform_.rotation());
  const Eigen::Quaterniond q = q_start.slerp(factor, q_end);

  Transform tf(q);
  tf.translation() = p;
  return {query, tf};
}

/* ------------------------------------------------------------------------- */
/*  Extrapolate *outside* the interval.                                      */
/*  If query < start.time_  : it linearly projects backwards.                 */
/*  If query > end.time_    : it linearly projects forwards.                  */
/*  Inside the interval it simply delegates to interpolate().               */
/* ------------------------------------------------------------------------- */
TimestampedTransform extrapolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& query) {
  /* swap to ensure start.time_ ≤ end.time_ for the math */
  const TimestampedTransform& s = (start.time_ <= end.time_) ? start : end;
  const TimestampedTransform& e = (start.time_ <= end.time_) ? end : start;

  /* inside the segment → ordinary interpolation */
  if (query >= s.time_ && query <= e.time_) return interpolate(s, e, query);

  /* identical stamps → cannot extrapolate velocity */
  constexpr double kEps = 1e-12;
  const double duration = toSeconds(e.time_ - s.time_);
  if (duration < kEps) return {query, s.transform_};

  /* compute factor that may be <0 or >1 */
  const double factor = toSeconds(query - s.time_) / duration;

  const Eigen::Vector3d p = s.transform_.translation() + factor * (e.transform_.translation() - s.transform_.translation());

  const Eigen::Quaterniond q_s(s.transform_.rotation());
  const Eigen::Quaterniond q_e(e.transform_.rotation());
  const Eigen::Quaterniond q = q_s.slerp(factor, q_e);

  Transform tf(q);
  tf.translation() = p;
  return {query, tf};
}

Transform makeTransform(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Transform transform(q);
  transform.translation() = p;
  return transform;
}

} /* namespace o3d_slam */
