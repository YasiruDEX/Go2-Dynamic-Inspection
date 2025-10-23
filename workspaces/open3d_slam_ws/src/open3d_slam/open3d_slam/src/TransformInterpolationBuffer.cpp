#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include <algorithm>
#include <iostream>

namespace o3d_slam {

/* ---------------------------------------------------------------- ctor */

// TransformInterpolationBuffer::TransformInterpolationBuffer() : TransformInterpolationBuffer(2000) {}

TransformInterpolationBuffer::TransformInterpolationBuffer(std::size_t bufferSize) : bufferSizeLimit_(bufferSize) {}

/* ------------------------------------------------------------ push & cfg */

void TransformInterpolationBuffer::push(const Time& time, const Transform& tf) {
  std::unique_lock lk(m_);

  if (!transforms_.empty()) {
    if (time < transforms_.front().time_) {
      std::cerr << "TransformInterpolationBuffer:: you are trying to push "
                   "something earlier than the earliest measurement, this "
                   "should not happen \nignnoring the measurement \nTime: "
                << toSecondsSinceFirstMeasurement(time) << '\n'
                << "earliest time: " << toSecondsSinceFirstMeasurement(transforms_.front().time_) << '\n';
      return;
    }
    if (time < transforms_.back().time_) {
      std::cerr << "TransformInterpolationBuffer:: you are trying to push "
                   "something out of order, this can only happen in the "
                   "beginning. \nignoring this measurement \nTime: "
                << toSecondsSinceFirstMeasurement(time) << '\n'
                << "latest time: " << toSecondsSinceFirstMeasurement(transforms_.back().time_) << '\n';
      return;
    }
  }
  transforms_.push_back({time, tf});
  removeOldMeasurementsIfNeeded();
  isBufferExtended = true;
}

void TransformInterpolationBuffer::setSizeLimit(std::size_t limit) {
  std::unique_lock lk(m_);
  bufferSizeLimit_ = limit;
  removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::clear() {
  std::unique_lock lk(m_);
  transforms_.clear();
}

/* ------------------------------------------------------ private helper */

void TransformInterpolationBuffer::removeOldMeasurementsIfNeeded() {
  while (transforms_.size() > bufferSizeLimit_) transforms_.pop_front();
}

/* ----------------------------------------------------------- queries */

bool TransformInterpolationBuffer::empty() const {
  std::shared_lock lk(m_);
  return transforms_.empty();
}

std::size_t TransformInterpolationBuffer::size() const {
  std::shared_lock lk(m_);
  return transforms_.size();
}

std::size_t TransformInterpolationBuffer::size_limit() const {
  return bufferSizeLimit_;
}

Time TransformInterpolationBuffer::earliest_time() const {
  std::shared_lock lk(m_);
  if (transforms_.empty()) throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  return transforms_.front().time_;
}

Time TransformInterpolationBuffer::latest_time() const {
  std::shared_lock lk(m_);
  if (transforms_.empty()) throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  return transforms_.back().time_;
}

bool TransformInterpolationBuffer::has(const Time& time) const {
  std::shared_lock lk(m_);
  return !transforms_.empty() && transforms_.front().time_ <= time && time <= transforms_.back().time_;
}

bool TransformInterpolationBuffer::has_query(const Time& time) const {
  std::shared_lock lk(m_);
  return !transforms_.empty() && transforms_.front().time_ <= time && time <= transforms_.back().time_ + std::chrono::duration<double>(0.5);
}

/* ---------------------------------------------------- lookup + EXTRAPOLATION */

Transform TransformInterpolationBuffer::lookup(const Time& time) const {
  std::shared_lock lk(m_);
  // Check for empty buffer
  if (transforms_.empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  }

  // ---------- 1. Outside range → extrapolate --------------------------
  // If the query time is before the earliest buffered transform
  if (time < transforms_.front().time_) {
    // If there is only one transform, return it (no velocity info for extrapolation)
    if (transforms_.size() == 1) {
      return transforms_.front().transform_;
    }

    std::cerr << "[TransformInterpolationBuffer] Extrapolation requested: query time ("
              << toSecondsSinceFirstMeasurement(time)
              << ") is before earliest buffered transform ("
              << toSecondsSinceFirstMeasurement(transforms_.front().time_)
              << "). Using velocity between first two entries for extrapolation.\n";

    return extrapolate(transforms_.front(), *std::next(transforms_.begin()), time).transform_;
  }

  if (time > transforms_.back().time_) {
    if (transforms_.size() == 1) return transforms_.back().transform_;

    std::cerr << "[TransformInterpolationBuffer] Extrapolation requested: query time (" << toSecondsSinceFirstMeasurement(time)
              << ") is after latest buffered transform (" << toSecondsSinceFirstMeasurement(transforms_.back().time_)
              << "). Using velocity between last two entries for extrapolation.\n";
    return extrapolate(*std::prev(transforms_.end(), 2), transforms_.back(), time).transform_;
  }

  // ---------- 2. Inside range → interpolate (O(log N) search) ---------
  auto right = std::lower_bound(transforms_.begin(), transforms_.end(), time,
                                [](const TimestampedTransform& a, const Time& t) { return a.time_ < t; });

  if (right == transforms_.end()) return transforms_.back().transform_;
  if (right->time_ == time) return right->transform_;

  const auto& left = *std::prev(right);
  return interpolate(left, *right, time).transform_;
}

/* ------------------------------------- latest_measurement (read / write) */

const TimestampedTransform& TransformInterpolationBuffer::latest_measurement(int offset) const {
  std::shared_lock lk(m_);
  if (transforms_.empty()) throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  return *std::prev(transforms_.end(), offset + 1);
}

TimestampedTransform& TransformInterpolationBuffer::latest_measurement(int offset) {
  std::shared_lock lk(m_);
  if (transforms_.empty()) throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  return *std::prev(transforms_.end(), offset + 1);
}

/* ------------------------------------- misc utilities with logging ----- */

void TransformInterpolationBuffer::printTimesCurrentlyInBuffer() const {
  std::shared_lock lk(m_);
  for (const auto& t : transforms_) std::cout << toSecondsSinceFirstMeasurement(t.time_) << '\n';
}

void TransformInterpolationBuffer::applyToAllElementsInTimeInterval(const Transform& T, const Time& begin, const Time& end) {
  std::unique_lock lk(m_);
  for (auto& elem : transforms_)
    if (elem.time_ >= begin && elem.time_ <= end) elem.transform_ = elem.transform_ * T;
}

Transform getTransform(const Time& time, const TransformInterpolationBuffer& buffer) {
  if (time < buffer.earliest_time()) {
    std::cerr << "TransformInterpolationBuffer:: you are trying to get a "
                 "transform that is in the past, this should not happen \n";
    return buffer.lookup(time);
  }
  if (time > buffer.latest_time()) {
    std::cerr << "TransformInterpolationBuffer:: you are trying to get a "
                 "transform that is in the future, this should not happen \n";
    return buffer.lookup(time);
  }
  return buffer.lookup(time);
}

}  // namespace o3d_slam
