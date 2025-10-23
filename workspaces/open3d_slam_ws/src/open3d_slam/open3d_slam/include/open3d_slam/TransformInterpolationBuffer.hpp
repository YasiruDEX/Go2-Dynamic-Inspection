/*
 * TransformInterpolationBuffer.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#pragma once
#include <atomic>
#include <deque>
#include <limits>
#include <mutex>
#include <shared_mutex>

#include "open3d_slam/Transform.hpp"
#include "open3d_slam/time.hpp"

namespace o3d_slam {

class TransformInterpolationBuffer {
 public:
  explicit TransformInterpolationBuffer(std::size_t bufferSize = 500);
  // TransformInterpolationBuffer();                         // default size
  ~TransformInterpolationBuffer() = default;

  /* --- writer ---------------------------------------------------------- */
  void setSizeLimit(std::size_t bufferSizeLimit);
  void push(const Time& time, const Transform& transform);
  void clear();

  /* --- queries --------------------------------------------------------- */
  bool has(const Time& time) const;
  bool has_query(const Time& time) const;
  bool empty() const;
  Time earliest_time() const;
  Time latest_time() const;
  std::size_t size() const;
  std::size_t size_limit() const;

  Transform lookup(const Time& time) const;  // now extrapolates if needed

  const TimestampedTransform& latest_measurement(int offsetFromLastElement = 0) const;
  TimestampedTransform& latest_measurement(int offsetFromLastElement = 0);

  void printTimesCurrentlyInBuffer() const;
  void applyToAllElementsInTimeInterval(const Transform& T, const Time& begin, const Time& end);

 private:
  void removeOldMeasurementsIfNeeded();

  std::deque<TimestampedTransform> transforms_;
  std::size_t bufferSizeLimit_;
  mutable std::shared_mutex m_;
  std::atomic<bool> isBufferExtended{false};
};

/* helper with same logging strings, now calls the new extrapolating lookup */
Transform getTransform(const Time& time, const TransformInterpolationBuffer& buffer);

}  // namespace o3d_slam