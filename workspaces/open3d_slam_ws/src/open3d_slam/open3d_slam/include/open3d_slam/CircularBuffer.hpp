#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace o3d_slam {

template <typename T>
class CircularBuffer {
 public:
  CircularBuffer() = default;

  std::deque<T> snapshot() const {
    std::lock_guard<std::mutex> lck(m_);
    return data_;
  }

  void set_size_limit(size_t size) {
    std::lock_guard<std::mutex> lck(m_);
    bufferSizeLimit_ = size;
    trim_unsafe();
  }

  void push(const T& data) {
    {
      std::lock_guard<std::mutex> lck(m_);
      data_.push_back(data);
      trim_unsafe();
    }
    cv_.notify_one();
  }

  const T& peek_front() const { return data_.front(); }
  const T& peek_back() const { return data_.back(); }

  T pop() {
    std::lock_guard<std::mutex> lck(m_);
    T copy = std::move(data_.front());
    data_.pop_front();
    return copy;
  }

  bool empty() const { return data_.empty(); }
  size_t size_limit() const { return bufferSizeLimit_; }
  size_t size() const { return data_.size(); }

  void clear() {
    std::lock_guard<std::mutex> lck(m_);
    data_.clear();
  }

  const std::deque<T>& getImplementation() const { return data_; }
  std::deque<T>* getImplementationPtr() { return &data_; }

  /// blocks until an element is available, or the buffer is closed
  T wait_and_pop() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return !data_.empty() || !open_; });
    if (data_.empty())  // open_ must be false
      throw std::runtime_error("buffer closed");
    T v = std::move(data_.front());
    data_.pop_front();
    return v;
  }

  /// non-blocking; returns std::nullopt if empty
  std::optional<T> try_pop() {
    std::lock_guard<std::mutex> lck(m_);
    if (data_.empty()) return std::nullopt;
    T v = std::move(data_.front());
    data_.pop_front();
    return v;
  }

  /// signal all waiting threads to finish
  void close() {
    {
      std::lock_guard<std::mutex> lck(m_);
      open_ = false;
    }
    cv_.notify_all();
  }

  bool open() const { return open_; }

 private:
  void trim_unsafe() {
    while (data_.size() > bufferSizeLimit_) data_.pop_front();
  }

  std::deque<T> data_;
  size_t bufferSizeLimit_ = 100;

  mutable std::mutex m_;
  std::condition_variable cv_;
  bool open_ = true;
};

}  // namespace o3d_slam
