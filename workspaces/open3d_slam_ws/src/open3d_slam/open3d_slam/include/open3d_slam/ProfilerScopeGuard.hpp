#pragma once

#include <string>

#ifdef ENABLE_PROFILING

#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>

class ProfilerScopeGuard {
 public:
  ProfilerScopeGuard(const std::string& label, const std::string& csv_path, const std::string& extra_info = "")
      : label_(label), file_path_(csv_path), extra_info_(extra_info) {
    start_time_ = std::chrono::steady_clock::now();
    thread_id_ = std::this_thread::get_id();
  }

  ~ProfilerScopeGuard() {
    const auto end = std::chrono::steady_clock::now();
    const double duration_ms = std::chrono::duration<double, std::milli>(end - start_time_).count();

    std::ostringstream oss;
    oss << label_ << "," << duration_ms << "," << thread_id_ << "," << extra_info_ << "\n";

    std::lock_guard<std::mutex> lock(write_mutex_);
    if (is_first_write_) {
      std::ofstream out(file_path_, std::ios::trunc);  // Truncate file on first write
      if (out.is_open()) {
        out << "Label,Duration(ms),ThreadID,Info\n";
        out << oss.str();
        is_first_write_ = false;
      }
    } else {
      std::ofstream out(file_path_, std::ios::app);
      if (out.is_open()) {
        out << oss.str();
      }
    }
  }

 private:
  std::string label_;
  std::string file_path_;
  std::string extra_info_;
  std::thread::id thread_id_;
  std::chrono::steady_clock::time_point start_time_;

  static inline std::mutex write_mutex_;
  static inline bool is_first_write_ = true;  // Initialize to true
};

#else  // ENABLE_PROFILING not defined

// No-op version
class ProfilerScopeGuard {
 public:
  ProfilerScopeGuard(const std::string&, const std::string&, const std::string& = "") {}
};

#endif  // ENABLE_PROFILING
