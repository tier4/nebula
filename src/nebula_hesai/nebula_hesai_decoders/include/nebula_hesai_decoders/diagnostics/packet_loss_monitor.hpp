// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <nebula_core_common/diagnostics/level.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <nebula_core_common/util/thread_safety_annotations.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>

namespace nebula::drivers
{

/// @brief Counts the packets that were lost between two reports
class PacketLossMonitor
{
public:
  struct Report
  {
    /// The number of packets lost since the last report
    uint64_t lost_packets{};
    /// The number of lost packets from which on an error is reported
    uint64_t error_threshold{};
    std::chrono::nanoseconds time_since_last_report{};

    [[nodiscard]] diagnostics::Level level() const
    {
      if (lost_packets == 0) {
        return diagnostics::Level::OK;
      }
      return lost_packets < error_threshold ? diagnostics::Level::WARN : diagnostics::Level::ERROR;
    }
  };

  PacketLossMonitor(uint64_t error_threshold, std::shared_ptr<const util::Clock> clock)
  : clock_(std::move(clock)), error_threshold_(error_threshold), last_report_time_(clock_->now())
  {
  }

  void on_lost(uint64_t n_lost)
  {
    std::lock_guard lock(mutex_);
    n_lost_packets_ += n_lost;
  }

  /// @brief Report the packets lost since the last call, and start counting anew
  [[nodiscard]] Report report()
  {
    std::lock_guard lock(mutex_);

    const auto now = clock_->now();

    Report report;
    report.lost_packets = n_lost_packets_;
    report.error_threshold = error_threshold_;
    report.time_since_last_report = now - last_report_time_;

    last_report_time_ = now;
    n_lost_packets_ = 0;
    return report;
  }

private:
  std::shared_ptr<const util::Clock> clock_;
  uint64_t error_threshold_;

  std::mutex mutex_;
  std::chrono::nanoseconds last_report_time_ NEBULA_TSA_GUARDED_BY(mutex_);
  uint64_t n_lost_packets_ NEBULA_TSA_GUARDED_BY(mutex_){};
};

}  // namespace nebula::drivers
