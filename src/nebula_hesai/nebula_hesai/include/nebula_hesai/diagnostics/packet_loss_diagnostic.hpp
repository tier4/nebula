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

#include <nebula_core_common/diagnostics/diagnostic_task.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <rcpputils/thread_safety_annotations.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace nebula::ros
{

class PacketLossDiagnosticTask : public diagnostics::DiagnosticTask
{
public:
  PacketLossDiagnosticTask(uint64_t error_threshold, std::shared_ptr<const util::Clock> clock)
  : DiagnosticTask("Packet loss status"),
    clock_(std::move(clock)),
    last_run_time_(clock_->now()),
    error_threshold_(error_threshold)
  {
  }

  void on_lost(uint64_t n_lost)
  {
    std::lock_guard lock(mutex_);
    n_lost_packets_ += n_lost;
  }

  void run(diagnostics::DiagnosticStatus & status) override
  {
    std::lock_guard lock(mutex_);

    const auto now = clock_->now();
    const auto dt = now - last_run_time_;
    last_run_time_ = now;

    using seconds = std::chrono::duration<double>;
    diagnostics::add_value(status, "Lost packets", n_lost_packets_);
    diagnostics::add_value(status, "Error threshold", error_threshold_);
    diagnostics::add_value(
      status, "Time since last update [s]", std::to_string(seconds(dt).count()));

    if (n_lost_packets_ == 0) {
      diagnostics::set_summary(
        status, diagnostic_msgs::msg::DiagnosticStatus::OK, "No packet loss");
    } else if (n_lost_packets_ < error_threshold_) {
      diagnostics::set_summary(
        status, diagnostic_msgs::msg::DiagnosticStatus::WARN, "Slight packet loss");
    } else {
      diagnostics::set_summary(
        status, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Severe packet loss");
    }

    n_lost_packets_ = 0;
  }

private:
  std::shared_ptr<const util::Clock> clock_;
  std::chrono::nanoseconds last_run_time_;
  uint64_t error_threshold_;

  std::mutex mutex_;
  uint64_t n_lost_packets_ {}
  RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace nebula::ros
