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

#include "nebula_core_common/diagnostics/diagnostic_task.hpp"
#include "nebula_core_common/util/clock.hpp"

#include <rcpputils/thread_safety_annotations.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace nebula::diagnostics
{

/**
 * @brief Monitor whether a routine is alive (is running at least once in a given period).
 *
 * This can be used for getting notified when a certain routine gets stuck, e.g. when packets stop
 * arriving or when pointclouds stop being published.
 */
class LivenessMonitor : public DiagnosticTask
{
public:
  /**
   * @brief Create and activate a new liveness monitor.
   *
   * The monitored routine has to call `tick()` on every iteration to prove its liveness. If there
   * is no call for a length of `timeout`, the routine is declared dead.
   *
   * @param name The name of the task
   * @param clock The clock that `timeout` is measured on
   * @param timeout The time after the last call to `tick()` where the routine is declared dead
   */
  LivenessMonitor(
    const std::string & name, std::shared_ptr<const util::Clock> clock,
    std::chrono::nanoseconds timeout)
  : DiagnosticTask(name), clock_(std::move(clock)), timeout_(timeout), last_tick_(clock_->now())
  {
  }

  /**
   * @brief Proves the liveness of a routine. Has to be called frequently.
   *
   * A call to `tick()` resets the internal timer. If the timer is not reset at least once before it
   * expires (within `timeout`), the monitored routine is pronounced dead.
   */
  void tick()
  {
    std::lock_guard lock(mutex_);
    last_tick_ = clock_->now();
  }

  void run(DiagnosticStatus & status) override
  {
    std::lock_guard lock(mutex_);

    const std::chrono::nanoseconds lateness = clock_->now() - last_tick_;
    const bool is_live = lateness < timeout_;

    uint8_t severity = DiagnosticStatus::OK;
    std::string message = "Alive";
    std::string value = "true";

    if (!is_live) {
      severity = DiagnosticStatus::ERROR;
      message = "Dead";
      value = "false";
    }

    using seconds = std::chrono::duration<double>;
    using milliseconds = std::chrono::duration<double, std::milli>;

    set_summary(status, severity, message);
    add_value(status, "Is alive", value);
    add_value(status, "Last tick [s]", std::to_string(seconds(last_tick_).count()));
    add_value(status, "Lateness [ms]", std::to_string(milliseconds(lateness).count()));
  }

private:
  std::shared_ptr<const util::Clock> clock_;
  std::chrono::nanoseconds timeout_;

  std::mutex mutex_;
  std::chrono::nanoseconds last_tick_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace nebula::diagnostics
