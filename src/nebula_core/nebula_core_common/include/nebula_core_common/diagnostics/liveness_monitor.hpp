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

#include "nebula_core_common/diagnostics/level.hpp"
#include "nebula_core_common/util/clock.hpp"
#include "nebula_core_common/util/thread_safety_annotations.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

namespace nebula::diagnostics
{

/**
 * @brief Monitor whether a routine is alive (is running at least once in a given period).
 *
 * This can be used for getting notified when a certain routine gets stuck, e.g. when packets stop
 * arriving or when pointclouds stop being published.
 */
class LivenessMonitor
{
public:
  struct Report
  {
    bool is_alive{};
    /// The time of the last call to `tick()`, on the monitor's clock
    std::chrono::nanoseconds last_tick{};
    /// The time since the last call to `tick()`
    std::chrono::nanoseconds lateness{};

    [[nodiscard]] Level level() const { return is_alive ? Level::OK : Level::ERROR; }
  };

  /**
   * @brief Create and activate a new liveness monitor.
   *
   * The monitored routine has to call `tick()` on every iteration to prove its liveness. If there
   * is no call for a length of `timeout`, the routine is declared dead.
   *
   * @param clock The clock that `timeout` is measured on
   * @param timeout The time after the last call to `tick()` where the routine is declared dead
   */
  LivenessMonitor(std::shared_ptr<const util::Clock> clock, std::chrono::nanoseconds timeout)
  : clock_(std::move(clock)), timeout_(timeout), last_tick_(clock_->now())
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

  [[nodiscard]] Report report()
  {
    std::lock_guard lock(mutex_);

    Report report;
    report.last_tick = last_tick_;
    report.lateness = clock_->now() - last_tick_;
    report.is_alive = report.lateness < timeout_;
    return report;
  }

private:
  std::shared_ptr<const util::Clock> clock_;
  std::chrono::nanoseconds timeout_;

  std::mutex mutex_;
  std::chrono::nanoseconds last_tick_ NEBULA_TSA_GUARDED_BY(mutex_);
};

}  // namespace nebula::diagnostics
