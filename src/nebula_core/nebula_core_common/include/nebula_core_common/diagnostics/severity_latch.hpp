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
#include "nebula_core_common/util/thread_safety_annotations.hpp"

#include <mutex>
#include <optional>
#include <utility>

namespace nebula::diagnostics
{

/**
 * @brief Keeps the most severe status submitted since the last `report()`.
 *
 * If nothing was submitted since then, the last submitted status is reported again.
 *
 * @tparam T The type of the latched status
 */
template <typename T>
class SeverityLatch
{
public:
  struct Report
  {
    Level level{};
    T status{};
  };

  /// @brief The latched status, or nothing if none has been submitted yet.
  [[nodiscard]] std::optional<Report> report()
  {
    std::lock_guard lock(mutex_);

    // Reported the status, allow it to be reset in the next `submit()`.
    should_reset_ = current_.has_value();
    return current_;
  }

  void submit(Level level, T status)
  {
    std::lock_guard lock(mutex_);

    // Only the highest severity status in between two `report()`s is kept.
    // If multiple statuses with the same severity are submitted, only the last one is kept.
    // If the current status is from a previous cycle (demarked by `should_reset_`), it is
    // replaced with the new status, even if that new status has a lower severity.
    if (!current_ || level >= current_->level || should_reset_) {
      current_ = Report{level, std::move(status)};
      // The status of the previous cycle is gone, latch within the new cycle from here on.
      should_reset_ = false;
    }
  }

private:
  std::mutex mutex_;
  //! Indicates that the current status has been reported at least once via `report()`. In order to
  //! have something to report in the next `report()` the status is kept until the next one is
  //! `submit()`-ed. This member signals that the status has been reported and can be reset.
  bool should_reset_ NEBULA_TSA_GUARDED_BY(mutex_) = false;
  std::optional<Report> current_ NEBULA_TSA_GUARDED_BY(mutex_);
};

}  // namespace nebula::diagnostics
