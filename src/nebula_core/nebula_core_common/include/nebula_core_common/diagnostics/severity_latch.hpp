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

#include <rcpputils/thread_safety_annotations.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace nebula::diagnostics
{

/**
 * @brief Reports the most severe status submitted since the last `run()`.
 *
 * If nothing was submitted since then, the last submitted status is reported again.
 */
class SeverityLatch : public DiagnosticTask
{
public:
  using DiagnosticTask::DiagnosticTask;

  void run(DiagnosticStatus & status) override
  {
    std::lock_guard lock(mutex_);

    if (!current_status_) {
      set_summary(status, DiagnosticStatus::STALE, "No status available");
      return;
    }

    status.values.insert(
      status.values.end(), current_status_->values.begin(), current_status_->values.end());
    set_summary(status, current_status_->level, current_status_->message);

    // Reported the status, allow it to be reset in the next `submit()`.
    should_reset_ = true;
  }

  void submit(const DiagnosticStatus & status)
  {
    std::lock_guard lock(mutex_);

    if (!current_status_) {
      current_status_ = status;
      return;
    }

    // Only the highest severity status in between two `run()`s is reported.
    // If multiple statuses with the same severity are submitted, only the last one is reported.
    // If the `current_status_` is from a previous cycle (demarked by `should_reset_`), it is
    // replaced with the new status, even if that new status has a lower severity.
    if (status.level >= current_status_->level || should_reset_) {
      current_status_ = status;
    }
  }

private:
  std::mutex mutex_;
  //! Indicates that `current_status_` has been reported at least once via `run()`. In order to
  //! have something to report in the next `run()` the status is kept until the next one is
  //! `submit()`-ed. This member signals that the status has been reported and can be reset.
  bool should_reset_ RCPPUTILS_TSA_GUARDED_BY(mutex_) = false;
  std::optional<DiagnosticStatus> current_status_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace nebula::diagnostics
