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

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>

namespace nebula::ros
{

class SeverityLatch : public diagnostic_updater::DiagnosticTask
{
public:
  explicit SeverityLatch(const std::string & name) : diagnostic_updater::DiagnosticTask(name) {}

  void run(diagnostic_updater::DiagnosticStatusWrapper & status) override
  {
    if (!current_status_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No status available");
      return;
    }

    status.values.insert(
      status.values.end(), current_status_->values.begin(), current_status_->values.end());
    status.summary(current_status_->level, current_status_->message);
    current_status_ = std::nullopt;
  }

  void submit(diagnostic_msgs::msg::DiagnosticStatus status)
  {
    if (!current_status_) {
      current_status_ = status;
      return;
    }

    // Only the highest severity status in between two `run()`s is reported.
    // If multiple statuses with the same severity are submitted, only the last one is reported.
    if (status.level >= current_status_->level) {
      current_status_ = status;
    }
  }

private:
  std::optional<diagnostic_msgs::msg::DiagnosticStatus> current_status_;
};

}  // namespace nebula::ros
