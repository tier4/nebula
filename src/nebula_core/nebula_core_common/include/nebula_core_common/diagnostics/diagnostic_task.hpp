// Copyright 2026 TIER IV, Inc.
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

// VENDORED from diagnostic_updater, to keep rclcpp out of the non-ROS packages.
//
//   Upstream repo : https://github.com/ros/diagnostics
//   Upstream ref  : 4.0.6 (commit 08dc9bc)
//   Upstream files: diagnostic_updater/include/diagnostic_updater/
//     diagnostic_updater.hpp         L72-98   DiagnosticTask
//                                    L152-196 CompositeDiagnosticTask
//     diagnostic_status_wrapper.hpp  L90-94   summary()      -> set_summary()
//                                    L116-129 mergeSummary() -> merge_summary()
//                                    L222-230 add()          -> add_value()
//   Upstream license: BSD-3-Clause, Copyright (c) 2008, Willow Garage, Inc.
//
// Modifications:
//   1. tasks write into a plain diagnostic_msgs::msg::DiagnosticStatus, the wrapper's member
//      functions became free functions;
//   2. names follow Nebula's snake_case style, tasks are non-copyable;
//   3. CompositeDiagnosticTask takes children by reference.

#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace nebula::diagnostics
{

using diagnostic_msgs::msg::DiagnosticStatus;

/// @brief Set the level and message of `status`.
inline void set_summary(DiagnosticStatus & status, uint8_t level, std::string message)
{
  status.level = level;
  status.message = std::move(message);
}

/**
 * @brief Merge a level and message into the summary of `status`.
 *
 * Behaves exactly like `diagnostic_updater::DiagnosticStatusWrapper::mergeSummary`, so that
 * composite tasks report the same summaries as their `diagnostic_updater` counterparts: the
 * highest level wins, messages of non-OK levels are joined by "; ", and OK messages are dropped.
 */
inline void merge_summary(DiagnosticStatus & status, uint8_t level, const std::string & message)
{
  if ((level > 0) == (status.level > 0)) {
    if (!status.message.empty()) {
      status.message += "; ";
      status.message += message;
    }
  } else if (level > status.level) {
    status.message = message;
  }

  if (level > status.level) {
    status.level = level;
  }
}

/// @brief Append a key-value pair to `status`. The value is formatted via `operator<<`.
template <typename T>
void add_value(DiagnosticStatus & status, const std::string & key, const T & value)
{
  std::stringstream ss;
  ss << value;

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = ss.str();
  status.values.push_back(std::move(kv));
}

/**
 * @brief A named check that reports its current state as a `DiagnosticStatus`.
 *
 * Tasks do not know when or how often they are run and where their status goes. The owner decides
 * that, e.g. by registering the task with a `diagnostic_updater::Updater` in a ROS 2 node, or by
 * calling `run()` directly.
 */
class DiagnosticTask
{
public:
  explicit DiagnosticTask(std::string name) : name_(std::move(name)) {}
  DiagnosticTask(const DiagnosticTask &) = delete;
  DiagnosticTask & operator=(const DiagnosticTask &) = delete;
  DiagnosticTask(DiagnosticTask &&) = delete;
  DiagnosticTask & operator=(DiagnosticTask &&) = delete;
  virtual ~DiagnosticTask() = default;

  [[nodiscard]] const std::string & name() const { return name_; }

  /// @brief Write the current level, message and key-value pairs into `status`.
  virtual void run(DiagnosticStatus & status) = 0;

private:
  std::string name_;
};

/**
 * @brief Runs several tasks and reports them as one status.
 *
 * The key-value pairs of all children are concatenated and their summaries are merged via
 * `merge_summary`.
 */
class CompositeDiagnosticTask : public DiagnosticTask
{
public:
  using DiagnosticTask::DiagnosticTask;

  /// @brief Add a child. It has to outlive this task.
  void add_task(DiagnosticTask & task) { tasks_.push_back(&task); }

  void run(DiagnosticStatus & status) override
  {
    DiagnosticStatus combined;
    const uint8_t original_level = status.level;
    const std::string original_message = status.message;

    for (auto * task : tasks_) {
      set_summary(status, original_level, original_message);
      task->run(status);
      merge_summary(combined, status.level, status.message);
    }

    set_summary(status, combined.level, combined.message);
  }

private:
  std::vector<DiagnosticTask *> tasks_;
};

}  // namespace nebula::diagnostics
