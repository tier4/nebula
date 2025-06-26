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

#include "nebula_ros/common/sync_tooling/sync_tooling_worker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <utility>

namespace nebula::ros
{

/// @brief Plugin for submitting time difference measurements to the sync tooling worker
class TimeDifferencePlugin
{
public:
  /// @brief Constructor
  /// @param sync_tooling_worker Shared pointer to the sync tooling worker for submitting
  /// measurements
  /// @param logger Shared pointer to the logger for error reporting
  explicit TimeDifferencePlugin(
    std::shared_ptr<SyncToolingWorker> sync_tooling_worker, const rclcpp::Logger & logger,
    std::chrono::milliseconds rate_limit = std::chrono::milliseconds(100))
  : sync_tooling_worker_(std::move(sync_tooling_worker)),
    logger_(logger),
    rate_limit_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(rate_limit).count())
  {
  }

  /// @brief Process timing difference between packet reception and packet payload
  /// @param reception_timestamp_ns UDP socket reception timestamp in nanoseconds
  /// @param packet_timestamp_ns Packet payload timestamp in nanoseconds
  void submit_time_difference(uint64_t reception_timestamp_ns, uint64_t packet_timestamp_ns)
  {
    if (!sync_tooling_worker_) {
      return;
    }

    if (reception_timestamp_ns - last_measurement_send_time_ns_ < rate_limit_ns_) {
      return;
    }

    last_measurement_send_time_ns_ = reception_timestamp_ns;

    try {
      int64_t diff_ns =
        static_cast<int64_t>(reception_timestamp_ns) - static_cast<int64_t>(packet_timestamp_ns);
      sync_tooling_worker_->submit_clock_diff_measurement(diff_ns);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Could not send measurement:" << e.what());
    }
  }

private:
  std::shared_ptr<SyncToolingWorker> sync_tooling_worker_;
  rclcpp::Logger logger_;
  uint32_t rate_limit_ns_;
  uint64_t last_measurement_send_time_ns_{0};
};

}  // namespace nebula::ros
