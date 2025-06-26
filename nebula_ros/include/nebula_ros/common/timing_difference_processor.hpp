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

#include "nebula_ros/common/sync_diag_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

namespace nebula::ros
{

/// @brief Processes timing differences between packet reception and packet payload timestamps
/// with configurable rate limiting
class TimingDifferenceProcessor
{
public:
  /// @brief Constructor
  /// @param sync_diag_client Shared pointer to the sync diagnostic client for submitting
  /// measurements
  /// @param logger Shared pointer to the logger for error reporting
  /// @param rate_limit Rate limiting interval (default: 100ms)
  explicit TimingDifferenceProcessor(
    std::shared_ptr<SyncDiagClient> sync_diag_client, const rclcpp::Logger & logger,
    std::chrono::milliseconds rate_limit = std::chrono::milliseconds(100));

  /// @brief Process timing difference between packet reception and packet payload
  /// @param reception_timestamp_ns UDP socket reception timestamp in nanoseconds
  /// @param packet_timestamp_ns Packet payload timestamp in nanoseconds
  void process_timing_difference(uint64_t reception_timestamp_ns, uint64_t packet_timestamp_ns);

private:
  std::shared_ptr<SyncDiagClient> sync_diag_client_;
  rclcpp::Logger logger_;
  uint32_t rate_limit_ns_;
  uint64_t last_measurement_send_time_ns_{0};
};

}  // namespace nebula::ros
