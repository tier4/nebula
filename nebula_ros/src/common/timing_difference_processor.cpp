// Copyright 2024 TIER IV, Inc.
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

#include "nebula_ros/common/timing_difference_processor.hpp"

#include <chrono>
#include <memory>
#include <utility>

namespace nebula::ros
{

TimingDifferenceProcessor::TimingDifferenceProcessor(
  std::shared_ptr<SyncDiagClient> sync_diag_client, const rclcpp::Logger & logger,
  std::chrono::milliseconds rate_limit)
: sync_diag_client_(std::move(sync_diag_client)),
  logger_(logger),
  rate_limit_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(rate_limit).count())
{
}

void TimingDifferenceProcessor::process_timing_difference(
  uint64_t reception_timestamp_ns, uint64_t packet_timestamp_ns)
{
  if (!sync_diag_client_) {
    return;
  }

  if (reception_timestamp_ns - last_measurement_send_time_ns_ < rate_limit_ns_) {
    return;
  }

  last_measurement_send_time_ns_ = reception_timestamp_ns;

  try {
    int64_t diff_ns =
      static_cast<int64_t>(reception_timestamp_ns) - static_cast<int64_t>(packet_timestamp_ns);
    sync_diag_client_->submit_clock_diff_measurement(diff_ns);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Could not send measurement:" << e.what());
  }
}

}  // namespace nebula::ros
