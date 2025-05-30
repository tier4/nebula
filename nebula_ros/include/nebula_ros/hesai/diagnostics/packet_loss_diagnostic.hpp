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
#include <nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <utility>
namespace nebula::ros
{

class PacketLossDiagnosticTask : public diagnostic_updater::DiagnosticTask
{
public:
  explicit PacketLossDiagnosticTask(uint64_t error_threshold, rclcpp::Clock::SharedPtr clock)
  : DiagnosticTask("Packet loss status"),
    clock_(std::move(clock)),
    last_run_time_(clock_->now()),
    error_threshold_(error_threshold)
  {
  }

  void on_lost(uint64_t n_lost) { n_lost_packets_ += n_lost; }

  void run(diagnostic_updater::DiagnosticStatusWrapper & status) override
  {
    const auto now = clock_->now();
    const auto dt = now - last_run_time_;
    last_run_time_ = now;

    status.add("Lost packets", n_lost_packets_);
    status.add("Error threshold", error_threshold_);
    status.add("Time since last update [s]", std::to_string(dt.seconds()));

    if (n_lost_packets_ == 0) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No packet loss");
    } else if (n_lost_packets_ < error_threshold_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Slight packet loss");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Severe packet loss");
    }

    n_lost_packets_ = 0;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  uint64_t n_lost_packets_{};
  rclcpp::Time last_run_time_;
  uint64_t error_threshold_;
};

}  // namespace nebula::ros
