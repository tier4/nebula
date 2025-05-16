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

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <utility>

namespace nebula::ros
{

struct FrequencyDiagnosticTask : public diagnostic_updater::DiagnosticTask
{
  struct Params
  {
    double expected_freq_hz;
    double relative_tolerance;
  };

  FrequencyDiagnosticTask(
    const std::string & name, diagnostic_updater::Updater & updater, rclcpp::Clock::SharedPtr clock,
    const Params & params)
  : diagnostic_updater::DiagnosticTask(name),
    clock_(std::move(clock)),
    last_tick_(clock_->now()),
    params_(params)
  {
    updater.add(*this);
  }

  [[nodiscard]] double max_freq_hz() const
  {
    return params_.expected_freq_hz * (1.0 + params_.relative_tolerance);
  }

  [[nodiscard]] double min_freq_hz() const
  {
    return params_.expected_freq_hz * (1.0 - params_.relative_tolerance);
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    double max_acceptable_diff_s = 1.0 / min_freq_hz();
    double min_acceptable_diff_s = 1.0 / max_freq_hz();
    rclcpp::Time now = clock_->now();

    double last_frequency_hz = 1.0 / last_tick_diff_.seconds();

    stat.add("Frequency [Hz]", last_frequency_hz);
    stat.add("Min acceptable frequency [Hz]", min_freq_hz());
    stat.add("Max acceptable frequency [Hz]", max_freq_hz());

    stat.add("Time diff [s]", last_tick_diff_.seconds());
    stat.add("Min acceptable diff [s]", min_acceptable_diff_s);
    stat.add("Max acceptable diff [s]", max_acceptable_diff_s);

    stat.add("Last tick", last_tick_.seconds());

    if (now - last_tick_ > rclcpp::Duration::from_seconds(max_acceptable_diff_s)) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Frequency too low");
      return;
    }

    double frequency_hz = 1.0 / last_tick_diff_.seconds();
    if (frequency_hz < min_freq_hz()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Frequency too low");
      return;
    }

    if (frequency_hz > max_freq_hz()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Frequency too high");
      return;
    }

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Frequency is within tolerance");
  }

  void tick()
  {
    rclcpp::Time now = clock_->now();
    last_tick_diff_ = now - last_tick_;
    last_tick_ = now;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_tick_;
  rclcpp::Duration last_tick_diff_{0, 0};
  Params params_;
};

}  // namespace nebula::ros
