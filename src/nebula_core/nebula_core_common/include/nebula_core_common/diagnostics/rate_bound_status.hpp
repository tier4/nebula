// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Copied from https://github.com/tier4/ros2_v4l2_camera/pull/29
// Patched with https://github.com/tier4/ros2_v4l2_camera/pull/30
// Patched with https://github.com/tier4/ros2_v4l2_camera/pull/37

#ifndef NEBULA_CORE_COMMON__DIAGNOSTICS__RATE_BOUND_STATUS_HPP_
#define NEBULA_CORE_COMMON__DIAGNOSTICS__RATE_BOUND_STATUS_HPP_

#include "nebula_core_common/diagnostics/hysteresis_state_machine.hpp"
#include "nebula_core_common/diagnostics/level.hpp"
#include "nebula_core_common/util/clock.hpp"

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace nebula::diagnostics
{
/**
 * \brief A structure that holds the constructor parameters for the
 * RateBoundStatus class.
 */
struct RateBoundStatusParam
{
  explicit RateBoundStatusParam(
    const double min_freq, const std::optional<double> max_freq = std::nullopt)
  : min_frequency(min_freq), max_frequency(max_freq)
  {
  }

  double min_frequency;
  std::optional<double> max_frequency;
};

/**
 * \brief Monitors the interval between events.
 *
 * This monitors the difference between consecutive events, and categorizes observed intervals
 * into OK/WARN/ERROR according to the value ranges passed via constructor arguments
 */
class RateBoundStatus
{
public:
  struct Report
  {
    /// The level after hysteresis
    Level level{};
    /// The rate of the last two events, or of the time since the last event if events stopped.
    /// Unset if less than two events have been observed.
    std::optional<double> rate_hz;
    /// The level that the current observations would lead to
    Level candidate_level{};
    /// The number of successive observations of `candidate_level`
    size_t candidate_num_observations{};
    /// The minimum number of events that were missed since the last event
    size_t assumed_skipped_frames{};
    size_t num_frame_transition{};
    RateBoundStatusParam ok_params{0};
    RateBoundStatusParam warn_params{0};
    bool immediate_error_report{};
    bool immediate_relax_state{};
  };

  /**
   * \brief Constructs RateBoundStatus.
   *
   * \param clock The clock that the interval between events is measured on.
   * \param ok_params The pair of min/max frequency for the topic rate to be recognized as "OK".
   * \param warn_params The pair of min/max frequency for the topic rate to be recognized as "WARN".
   * These values should have a wider range than `ok_params`.
   * \param num_frame_transition The number of the successive observations for the status
   * transition. E.g., the status will not be changed from OK to WARN until successive
   * `num_frame_transition` WARNs are observed.
   * \param immediate_error_report If true, errors related to the rate bounds will be
   * reported immediately once observed; otherwise, the hysteresis damping method using
   * `num_frame_transition` will be adopted
   */
  RateBoundStatus(
    std::shared_ptr<const util::Clock> clock, const RateBoundStatusParam & ok_params,
    const RateBoundStatusParam & warn_params, const size_t num_frame_transition = 1,
    const bool immediate_error_report = false, const bool immediate_relax_state = true)
  : clock_(std::move(clock)),
    ok_params_(ok_params),
    warn_params_(warn_params),
    num_frame_transition_(num_frame_transition),
    zero_seen_(false),
    hysteresis_state_machine_(num_frame_transition, immediate_error_report, immediate_relax_state)
  {
    if (num_frame_transition < 1) {
      num_frame_transition_ = 1;
    }

    // Confirm `warn_params` surely has wider range than `ok_params`
    if (warn_params_.min_frequency >= ok_params_.min_frequency) {
      throw std::runtime_error(
        "Invalid range parameters were detected. warn_params should specify a range "
        "that includes a range of ok_params.");
    }
  }

  /**
   * \brief Calculate the frequency of how much this function is called
   *
   */
  void tick()
  {
    std::unique_lock<std::mutex> lock(lock_);
    double stamp = get_now();

    if (!previous_frame_timestamp_) {
      zero_seen_ = true;
    } else {
      zero_seen_ = false;
      double delta = stamp - previous_frame_timestamp_.value();
      frequency_ = (delta < 10 * std::numeric_limits<double>::epsilon())
                     ? std::numeric_limits<double>::infinity()
                     : 1. / delta;
    }
    previous_frame_timestamp_ = stamp;
  }

  bool is_ok(double observation)
  {
    bool result = ok_params_.min_frequency < observation;
    if (ok_params_.max_frequency) {
      // If the max_frequency is defined, consider the upper bound
      result = result && (observation < ok_params_.max_frequency);
    }
    return result;
  }

  bool is_warn(double observation)
  {
    bool result =
      (warn_params_.min_frequency <= observation && observation <= ok_params_.min_frequency);
    if (ok_params_.max_frequency && warn_params_.max_frequency) {
      // If the max_frequency is defined, consider the upper bound
      result = result || (ok_params_.max_frequency <= observation &&
                          observation <= warn_params_.max_frequency);
    }
    return result;
  }

  /**
   * \brief Classify the current rate and advance the hysteresis by one observation
   */
  [[nodiscard]] Report report()
  {
    std::unique_lock<std::mutex> lock(lock_);

    // classify the current observation
    Level frame_result{};
    if (!frequency_ || zero_seen_) {
      frame_result = Level::STALE;
    } else {
      if (is_ok(frequency_.value())) {
        frame_result = Level::OK;
      } else if (is_warn(frequency_.value())) {
        frame_result = Level::WARN;
      } else {
        frame_result = Level::ERROR;
      }
    }

    // check the latest update is valid one
    size_t num_frame_skipped = 0;
    bool is_valid_observation = true;
    if (previous_frame_timestamp_) {
      double stamp = get_now();
      double delta = stamp - previous_frame_timestamp_.value();
      double freq_from_prev_tick = 1. / delta;
      // If the latest update too older than warn_params_ criteria, frame_result fires error
      if (freq_from_prev_tick < warn_params_.min_frequency) {
        frame_result = Level::ERROR;
        is_valid_observation = false;
        frequency_ = freq_from_prev_tick;
        auto max_frame_period_s = 1. / warn_params_.min_frequency;
        // Minimum frames to assume skipped if 'tick' calls occur at 'warn_params_.min_frequency'.
        num_frame_skipped = static_cast<size_t>(delta / max_frame_period_s);
      }
    }

    // Update state using hysteresis
    hysteresis_state_machine_.update_state(frame_result);
    if (!is_valid_observation && num_frame_skipped >= num_frame_transition_) {
      hysteresis_state_machine_.set_current_state_level(Level::ERROR);
    }

    Report report;
    report.level = hysteresis_state_machine_.get_current_state_level();
    report.rate_hz = frequency_;
    report.candidate_level = hysteresis_state_machine_.get_candidate_level();
    report.candidate_num_observations = hysteresis_state_machine_.get_candidate_num_observation();
    report.assumed_skipped_frames = num_frame_skipped;
    report.num_frame_transition = hysteresis_state_machine_.get_num_frame_transition();
    report.ok_params = ok_params_;
    report.warn_params = warn_params_;
    report.immediate_error_report = hysteresis_state_machine_.get_immediate_error_report_param();
    report.immediate_relax_state = hysteresis_state_machine_.get_immediate_relax_state_param();
    return report;
  }

protected:
  std::shared_ptr<const util::Clock> clock_;
  RateBoundStatusParam ok_params_;
  RateBoundStatusParam warn_params_;
  size_t num_frame_transition_;
  bool zero_seen_;
  std::optional<double> frequency_;
  std::optional<double> previous_frame_timestamp_;
  std::mutex lock_;

  HysteresisStateMachine hysteresis_state_machine_;

  double get_now() { return std::chrono::duration<double>(clock_->now()).count(); }
};  // class RateBoundStatus

}  // namespace nebula::diagnostics

#endif  // NEBULA_CORE_COMMON__DIAGNOSTICS__RATE_BOUND_STATUS_HPP_
