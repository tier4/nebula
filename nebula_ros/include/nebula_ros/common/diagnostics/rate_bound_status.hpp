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

#ifndef RATE_BOUND_STATUS_HPP_
#define RATE_BOUND_STATUS_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

namespace custom_diagnostic_tasks
{
/**
 * \brief A structure that holds the constructor parameters for the
 * RateBoundStatus class.
 */
struct RateBoundStatusParam
{
  RateBoundStatusParam(const double min_freq, const double max_freq)
  : min_frequency(min_freq), max_frequency(max_freq)
  {
  }

  double min_frequency;
  double max_frequency;
};

/**
 * \brief Diagnostic task to monitor the interval between events.
 *
 * This diagnostic task monitors the difference between consecutive events,
 * and creates corresponding diagnostics. This task categorize observed intervals into
 * OK/WARN/ERROR according to the value ranges passed via constructor arguments
 */
class RateBoundStatus : public diagnostic_updater::DiagnosticTask
{
private:
  // Helper struct to express state machine nodes
  struct StateBase
  {
    StateBase(const unsigned char lv, const std::string m) : level(lv), num_observations(1), msg(m)
    {
    }

    unsigned char level;
    size_t num_observations;
    std::string msg;
  };

  struct Stale : public StateBase
  {
    Stale()
    : StateBase(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Topic has not been received yet")
    {
    }
  };

  struct Ok : public StateBase
  {
    Ok() : StateBase(diagnostic_msgs::msg::DiagnosticStatus::OK, "Rate is reasonable") {}
  };

  struct Warn : public StateBase
  {
    Warn() : StateBase(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Rate is within warning range")
    {
    }
  };

  struct Error : public StateBase
  {
    Error() : StateBase(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Rate is out of valid range")
    {
    }
  };

  using StateHolder = std::variant<Stale, Ok, Warn, Error>;

public:
  /**
   * \brief Constructs RateBoundstatus, which inherits diagnostic_updater::DiagnosticTask.
   *
   * \param ok_params The pair of min/max frequency for the topic rate to be recognized as "OK".
   * \param warn_params The pair of min/max frequency for the topic rate to be recognized as "WARN".
   * These values should have a wider range than `ok_params`.
   * \param num_frame_transition The number of the successive observations for the status
   * transition. E.g., the status will not be changed from OK to WARN until successive
   * `num_frame_transition` WARNs are observed.
   * \param immediate_error_report If true (default), errors related to the rate bounds will be
   * reported immediately once observed; otherwise, the hysteresis damping method using
   * `num_frame_transition` will be adopted
   * \param name The arbitrary string to be assigned for this diagnostic task.
   * This name will not be exposed in the actual published topics.
   */
  RateBoundStatus(
    const RateBoundStatusParam & ok_params, const RateBoundStatusParam & warn_params,
    const size_t num_frame_transition = 1, const bool immediate_error_report = true,
    const std::string & name = "rate bound check")
  : DiagnosticTask(name),
    ok_params_(ok_params),
    warn_params_(warn_params),
    num_frame_transition_(num_frame_transition),
    immediate_error_report_(immediate_error_report),
    zero_seen_(false),
    candidate_state_(Stale{}),
    current_state_(Stale{})
  {
    if (num_frame_transition < 1) {
      num_frame_transition_ = 1;
    }

    // Confirm `warn_params` surely has wider range than `ok_params`
    if (
      warn_params_.min_frequency >= ok_params_.min_frequency ||
      ok_params_.max_frequency >= warn_params_.max_frequency) {
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
    double stamp =
      std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

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

  /**
   * \brief function called every update
   */
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    std::unique_lock<std::mutex> lock(lock_);

    // classify the current observation
    StateHolder frame_result;
    if (!frequency_ || zero_seen_) {
      frame_result.emplace<Stale>();
    } else {
      if (ok_params_.min_frequency < frequency_ && frequency_ < ok_params_.max_frequency) {
        frame_result.emplace<Ok>();
      } else if (
        (warn_params_.min_frequency <= frequency_ && frequency_ <= ok_params_.min_frequency) ||
        (ok_params_.max_frequency <= frequency_ && frequency_ <= warn_params_.max_frequency)) {
        frame_result.emplace<Warn>();
      } else {
        frame_result.emplace<Error>();
      }
    }

    // If the classify result is same as previous one, count the number of observation
    // Otherwise, update candidate
    if (candidate_state_.index() == frame_result.index()) {  // if result has the same status as
                                                             // candidate
      std::visit([](auto & s) { s.num_observations += 1; }, candidate_state_);
    } else {
      candidate_state_ = frame_result;
    }

    // Update the current state if
    // - immediate error report is required and the observed state is error
    // - Or the same state is observed multiple times
    if (
      (immediate_error_report_ && std::holds_alternative<Error>(candidate_state_)) ||
      (get_num_observations(candidate_state_) >= num_frame_transition_)) {
      current_state_ = candidate_state_;
      std::visit([](auto & s) { s.num_observations = 1; }, candidate_state_);
    }

    stat.summary(get_level(current_state_), get_msg(current_state_));

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << frequency_.value_or(0.0);
    stat.add("Publish rate", ss.str());

    ss.str("");  // reset contents
    ss << get_level_string(get_level(frame_result));
    stat.add("Rate status", ss.str());

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << ok_params_.min_frequency;
    stat.add("Minimum OK rate threshold", ss.str());

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << ok_params_.max_frequency;
    stat.add("Maximum OK rate threshold", ss.str());

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << warn_params_.min_frequency;
    stat.add("Minimum WARN rate threshold", ss.str());

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << warn_params_.max_frequency;
    stat.add("Maximum WARN rate threshold", ss.str());

    ss.str("");  // reset contents
    ss << get_num_observations(candidate_state_);
    stat.add("Observed frames", ss.str());

    ss.str("");  // reset contents
    ss << num_frame_transition_;
    stat.add("Observed frames transition threshold", ss.str());
  }

protected:
  RateBoundStatusParam ok_params_;
  RateBoundStatusParam warn_params_;
  size_t num_frame_transition_;
  bool immediate_error_report_;
  bool zero_seen_;
  std::optional<double> frequency_;
  std::optional<double> previous_frame_timestamp_;
  std::mutex lock_;

  StateHolder candidate_state_;
  StateHolder current_state_;

  static unsigned char get_level(const StateHolder & state)
  {
    return std::visit([](const auto & s) { return s.level; }, state);
  }

  static size_t get_num_observations(const StateHolder & state)
  {
    return std::visit([](const auto & s) { return s.num_observations; }, state);
  }

  static std::string get_msg(const StateHolder & state)
  {
    return std::visit([](const auto & s) { return s.msg; }, state);
  }

  static std::string get_level_string(unsigned char level)
  {
    switch (level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        return "OK";
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        return "WARN";
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        return "ERROR";
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
        return "STALE";
      default:
        return "UNDEFINED";
    }
  }
};  // class RateBoundStatus

}  // namespace custom_diagnostic_tasks

#endif  // RATE_BOUND_STATUS_HPP_
