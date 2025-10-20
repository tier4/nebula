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

#ifndef RATE_BOUND_STATUS_HPP_
#define RATE_BOUND_STATUS_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_ros/common/diagnostics/hysteresis_state_machine.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <rcl/time.h>

#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace custom_diagnostic_tasks
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
 * \brief Diagnostic task to monitor the interval between events.
 *
 * This diagnostic task monitors the difference between consecutive events,
 * and creates corresponding diagnostics. This task categorize observed intervals into
 * OK/WARN/ERROR according to the value ranges passed via constructor arguments
 */
class RateBoundStatus : public diagnostic_updater::DiagnosticTask
{
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
   * \param immediate_error_report If true, errors related to the rate bounds will be
   * reported immediately once observed; otherwise, the hysteresis damping method using
   * `num_frame_transition` will be adopted
   * \param name The arbitrary string to be assigned for this diagnostic task.
   * This name will not be exposed in the actual published topics.
   */
  RateBoundStatus(
    const rclcpp::Node * parent_node, const RateBoundStatusParam & ok_params,
    const RateBoundStatusParam & warn_params, const size_t num_frame_transition = 1,
    const bool immediate_error_report = false, const bool immediate_relax_state = true,
    const std::string & name = "rate bound check")
  : DiagnosticTask(name),
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

    // select clock according to the use_sim_time parameter set to the parent
    bool use_sim_time = false;
    if (parent_node->has_parameter("use_sim_time")) {
      use_sim_time = parent_node->get_parameter("use_sim_time").as_bool();
    }
    if (use_sim_time) {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    } else {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
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
   * \brief function called every update
   */
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    std::unique_lock<std::mutex> lock(lock_);

    // classify the current observation
    DiagnosticStatus_t frame_result{};
    if (!frequency_ || zero_seen_) {
      frame_result = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else {
      if (is_ok(frequency_.value())) {
        frame_result = diagnostic_msgs::msg::DiagnosticStatus::OK;
      } else if (is_warn(frequency_.value())) {
        frame_result = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      } else {
        frame_result = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
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
        frame_result = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
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
      hysteresis_state_machine_.set_state_level(diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }

    auto current_state = hysteresis_state_machine_.get_state_level();
    stat.summary(current_state, generate_msg(current_state));

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << frequency_.value_or(0.0);
    stat.add("Publish rate", ss.str());

    ss.str("");  // reset contents
    ss << get_level_string(current_state);
    stat.add("Effective rate status", ss.str());

    ss.str("");  // reset contents
    ss << get_level_string(hysteresis_state_machine_.get_candidate_level());
    stat.add("Candidate rate status", ss.str());

    ss.str("");  // reset contents
    ss << hysteresis_state_machine_.get_candidate_num_observation();
    stat.add("Candidate status observed frames", ss.str());

    ss.str("");  // reset contents
    ss << num_frame_skipped;
    stat.add("Assumed skipped frames", ss.str());

    ss.str("");  // reset contents
    ss << num_frame_transition_;
    stat.add("Observed frames transition threshold", ss.str());

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << ok_params_.min_frequency;
    stat.add("Minimum OK rate threshold", ss.str());

    if (ok_params_.max_frequency) {
      ss.str("");  // reset contents
      ss << std::fixed << std::setprecision(2) << ok_params_.max_frequency.value();
      stat.add("Maximum OK rate threshold", ss.str());
    }

    ss.str("");  // reset contents
    ss << std::fixed << std::setprecision(2) << warn_params_.min_frequency;
    stat.add("Minimum WARN rate threshold", ss.str());

    if (warn_params_.max_frequency) {
      ss.str("");  // reset contents
      ss << std::fixed << std::setprecision(2) << warn_params_.max_frequency.value();
      stat.add("Maximum WARN rate threshold", ss.str());
    }
  }

protected:
  RateBoundStatusParam ok_params_;
  RateBoundStatusParam warn_params_;
  size_t num_frame_transition_;
  bool zero_seen_;
  std::optional<double> frequency_;
  std::optional<double> previous_frame_timestamp_;
  std::mutex lock_;

  HysteresisStateMachine hysteresis_state_machine_;

  std::shared_ptr<rclcpp::Clock> clock_;

  inline double get_now() { return clock_->now().seconds(); }

  static std::string generate_msg(const DiagnosticStatus_t & state)
  {
    std::string ret;
    switch (state) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        ret = "Rate is reasonable";
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        ret = "Rate is within warning range";
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        ret = "Rate is out of valid range";
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
        ret = "Topic has not been received yet";
        break;
      default:
        ret = "Undefined state";
        break;
    }
    return ret;
  }
};  // class RateBoundStatus

}  // namespace custom_diagnostic_tasks

#endif  // RATE_BOUND_STATUS_HPP_
