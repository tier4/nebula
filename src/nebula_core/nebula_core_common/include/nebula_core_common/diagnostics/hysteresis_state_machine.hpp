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

// Copied from https://github.com/tier4/ros2_v4l2_camera/pull/37

#ifndef NEBULA_CORE_COMMON__DIAGNOSTICS__HYSTERESIS_STATE_MACHINE_HPP_
#define NEBULA_CORE_COMMON__DIAGNOSTICS__HYSTERESIS_STATE_MACHINE_HPP_

#include "nebula_core_common/diagnostics/level.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <variant>

namespace nebula::diagnostics
{
// Helper struct to express state machine nodes
struct StateBase
{
  explicit StateBase(const Level lv) : level(lv), num_observations(1) {}

  Level level;
  size_t num_observations;
};

struct Stale : public StateBase
{
  Stale() : StateBase(Level::STALE) {}
};

struct Ok : public StateBase
{
  Ok() : StateBase(Level::OK) {}
};

struct Warn : public StateBase
{
  Warn() : StateBase(Level::WARN) {}
};

struct Error : public StateBase
{
  Error() : StateBase(Level::ERROR) {}
};

using StateHolder = std::variant<Stale, Ok, Warn, Error>;

inline StateHolder generate_state(const Level & state)
{
  switch (state) {
    case Level::STALE:
      return Stale{};
    case Level::OK:
      return Ok{};
    case Level::WARN:
      return Warn{};
    case Level::ERROR:
      return Error{};
    default:
      throw std::runtime_error("Undefined status");
  }
}

inline Level get_level(const StateHolder & state)
{
  return std::visit([](const auto & s) { return s.level; }, state);
}

inline size_t get_num_observations(const StateHolder & state)
{
  return std::visit([](const auto & s) { return s.num_observations; }, state);
}

class HysteresisStateMachine
{
public:
  /**
   * \brief Constructs HysteresisStateMachine, which implements a smoothing
            filter over observation
   *
   * \param num_frame_transition The number of the successive observations for
            the status transition. E.g., the status will not be changed from OK
to WARN until successive `num_frame_transition` WARNs are observed.
   * \param immediate_error_report If true, errors will be reported immediately
            once observed; otherwise, the hysteresis damping method using
            `num_frame_transition` will be adopted
   * \param immediate_relax_state if true, reported state will immediately
            change if better state that the current one is observed
   */
  explicit HysteresisStateMachine(
    const size_t num_frame_transition = 1, const bool immediate_error_report = false,
    const bool immediate_relax_state = true)
  : num_frame_transition_(num_frame_transition),
    immediate_error_report_(immediate_error_report),
    immediate_relax_state_(immediate_relax_state),
    current_state_(Stale{})
  {
    if (num_frame_transition < 1) {
      num_frame_transition_ = 1;
    }
  }

  /**
   * \brief update internal state and returns the filtered state
   */
  void update_state(const Level & observation)
  {
    // If the classify result is same as previous one and the observation is
    // different from the current one, increment the number of observation
    // Otherwise, update candidate
    auto candidate_level = get_level(candidate_state_);
    auto current_level = get_level(current_state_);
    if (candidate_level == observation && candidate_level != current_level) {
      std::visit([](auto & s) { s.num_observations += 1; }, candidate_state_);
    } else {
      candidate_state_ = generate_state(observation);
    }

    // Update the current state if
    // - immediate error report is required and the observed state is error
    // - Or the same state is observed multiple times
    // - Or the observed state has lower level than the current one (i.e., the state is improved)
    bool is_immediate_error =
      (immediate_error_report_ && std::holds_alternative<Error>(candidate_state_));
    bool observed_over_threshold =
      (get_num_observations(candidate_state_) >= num_frame_transition_);
    bool is_immediate_relax =
      (immediate_relax_state_ && get_level(candidate_state_) < current_level);

    Level updated_level = current_level;
    if (is_immediate_error || observed_over_threshold || is_immediate_relax) {
      updated_level = get_level(candidate_state_);
    }

    current_state_ = generate_state(updated_level);
  }

  Level get_candidate_level() { return get_level(candidate_state_); }

  size_t get_candidate_num_observation() { return get_num_observations(candidate_state_); }

  size_t get_num_frame_transition() { return num_frame_transition_; }

  Level get_current_state_level() { return get_level(current_state_); }

  void set_current_state_level(const Level & state) { current_state_ = generate_state(state); }

  bool get_immediate_error_report_param() { return immediate_error_report_; }

  bool get_immediate_relax_state_param() { return immediate_relax_state_; }

protected:
  size_t num_frame_transition_;
  bool immediate_error_report_;
  bool immediate_relax_state_;
  StateHolder candidate_state_;
  StateHolder current_state_;
};  // class HysteresisStateMachine

}  // namespace nebula::diagnostics

#endif  // NEBULA_CORE_COMMON__DIAGNOSTICS__HYSTERESIS_STATE_MACHINE_HPP_
