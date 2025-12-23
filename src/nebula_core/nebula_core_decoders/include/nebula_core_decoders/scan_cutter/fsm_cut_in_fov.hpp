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

#include "nebula_core_decoders/scan_cutter/types.hpp"

#include <stdexcept>

namespace nebula::drivers
{

/// @brief FSM for scan cutting when the cut angle is within the field of view (360° FoV or
/// cut inside limited FoV). This is the simpler 4-state FSM.
///
/// States:
/// - F0: All channels in buffer 0 (Filled 0)
/// - C0_1: Channels split, transitioning from buffer 0 to buffer 1 (Crossing 0->1)
/// - F1: All channels in buffer 1 (Filled 1)
/// - C1_0: Channels split, transitioning from buffer 1 to buffer 0 (Crossing 1->0)
///
/// Transition table (From columns, To rows):
///
/// | To \ From | F0      | C0_1 | F1      | C1_0 |
/// | --------- | ------- | ---- | ------- | ---- |
/// | F0        | -       |      | T0, E1  | E1   |
/// | C0_1      | T1      | -    |         | ⛔   |
/// | F1        | T1, E0  | E0   | -       |      |
/// | C1_0      |         | ⛔   | T0      | -    |
class FsmCutInFov
{
public:
  using buffer_index_t = scan_cutter::buffer_index_t;
  using ChannelBufferState = scan_cutter::ChannelBufferState;
  using TransitionActions = scan_cutter::TransitionActions;

  template <typename T>
  using AllSame = scan_cutter::AllSame<T>;
  using Different = scan_cutter::Different;

  enum class State : uint8_t {
    F0,    // All channels in buffer 0
    C0_1,  // Crossing from buffer 0 to buffer 1
    F1,    // All channels in buffer 1
    C1_0   // Crossing from buffer 1 to buffer 0
  };

  /// @brief Determine the FSM state from the buffer state and current active buffer.
  /// @param buffer_state The current buffer state (AllSame or Different).
  /// @param current_buffer The current active buffer index.
  /// @return The FSM state.
  [[nodiscard]] static State determine_state(
    const ChannelBufferState & buffer_state, buffer_index_t current_buffer)
  {
    if (std::holds_alternative<AllSame<buffer_index_t>>(buffer_state)) {
      buffer_index_t buffer_index = std::get<AllSame<buffer_index_t>>(buffer_state).value;
      return buffer_index == 0 ? State::F0 : State::F1;
    }

    // Different - channels are split between buffers
    return current_buffer == 0 ? State::C0_1 : State::C1_0;
  }

  /// @brief Compute the transition actions based on state change.
  /// @param state_before The FSM state before the update.
  /// @param state_after The FSM state after the update.
  /// @return The transition actions to perform.
  [[nodiscard]] static TransitionActions get_transition_actions(
    State state_before, State state_after)
  {
    TransitionActions actions{std::nullopt, std::nullopt};

    // No transition - no action
    if (state_before == state_after) {
      return actions;
    }

    // Transition table implementation
    switch (state_before) {
      case State::F0:
        switch (state_after) {
          case State::C0_1:
            // F0 -> C0_1: T1 (reset timestamp of buffer 1)
            actions.reset_timestamp_buffer = 1;
            break;
          case State::F1:
            // F0 -> F1: T1, E0 (reset timestamp 1, emit buffer 0)
            actions.reset_timestamp_buffer = 1;
            actions.emit_scan_buffer = 0;
            break;
          default:
            // F0 -> F0: no action (handled above)
            // F0 -> C1_0: invalid (empty cell in table)
            break;
        }
        break;

      case State::C0_1:
        switch (state_after) {
          case State::F1:
            // C0_1 -> F1: E0 (emit buffer 0)
            actions.emit_scan_buffer = 0;
            break;
          case State::C1_0:
            // C0_1 -> C1_0: ⛔ Invalid transition
            throw std::runtime_error("Invalid FSM transition: C0_1 -> C1_0");
          default:
            // C0_1 -> F0: invalid (empty cell in table)
            // C0_1 -> C0_1: no action (handled above)
            break;
        }
        break;

      case State::F1:
        switch (state_after) {
          case State::C1_0:
            // F1 -> C1_0: T0 (reset timestamp of buffer 0)
            actions.reset_timestamp_buffer = 0;
            break;
          case State::F0:
            // F1 -> F0: T0, E1 (reset timestamp 0, emit buffer 1)
            actions.reset_timestamp_buffer = 0;
            actions.emit_scan_buffer = 1;
            break;
          default:
            // F1 -> F1: no action (handled above)
            // F1 -> C0_1: invalid (empty cell in table)
            break;
        }
        break;

      case State::C1_0:
        switch (state_after) {
          case State::F0:
            // C1_0 -> F0: E1 (emit buffer 1)
            actions.emit_scan_buffer = 1;
            break;
          case State::C0_1:
            // C1_0 -> C0_1: ⛔ Invalid transition
            throw std::runtime_error("Invalid FSM transition: C1_0 -> C0_1");
          default:
            // C1_0 -> F1: invalid (empty cell in table)
            // C1_0 -> C1_0: no action (handled above)
            break;
        }
        break;
    }

    return actions;
  }

  /// @brief Step the FSM and return the actions to perform.
  /// @param buffer_state_before Buffer state before the azimuth update.
  /// @param buffer_state_after Buffer state after the azimuth update.
  /// @param current_buffer The current active buffer index.
  /// @return The transition actions to perform.
  [[nodiscard]] static TransitionActions step(
    const ChannelBufferState & buffer_state_before, const ChannelBufferState & buffer_state_after,
    buffer_index_t current_buffer)
  {
    State state_before = determine_state(buffer_state_before, current_buffer);
    State state_after = determine_state(buffer_state_after, current_buffer);
    return get_transition_actions(state_before, state_after);
  }
};

}  // namespace nebula::drivers
