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

/// @brief FSM for scan cutting when the cut angle is at the FoV end (limited FoV mode).
/// This is the more complex 8-state FSM that tracks both buffer and FoV state.
///
/// States:
/// - O0: All channels in buffer 0, all outside FoV (Outside 0)
/// - S0: All channels in buffer 0, some in FoV, some outside (Spanning 0)
/// - F0: All channels in buffer 0, all in FoV (Filled 0)
/// - C0_1: Channels split between buffers, transitioning 0->1 (Crossing 0->1)
/// - O1: All channels in buffer 1, all outside FoV (Outside 1)
/// - S1: All channels in buffer 1, some in FoV, some outside (Spanning 1)
/// - F1: All channels in buffer 1, all in FoV (Filled 1)
/// - C1_0: Channels split between buffers, transitioning 1->0 (Crossing 1->0)
///
/// Transition table for buffer 0 states (From columns, To rows):
///
/// | To \ From | O0      | S0      | F0      | C0_1    |
/// | --------- | ------- | ------- | ------- | ------- |
/// | O0        | -       |         |         |         |
/// | S0        | T0      | -       |         |         |
/// | F0        | T0      | -       | -       |         |
/// | C0_1      | T0      | -       | -       | -       |
/// | O1        | T1, E0  | E0      | E0      | E0      |
/// | S1        |         | T1, E0  | T1, E0  | T1, E0  |
/// | F1        |         |         | T1, E0  | T1, E0  |
/// | C1_0      |         |         |         | ⛔      |
///
/// Analogous transitions exist for buffer 1 states (symmetric).
class FsmCutAtFovEnd
{
public:
  using buffer_index_t = scan_cutter::buffer_index_t;
  using ChannelBufferState = scan_cutter::ChannelBufferState;
  using ChannelFovState = scan_cutter::ChannelFovState;
  using TransitionActions = scan_cutter::TransitionActions;

  template <typename T>
  using AllSame = scan_cutter::AllSame<T>;
  using Different = scan_cutter::Different;

  enum class State : uint8_t {
    O0,    // All channels in buffer 0, all outside FoV
    S0,    // All channels in buffer 0, spanning FoV boundary
    F0,    // All channels in buffer 0, all in FoV
    C0_1,  // Crossing from buffer 0 to buffer 1
    O1,    // All channels in buffer 1, all outside FoV
    S1,    // All channels in buffer 1, spanning FoV boundary
    F1,    // All channels in buffer 1, all in FoV
    C1_0   // Crossing from buffer 1 to buffer 0
  };

  /// @brief Determine the FSM state from buffer state, FoV state, and current active buffer.
  /// @param buffer_state The current buffer state (AllSame or Different).
  /// @param fov_state The current FoV state (AllSame or Different).
  /// @param current_buffer The current active buffer index.
  /// @return The FSM state.
  [[nodiscard]] static State determine_state(
    const ChannelBufferState & buffer_state, const ChannelFovState & fov_state,
    buffer_index_t current_buffer)
  {
    // Check if channels are split between buffers (Crossing state)
    if (std::holds_alternative<Different>(buffer_state)) {
      return current_buffer == 0 ? State::C0_1 : State::C1_0;
    }

    // All channels are in the same buffer
    buffer_index_t buffer_index = std::get<AllSame<buffer_index_t>>(buffer_state).value;

    // Determine FoV-based state
    if (std::holds_alternative<Different>(fov_state)) {
      // Spanning FoV boundary
      return buffer_index == 0 ? State::S0 : State::S1;
    }

    bool all_in_fov = std::get<AllSame<bool>>(fov_state).value;
    if (all_in_fov) {
      // All in FoV
      return buffer_index == 0 ? State::F0 : State::F1;
    }
    // All outside FoV
    return buffer_index == 0 ? State::O0 : State::O1;
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

    // Transition table implementation for buffer 0 -> buffer 1 transitions
    switch (state_before) {
      case State::O0:
        switch (state_after) {
          case State::S0:
          case State::F0:
          case State::C0_1:
            // O0 -> S0/F0/C0_1: T0 (reset timestamp of buffer 0)
            actions.reset_timestamp_buffer = 0;
            break;
          case State::O1:
            // O0 -> O1: T1, E0 (reset timestamp 1, emit buffer 0)
            actions.reset_timestamp_buffer = 1;
            actions.emit_scan_buffer = 0;
            break;
          default:
            // O0 -> S1/F1/C1_0: invalid (empty cells)
            break;
        }
        break;

      case State::S0:
        switch (state_after) {
          case State::O1:
            // S0 -> O1: E0 (emit buffer 0)
            actions.emit_scan_buffer = 0;
            break;
          case State::S1:
            // S0 -> S1: T1, E0 (reset timestamp 1, emit buffer 0)
            actions.reset_timestamp_buffer = 1;
            actions.emit_scan_buffer = 0;
            break;
          default:
            // S0 -> O0/F0/C0_1/F1/C1_0: invalid or no action
            break;
        }
        break;

      case State::F0:
        switch (state_after) {
          case State::O1:
            // F0 -> O1: E0 (emit buffer 0)
            actions.emit_scan_buffer = 0;
            break;
          case State::S1:
          // F0 -> S1: T1, E0 (reset timestamp 1, emit buffer 0)
          case State::F1:
            // F0 -> F1: T1, E0 (reset timestamp 1, emit buffer 0)
            actions.reset_timestamp_buffer = 1;
            actions.emit_scan_buffer = 0;
            break;
          default:
            // F0 -> O0/S0/C0_1/C1_0: invalid or no action
            break;
        }
        break;

      case State::C0_1:
        switch (state_after) {
          case State::O1:
            // C0_1 -> O1: E0 (emit buffer 0)
            actions.emit_scan_buffer = 0;
            break;
          case State::S1:
          // C0_1 -> S1: T1, E0 (reset timestamp 1, emit buffer 0)
          case State::F1:
            // C0_1 -> F1: T1, E0 (reset timestamp 1, emit buffer 0)
            actions.reset_timestamp_buffer = 1;
            actions.emit_scan_buffer = 0;
            break;
          case State::C1_0:
            // C0_1 -> C1_0: ⛔ Invalid transition
            throw std::runtime_error("Invalid FSM transition: C0_1 -> C1_0");
          default:
            // C0_1 -> O0/S0/F0: invalid
            break;
        }
        break;

      // Symmetric transitions for buffer 1 states
      case State::O1:
        switch (state_after) {
          case State::S1:
          case State::F1:
          case State::C1_0:
            // O1 -> S1/F1/C1_0: T1 (reset timestamp of buffer 1)
            actions.reset_timestamp_buffer = 1;
            break;
          case State::O0:
            // O1 -> O0: T0, E1 (reset timestamp 0, emit buffer 1)
            actions.reset_timestamp_buffer = 0;
            actions.emit_scan_buffer = 1;
            break;
          default:
            // O1 -> S0/F0/C0_1: invalid (empty cells)
            break;
        }
        break;

      case State::S1:
        switch (state_after) {
          case State::O0:
            // S1 -> O0: E1 (emit buffer 1)
            actions.emit_scan_buffer = 1;
            break;
          case State::S0:
            // S1 -> S0: T0, E1 (reset timestamp 0, emit buffer 1)
            actions.reset_timestamp_buffer = 0;
            actions.emit_scan_buffer = 1;
            break;
          default:
            // S1 -> O1/F1/C1_0/F0/C0_1: invalid or no action
            break;
        }
        break;

      case State::F1:
        switch (state_after) {
          case State::O0:
            // F1 -> O0: E1 (emit buffer 1)
            actions.emit_scan_buffer = 1;
            break;
          case State::S0:
          // F1 -> S0: T0, E1 (reset timestamp 0, emit buffer 1)
          case State::F0:
            // F1 -> F0: T0, E1 (reset timestamp 0, emit buffer 1)
            actions.reset_timestamp_buffer = 0;
            actions.emit_scan_buffer = 1;
            break;
          default:
            // F1 -> O1/S1/C1_0/C0_1: invalid or no action
            break;
        }
        break;

      case State::C1_0:
        switch (state_after) {
          case State::O0:
            // C1_0 -> O0: E1 (emit buffer 1)
            actions.emit_scan_buffer = 1;
            break;
          case State::S0:
          // C1_0 -> S0: T0, E1 (reset timestamp 0, emit buffer 1)
          case State::F0:
            // C1_0 -> F0: T0, E1 (reset timestamp 0, emit buffer 1)
            actions.reset_timestamp_buffer = 0;
            actions.emit_scan_buffer = 1;
            break;
          case State::C0_1:
            // C1_0 -> C0_1: ⛔ Invalid transition
            throw std::runtime_error("Invalid FSM transition: C1_0 -> C0_1");
          default:
            // C1_0 -> O1/S1/F1: invalid
            break;
        }
        break;
    }

    return actions;
  }

  /// @brief Step the FSM and return the actions to perform.
  /// @param buffer_state_before Buffer state before the azimuth update.
  /// @param buffer_state_after Buffer state after the azimuth update.
  /// @param fov_state_before FoV state before the azimuth update.
  /// @param fov_state_after FoV state after the azimuth update.
  /// @param current_buffer The current active buffer index.
  /// @return The transition actions to perform.
  [[nodiscard]] static TransitionActions step(
    const ChannelBufferState & buffer_state_before, const ChannelBufferState & buffer_state_after,
    const ChannelFovState & fov_state_before, const ChannelFovState & fov_state_after,
    buffer_index_t current_buffer)
  {
    State state_before = determine_state(buffer_state_before, fov_state_before, current_buffer);
    State state_after = determine_state(buffer_state_after, fov_state_after, current_buffer);
    return get_transition_actions(state_before, state_after);
  }
};

}  // namespace nebula::drivers
