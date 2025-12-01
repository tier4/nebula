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

#include "nebula_core_ros/diagnostics/hysteresis_state_machine.hpp"

#include <gtest/gtest.h>

using custom_diagnostic_tasks::Error;
using custom_diagnostic_tasks::generate_state;
using custom_diagnostic_tasks::get_level;
using custom_diagnostic_tasks::get_level_string;
using custom_diagnostic_tasks::get_num_observations;
using custom_diagnostic_tasks::HysteresisStateMachine;
using custom_diagnostic_tasks::Ok;
using custom_diagnostic_tasks::Stale;
using custom_diagnostic_tasks::Warn;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

// ============================================================================
// Helper functions tests
// ============================================================================

TEST(HysteresisHelperTest, GenerateState)
{
  auto stale = generate_state(DiagStatus::STALE);
  EXPECT_EQ(get_level(stale), DiagStatus::STALE);

  auto ok = generate_state(DiagStatus::OK);
  EXPECT_EQ(get_level(ok), DiagStatus::OK);

  auto warn = generate_state(DiagStatus::WARN);
  EXPECT_EQ(get_level(warn), DiagStatus::WARN);

  auto error = generate_state(DiagStatus::ERROR);
  EXPECT_EQ(get_level(error), DiagStatus::ERROR);
}

TEST(HysteresisHelperTest, GenerateStateThrowsOnInvalid)
{
  EXPECT_THROW(generate_state(99), std::runtime_error);
}

TEST(HysteresisHelperTest, GetLevelString)
{
  EXPECT_EQ(get_level_string(DiagStatus::OK), "OK");
  EXPECT_EQ(get_level_string(DiagStatus::WARN), "WARN");
  EXPECT_EQ(get_level_string(DiagStatus::ERROR), "ERROR");
  EXPECT_EQ(get_level_string(DiagStatus::STALE), "STALE");
  EXPECT_EQ(get_level_string(99), "UNDEFINED");
}

TEST(HysteresisHelperTest, GetNumObservations)
{
  auto state = generate_state(DiagStatus::OK);
  EXPECT_EQ(get_num_observations(state), 1);
}

// ============================================================================
// State structs tests
// ============================================================================

TEST(StateBaseTest, StaleInitialization)
{
  Stale stale;
  EXPECT_EQ(stale.level, DiagStatus::STALE);
  EXPECT_EQ(stale.num_observations, 1);
}

TEST(StateBaseTest, OkInitialization)
{
  Ok ok;
  EXPECT_EQ(ok.level, DiagStatus::OK);
  EXPECT_EQ(ok.num_observations, 1);
}

TEST(StateBaseTest, WarnInitialization)
{
  Warn warn;
  EXPECT_EQ(warn.level, DiagStatus::WARN);
  EXPECT_EQ(warn.num_observations, 1);
}

TEST(StateBaseTest, ErrorInitialization)
{
  Error error;
  EXPECT_EQ(error.level, DiagStatus::ERROR);
  EXPECT_EQ(error.num_observations, 1);
}

// ============================================================================
// HysteresisStateMachine basic tests
// ============================================================================

TEST(HysteresisStateMachineTest, DefaultConstruction)
{
  HysteresisStateMachine hsm;

  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::STALE);
  EXPECT_EQ(hsm.get_num_frame_transition(), 1);
  EXPECT_FALSE(hsm.get_immediate_error_report_param());
  EXPECT_TRUE(hsm.get_immediate_relax_state_param());
}

TEST(HysteresisStateMachineTest, ParameterizedConstruction)
{
  HysteresisStateMachine hsm(5, true, false);

  EXPECT_EQ(hsm.get_num_frame_transition(), 5);
  EXPECT_TRUE(hsm.get_immediate_error_report_param());
  EXPECT_FALSE(hsm.get_immediate_relax_state_param());
}

TEST(HysteresisStateMachineTest, ZeroFrameTransitionClamped)
{
  HysteresisStateMachine hsm(0, false, true);

  // Should be clamped to 1
  EXPECT_EQ(hsm.get_num_frame_transition(), 1);
}

// ============================================================================
// State transition tests
// ============================================================================

TEST(HysteresisStateMachineTest, ImmediateTransitionWithThresholdOne)
{
  HysteresisStateMachine hsm(1);  // Threshold 1 means immediate transition

  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  hsm.update_state(DiagStatus::WARN);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::WARN);

  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);
}

TEST(HysteresisStateMachineTest, HysteresisDelaysTransition)
{
  HysteresisStateMachine hsm(3, false, false);  // Need 3 observations, no immediate relax

  // Start at STALE
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::STALE);

  // First OK observation - should not transition yet
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::STALE);
  EXPECT_EQ(hsm.get_candidate_level(), DiagStatus::OK);
  EXPECT_EQ(hsm.get_candidate_num_observation(), 1);

  // Second OK observation
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::STALE);
  EXPECT_EQ(hsm.get_candidate_num_observation(), 2);

  // Third OK observation - should transition now
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

TEST(HysteresisStateMachineTest, CandidateResetOnDifferentObservation)
{
  HysteresisStateMachine hsm(3, false, false);

  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);  // 2 OK observations

  // Different observation resets candidate
  hsm.update_state(DiagStatus::WARN);
  EXPECT_EQ(hsm.get_candidate_level(), DiagStatus::WARN);
  EXPECT_EQ(hsm.get_candidate_num_observation(), 1);
}

// ============================================================================
// Immediate error report tests
// ============================================================================

TEST(HysteresisStateMachineTest, ImmediateErrorReportEnabled)
{
  HysteresisStateMachine hsm(5, true, false);  // Immediate error, threshold 5

  // Start with some OK observations
  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);

  // ERROR should be reported immediately
  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);
}

TEST(HysteresisStateMachineTest, ImmediateErrorReportDisabled)
{
  HysteresisStateMachine hsm(3, false, false);  // No immediate error

  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  // ERROR should not be immediate
  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  // Third ERROR observation triggers transition
  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);
}

// ============================================================================
// Immediate relax state tests
// ============================================================================

TEST(HysteresisStateMachineTest, ImmediateRelaxEnabled)
{
  HysteresisStateMachine hsm(5, false, true);  // Immediate relax

  // Get to ERROR state
  hsm.set_current_state_level(DiagStatus::ERROR);

  // OK should immediately relax the state (OK < ERROR)
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

TEST(HysteresisStateMachineTest, ImmediateRelaxDisabled)
{
  HysteresisStateMachine hsm(3, false, false);  // No immediate relax

  // Get to ERROR state
  hsm.set_current_state_level(DiagStatus::ERROR);

  // OK should not immediately relax
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);

  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);

  // Third OK observation triggers transition
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

TEST(HysteresisStateMachineTest, RelaxOnlyToLowerSeverity)
{
  HysteresisStateMachine hsm(5, false, true);  // Immediate relax

  hsm.set_current_state_level(DiagStatus::WARN);

  // OK should relax (OK < WARN)
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

TEST(HysteresisStateMachineTest, NoRelaxToHigherSeverity)
{
  HysteresisStateMachine hsm(5, false, true);  // Immediate relax

  hsm.set_current_state_level(DiagStatus::WARN);

  // ERROR is higher severity, should not "relax"
  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::WARN);  // Still WARN
}

// ============================================================================
// Set current state level tests
// ============================================================================

TEST(HysteresisStateMachineTest, SetCurrentStateLevel)
{
  HysteresisStateMachine hsm;

  hsm.set_current_state_level(DiagStatus::WARN);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::WARN);

  hsm.set_current_state_level(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);

  hsm.set_current_state_level(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

// ============================================================================
// Complex scenario tests
// ============================================================================

TEST(HysteresisStateMachineTest, RealWorldScenario)
{
  // Simulate a sensor that occasionally has glitches
  HysteresisStateMachine hsm(3, true, true);  // 3 frames, immediate error, immediate relax

  // Start with OK data
  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  // Single warning glitch - should not change state
  hsm.update_state(DiagStatus::WARN);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);  // Hysteresis not yet reached

  // Back to OK
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);

  // Sustained warning
  hsm.update_state(DiagStatus::WARN);
  hsm.update_state(DiagStatus::WARN);
  hsm.update_state(DiagStatus::WARN);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::WARN);

  // Error should be immediate
  hsm.update_state(DiagStatus::ERROR);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::ERROR);

  // Recovery to OK should be immediate (relaxation)
  hsm.update_state(DiagStatus::OK);
  EXPECT_EQ(hsm.get_current_state_level(), DiagStatus::OK);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
