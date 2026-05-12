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

#include "nebula_hesai/diagnostics/functional_safety_advanced.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"

#include <boost/range/algorithm/sort.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#ifndef _TEST_RESOURCES_PATH
static_assert(false, " _TEST_RESOURCES_PATH is not defined");
#endif

using Severity = nebula::drivers::FunctionalSafetySeverity;
using diagnostic_msgs::msg::DiagnosticStatus;

struct CsvTestCase
{
  std::filesystem::path path;
  bool expected_valid;
  std::vector<nebula::ros::ErrorDefinition> expected_error_definitions;
};

struct PopulateStatusTestCase
{
  std::string name;
  Severity arg_severity;
  nebula::drivers::FunctionalSafetyErrorCodes arg_error_codes;

  std::vector<uint16_t> exempted_error_codes;
  uint8_t expected_status_level;  ///< DiagnosticStatus::{OK, WARN, ERROR}
};

namespace
{

std::vector<CsvTestCase> csv_test_cases()
{
  auto to_path = [](const std::string & name) {
    return std::filesystem::path(_TEST_RESOURCES_PATH) / "fusa_codes" / (name + ".csv");
  };
  return {
    {to_path("invalid_bigger_than_uint16"), false, {}},
    {to_path("invalid_lidar_state"), false, {}},
    {to_path("invalid_no_0x"), false, {}},
    {to_path("invalid_not_enough_fields"), false, {}},
    {to_path("valid_with_duplicates"),
     true,
     {
       {0x1111, "Aaa", Severity::OK},
     }},
    {to_path("valid_with_multi_severity_duplicates"),
     true,
     {
       {0x1111, "Aaa OR Bbb", Severity::ERROR},
     }},
    {to_path("valid_with_empty_lines"),
     true,
     {
       {0x1111, "Aaa", Severity::OK},
       {0x0a0b, "Bbb", Severity::OK},
     }},
    {to_path("valid_with_header"),
     true,
     {
       {0x1111, "Aaa", Severity::OK},
       {0x2222, "Bbb", Severity::WARNING},
       {0x3333, "Ccc", Severity::WARNING},
       {0x4444, "Ddd", Severity::ERROR},
       {0x5555, "Eee", Severity::ERROR},
     }},
  };
}

std::vector<nebula::ros::ErrorDefinition> error_definitions()
{
  return {
    {0x1111, "Aaa", Severity::OK},
    {0x2222, "Bbb", Severity::WARNING},
    {0x3333, "Ccc", Severity::ERROR},
  };
}

std::vector<PopulateStatusTestCase> populate_status_test_cases()
{
  return {
    // If no error codes are provided, the status level is the severity reported by the sensor
    {"no_codes_ok", Severity::OK, {}, {}, DiagnosticStatus::OK},
    {"no_codes_warn", Severity::WARNING, {}, {}, DiagnosticStatus::WARN},
    {"no_codes_error", Severity::ERROR, {}, {}, DiagnosticStatus::ERROR},
    // Error definitions override severity reported by the sensor
    {"def_ok", Severity::ERROR, {0x1111}, {}, DiagnosticStatus::OK},
    {"def_warn", Severity::ERROR, {0x2222}, {}, DiagnosticStatus::WARN},
    {"def_error", Severity::OK, {0x3333}, {}, DiagnosticStatus::ERROR},
    // The reported status is the worst severity of the non-exempted error codes
    {"exempt_ok_leave_error",
     Severity::OK,
     {0x1111, 0x2222, 0x3333},
     {0x1111},
     DiagnosticStatus::ERROR},
    {"exempt_error_leave_warn",
     Severity::OK,
     {0x1111, 0x2222, 0x3333},
     {0x3333},
     DiagnosticStatus::WARN},
    {"exempt_all_but_ok",
     Severity::OK,
     {0x1111, 0x2222, 0x3333},
     {0x2222, 0x3333},
     DiagnosticStatus::OK},
    // If all are exempted, the status is OK
    {"exempt_all",
     Severity::ERROR,
     {0x1111, 0x2222, 0x3333},
     {0x1111, 0x2222, 0x3333},
     DiagnosticStatus::OK},
  };
}

}  // namespace

class CsvReaderTest : public ::testing::TestWithParam<CsvTestCase>
{
};

TEST_P(CsvReaderTest, ReadCsv)
{
  const auto & test_case = GetParam();
  if (!test_case.expected_valid) {
    EXPECT_THROW(nebula::ros::read_error_definitions_from_csv(test_case.path), std::runtime_error);
    return;
  }

  std::vector<nebula::ros::ErrorDefinition> parsed;
  EXPECT_NO_THROW(parsed = nebula::ros::read_error_definitions_from_csv(test_case.path));

  auto expected = test_case.expected_error_definitions;

  boost::range::sort(parsed, [](const auto & a, const auto & b) { return a.code < b.code; });
  boost::range::sort(expected, [](const auto & a, const auto & b) { return a.code < b.code; });

  EXPECT_EQ(parsed, expected);
}

INSTANTIATE_TEST_SUITE_P(
  CsvReaderTest, CsvReaderTest, testing::ValuesIn(csv_test_cases()),
  [](const testing::TestParamInfo<CsvTestCase> & p) { return p.param.path.stem().string(); });

TEST(FunctionalSafetyAdvanced, ConstructorValidation)
{
  EXPECT_NO_THROW(nebula::ros::FunctionalSafetyAdvanced({}, {}));
  EXPECT_NO_THROW(nebula::ros::FunctionalSafetyAdvanced({{0x1111, "Aaa", Severity::OK}}, {}));
  EXPECT_NO_THROW(nebula::ros::FunctionalSafetyAdvanced({}, {0x1111}));
  EXPECT_NO_THROW(nebula::ros::FunctionalSafetyAdvanced({{0x1111, "Aaa", Severity::OK}}, {0x1111}));
}

class PopulateStatusTest : public ::testing::TestWithParam<PopulateStatusTestCase>
{
};

TEST_P(PopulateStatusTest, PopulateStatus)
{
  const auto & test_case = GetParam();
  nebula::ros::FunctionalSafetyAdvanced functional_safety_advanced(
    error_definitions(), test_case.exempted_error_codes);
  diagnostic_msgs::msg::DiagnosticStatus status;
  functional_safety_advanced.populate_status(
    test_case.arg_severity, test_case.arg_error_codes, status);
  EXPECT_EQ(status.level, test_case.expected_status_level);
}

INSTANTIATE_TEST_SUITE_P(
  PopulateStatusTest, PopulateStatusTest, testing::ValuesIn(populate_status_test_cases()),
  [](const testing::TestParamInfo<PopulateStatusTestCase> & p) { return p.param.name; });

TEST(FunctionalSafetyAdvanced, PopulateStatus)
{
  nebula::ros::FunctionalSafetyAdvanced functional_safety_advanced(error_definitions(), {});
  diagnostic_msgs::msg::DiagnosticStatus status;
  functional_safety_advanced.populate_status(Severity::OK, {0x1111}, status);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Operating nominally");
}
