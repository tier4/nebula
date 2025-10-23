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

#include "nebula_ros/hesai/diagnostics/functional_safety_diagnostic_task.hpp"

namespace nebula::ros
{

/**
 * @brief Forwards sensor error codes and LiDAR status to the diagnostic updater as-is.
 *
 * Since error names and severities for many Hesai LiDARs are not published to the general
 * public, this module only provides a single status, as reported by the sensor, along with a
 * list of error codes.
 */
class FunctionalSafetyBasic : public FunctionalSafetyStatusProcessor
{
public:
  void populate_status(
    drivers::FunctionalSafetySeverity severity,
    const drivers::FunctionalSafetyErrorCodes & error_codes,
    diagnostic_msgs::msg::DiagnosticStatus & inout_status) override
  {
    inout_status.level = detail::severity_to_diagnostic_status_level(severity);
    inout_status.message = detail::status_to_string(severity, error_codes.size());

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Diagnostic codes";
    kv.value = detail::error_codes_to_string(error_codes);
    inout_status.values.push_back(kv);
  }
};

}  // namespace nebula::ros
