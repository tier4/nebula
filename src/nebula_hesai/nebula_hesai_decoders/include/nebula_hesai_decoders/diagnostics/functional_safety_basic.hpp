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

#include "nebula_hesai_decoders/diagnostics/functional_safety_monitor.hpp"

namespace nebula::drivers
{

/**
 * @brief Takes over the severity reported by the sensor as-is.
 *
 * Since error names and severities for many Hesai LiDARs are not published to the general
 * public, this module only provides a single status, as reported by the sensor, along with a
 * list of error codes.
 */
class FunctionalSafetyBasic : public FunctionalSafetyStatusProcessor
{
public:
  [[nodiscard]] FunctionalSafetyStatus process(
    FunctionalSafetySeverity severity, const FunctionalSafetyErrorCodes & error_codes) override
  {
    FunctionalSafetyStatus status;
    status.sensor_severity = severity;
    status.severity = severity;
    status.error_codes = error_codes;
    return status;
  }
};

}  // namespace nebula::drivers
