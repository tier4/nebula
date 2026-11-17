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

#include <nebula_core_common/diagnostics/level.hpp>
#include <nebula_core_common/diagnostics/liveness_monitor.hpp>
#include <nebula_core_common/diagnostics/severity_latch.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <nebula_hesai_decoders/decoders/functional_safety.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

inline diagnostics::Level to_level(FunctionalSafetySeverity severity)
{
  switch (severity) {
    case FunctionalSafetySeverity::OK:
      return diagnostics::Level::OK;
    case FunctionalSafetySeverity::WARNING:
      return diagnostics::Level::WARN;
    default:
    case FunctionalSafetySeverity::ERROR:
      return diagnostics::Level::ERROR;
  }
}

/// @brief An error code reported by the sensor, interpreted with the configured error definitions
struct FunctionalSafetyFault
{
  uint16_t code{};
  /// The description from the error definitions, or "Unknown error" if the code is not defined
  std::string description;
  /// The severity from the error definitions, or the one reported by the sensor if the code is not
  /// defined
  FunctionalSafetySeverity severity{};
  /// Whether the fault is configured to not affect the status
  bool is_exempted{};

  bool operator==(const FunctionalSafetyFault & other) const
  {
    return code == other.code && description == other.description && severity == other.severity &&
           is_exempted == other.is_exempted;
  }
};

/// @brief One functional safety message of the sensor, and what Nebula makes of it
struct FunctionalSafetyStatus
{
  /// The severity as reported by the sensor
  FunctionalSafetySeverity sensor_severity{};
  /// The severity after taking error definitions and exemptions into account
  FunctionalSafetySeverity severity{};
  /// All error codes reported by the sensor
  FunctionalSafetyErrorCodes error_codes;
  /// `error_codes` interpreted with the error definitions. Unset if none are configured.
  std::optional<std::vector<FunctionalSafetyFault>> faults;

  [[nodiscard]] diagnostics::Level level() const { return to_level(severity); }
};

/// @brief Decides how severe the errors reported by the sensor are
class FunctionalSafetyStatusProcessor
{
public:
  virtual ~FunctionalSafetyStatusProcessor() = default;
  [[nodiscard]] virtual FunctionalSafetyStatus process(
    FunctionalSafetySeverity severity, const FunctionalSafetyErrorCodes & error_codes) = 0;
};

/// @brief Monitors the functional safety messages of a sensor
class FunctionalSafetyMonitor
{
public:
  struct Report
  {
    /// Whether functional safety messages keep arriving
    diagnostics::LivenessMonitor::Report liveness;
    /// The most severe status since the last report. Unset if none has been received yet.
    std::optional<FunctionalSafetyStatus> status;
    /// Whether the sensor's functional safety system stopped updating. Unset if not known yet.
    std::optional<bool> is_stuck;

    [[nodiscard]] diagnostics::Level level() const
    {
      using diagnostics::Level;
      const Level status_level = status ? status->level() : Level::STALE;
      const Level stuck_level =
        is_stuck ? (is_stuck.value() ? Level::ERROR : Level::OK) : Level::STALE;
      return std::max({liveness.level(), status_level, stuck_level});
    }
  };

  FunctionalSafetyMonitor(
    const std::shared_ptr<const util::Clock> & clock,
    std::unique_ptr<FunctionalSafetyStatusProcessor> status_processor)
  : liveness_monitor_(clock, std::chrono::milliseconds(100)),
    status_processor_(std::move(status_processor))
  {
  }

  void on_status(FunctionalSafetySeverity severity, const FunctionalSafetyErrorCodes & error_codes)
  {
    auto status = status_processor_->process(severity, error_codes);
    const auto level = status.level();
    status_latch_.submit(level, std::move(status));
  }

  void on_alive() { liveness_monitor_.tick(); }

  void on_stuck(bool is_stuck)
  {
    stuck_latch_.submit(is_stuck ? diagnostics::Level::ERROR : diagnostics::Level::OK, is_stuck);
  }

  [[nodiscard]] Report report()
  {
    Report report;
    report.liveness = liveness_monitor_.report();
    if (auto status = status_latch_.report()) {
      report.status = std::move(status->status);
    }
    if (auto stuck = stuck_latch_.report()) {
      report.is_stuck = stuck->status;
    }
    return report;
  }

private:
  diagnostics::LivenessMonitor liveness_monitor_;
  diagnostics::SeverityLatch<FunctionalSafetyStatus> status_latch_;
  diagnostics::SeverityLatch<bool> stuck_latch_;
  std::unique_ptr<FunctionalSafetyStatusProcessor> status_processor_;
};

}  // namespace nebula::drivers
