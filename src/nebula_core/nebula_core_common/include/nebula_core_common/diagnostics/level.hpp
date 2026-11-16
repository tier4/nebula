// Copyright 2026 TIER IV, Inc.
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

#include <cstdint>
#include <string>

namespace nebula::diagnostics
{

/// @brief How severe the state reported by a monitor is.
/// @details The values are the ones of `diagnostic_msgs/msg/DiagnosticStatus`.
enum class Level : uint8_t {
  OK = 0,
  WARN = 1,
  ERROR = 2,
  /// No up-to-date information is available
  STALE = 3,
};

inline std::string to_string(Level level)
{
  switch (level) {
    case Level::OK:
      return "OK";
    case Level::WARN:
      return "WARN";
    case Level::ERROR:
      return "ERROR";
    case Level::STALE:
      return "STALE";
    default:
      return "UNDEFINED";
  }
}

}  // namespace nebula::diagnostics
