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

#include <chrono>

namespace nebula::util
{

/**
 * @brief A source of time that is injected into components instead of being read from a global.
 *
 * Components only compare and subtract the returned values, so the epoch is up to the
 * implementation. This allows running them on wall time, on ROS time, or on the time of recorded
 * data during offline decoding.
 */
class Clock
{
public:
  Clock() = default;
  Clock(const Clock &) = delete;
  Clock & operator=(const Clock &) = delete;
  Clock(Clock &&) = delete;
  Clock & operator=(Clock &&) = delete;
  virtual ~Clock() = default;

  /// @brief The current time as a duration since the clock's epoch.
  [[nodiscard]] virtual std::chrono::nanoseconds now() const = 0;
};

/// @brief Monotonic time, unaffected by system time changes.
class SteadyClock final : public Clock
{
public:
  [[nodiscard]] std::chrono::nanoseconds now() const override
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
  }
};

}  // namespace nebula::util
