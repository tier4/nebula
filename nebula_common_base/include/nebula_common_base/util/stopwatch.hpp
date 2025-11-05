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

#include <chrono>
#include <cstdint>

namespace nebula::util
{

/// A stop watch for performance measurements. Starts automatically on construction.
class Stopwatch
{
public:
  /// Constructs and starts the stopwatch
  Stopwatch() { reset(); }

  /// Resets the start time to the current moment
  void reset() { start_time_ = std::chrono::steady_clock::now(); }

  /// Returns elapsed time in nanoseconds since last reset or construction
  [[nodiscard]] uint64_t elapsed_ns() const
  {
    return (std::chrono::steady_clock::now() - start_time_).count();
  }

private:
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace nebula::util
