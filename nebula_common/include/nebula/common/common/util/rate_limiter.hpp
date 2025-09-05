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
#include <functional>

namespace nebula::util
{

class RateLimiter
{
public:
  explicit RateLimiter(std::chrono::milliseconds rate_limit)
  : rate_limit_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(rate_limit).count())
  {
  }

  void with_rate_limit(uint64_t now_ns, const std::function<void()> & action)
  {
    if (now_ns - last_passed_time_ns_ < rate_limit_ns_) {
      return;
    }

    last_passed_time_ns_ = now_ns;
    action();
  }

private:
  uint64_t rate_limit_ns_;
  uint64_t last_passed_time_ns_{0};
};

}  // namespace nebula::util
