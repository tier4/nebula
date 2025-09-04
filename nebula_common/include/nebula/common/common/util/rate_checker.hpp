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

#include "nebula_common/util/ring_buffer.hpp"

#include <optional>

namespace nebula::util
{

class RateChecker
{
public:
  RateChecker(double min_rate_hz, double max_rate_hz, std::size_t buffer_size)
  : min_rate_hz_(min_rate_hz), max_rate_hz_(max_rate_hz), ring_buffer_(buffer_size)
  {
  }

  [[nodiscard]] bool is_full() const { return ring_buffer_.is_full(); }

  void update(double stamp)
  {
    if (last_stamp_) {
      ring_buffer_.push_back(stamp - last_stamp_.value());
    }

    last_stamp_ = stamp;
  }

  [[nodiscard]] bool is_valid() const
  {
    if (ring_buffer_.size() == 0) return false;
    double average = get_average();
    return average >= min_rate_hz_ && average <= max_rate_hz_;
  }

  [[nodiscard]] double get_average() const { return 1.0 / ring_buffer_.get_average(); }

private:
  std::optional<double> last_stamp_;
  double min_rate_hz_;
  double max_rate_hz_;
  RingBuffer<double> ring_buffer_;
};

}  // namespace nebula::util
