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

namespace nebula::util
{

class RateChecker
{
public:
  RateChecker(double min_rate, double max_rate, std::size_t buffer_size)
  : min_rate_(min_rate), max_rate_(max_rate), ring_buffer_(buffer_size)
  {
  }

  bool is_valid(double stamp)
  {
    if (last_stamp_ > 0.0) {
      double current_time = stamp;
      double dt = stamp - last_stamp_;
      ring_buffer_.push_back(dt);
    }

    last_stamp_ = stamp;

    if (!ring_buffer_.is_full()) {
      return true;
    }

    double average = 1.0 / ring_buffer_.get_average();
    return average >= min_rate_ && average <= max_rate_;
  }

  double get_average() const { return 1.0 / ring_buffer_.get_average(); }

private:
  double last_stamp_{0.0};
  double min_rate_;
  double max_rate_;
  RingBuffer<double> ring_buffer_;
};

}  // namespace nebula::util
