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

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
namespace nebula::drivers
{

template <typename PacketT>
class PacketLossDetector
{
  using udp_sequence_t = decltype(PacketT::tail::udp_sequence);

public:
  using lost_cb_t = std::function<void(uint64_t n_lost)>;

  explicit PacketLossDetector(lost_cb_t on_lost) : on_lost_(std::move(on_lost)) {}

  void update(udp_sequence_t current_sequence)
  {
    if (!previous_udp_sequence_) {
      previous_udp_sequence_ = current_sequence;
      return;
    }

    udp_sequence_t next_expected_sequence = *previous_udp_sequence_ + 1;
    if (current_sequence != next_expected_sequence) {
      if (on_lost_) on_lost_(next_expected_sequence - current_sequence);
    }

    previous_udp_sequence_ = current_sequence;
  }

private:
  std::optional<udp_sequence_t> previous_udp_sequence_{};
  lost_cb_t on_lost_;
};

}  // namespace nebula::drivers
