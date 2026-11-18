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
#include <limits>
#include <optional>
#include <utility>

namespace nebula::drivers
{

class PacketLossDetectorBase
{
public:
  using lost_cb_t = std::function<void(uint64_t n_lost)>;

  virtual ~PacketLossDetectorBase() = default;

  virtual void set_lost_callback(lost_cb_t /* on_lost */) {}
};

template <typename PacketT>
class PacketLossDetectorTypedBase : public PacketLossDetectorBase
{
public:
  virtual void update(const PacketT & /* packet */) {}
};

template <typename PacketT>
class PacketLossDetector : public PacketLossDetectorTypedBase<PacketT>
{
  using udp_sequence_t = decltype(std::declval<PacketT>().tail.udp_sequence);

public:
  using lost_cb_t = PacketLossDetectorBase::lost_cb_t;

  void set_lost_callback(lost_cb_t on_lost) override { on_lost_ = std::move(on_lost); }

  void update(const PacketT & packet)
  {
    if (!previous_udp_sequence_) {
      previous_udp_sequence_.emplace(packet.tail.udp_sequence);
      return;
    }

    // Sequence numbers wrap around, so the gap is taken modulo the range of udp_sequence_t. A gap
    // in the upper half of that range means that the sequence number went backwards, e.g. because
    // a packet was duplicated or reordered, or because the sensor restarted. That is not a loss.
    const auto n_lost =
      static_cast<udp_sequence_t>(packet.tail.udp_sequence - *previous_udp_sequence_ - 1);
    const bool went_backwards = n_lost > std::numeric_limits<udp_sequence_t>::max() / 2;
    if (n_lost != 0 && !went_backwards && on_lost_) {
      on_lost_(n_lost);
    }

    previous_udp_sequence_.emplace(packet.tail.udp_sequence);
  }

private:
  std::optional<udp_sequence_t> previous_udp_sequence_{};
  lost_cb_t on_lost_;
};

}  // namespace nebula::drivers
