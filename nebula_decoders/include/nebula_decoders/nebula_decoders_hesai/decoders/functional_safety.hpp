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

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"

#include <boost/container/static_vector.hpp>

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

namespace nebula::drivers
{

enum class FunctionalSafetySeverity : uint8_t { OK, WARNING, ERROR };
using FunctionalSafetyErrorCodes = boost::container::static_vector<uint16_t, 16>;

class FunctionalSafetyDecoderBase
{
public:
  using alive_cb_t = std::function<void()>;
  using stuck_cb_t = std::function<void(bool is_stuck)>;
  using status_cb_t = std::function<void(FunctionalSafetySeverity, FunctionalSafetyErrorCodes)>;

  virtual ~FunctionalSafetyDecoderBase() = default;
};

template <typename PacketT>
class FunctionalSafetyDecoderTypedBase : public FunctionalSafetyDecoderBase
{
public:
  virtual ~FunctionalSafetyDecoderTypedBase() = default;

  virtual void update(const PacketT & /* packet */) {}

  virtual void set_alive_callback(alive_cb_t /* on_alive */) {}
  virtual void set_stuck_callback(stuck_cb_t /* on_stuck */) {}
  virtual void set_status_callback(status_cb_t /* on_status */) {}
};

/**
 * @brief Interprets the functional safety part of Hesai pointcloud packets for supported
 * sensor models.
 *
 * @tparam PacketT A packet definition that has a supported functional safety section.
 */
template <typename PacketT>
class FunctionalSafetyDecoder : public FunctionalSafetyDecoderTypedBase<PacketT>
{
  using functional_safety_t = decltype(PacketT::fs);

  using alive_cb_t = typename FunctionalSafetyDecoderBase::alive_cb_t;
  using stuck_cb_t = typename FunctionalSafetyDecoderBase::stuck_cb_t;
  using status_cb_t = typename FunctionalSafetyDecoderBase::status_cb_t;

public:
  void update(const PacketT & packet) override
  {
    uint64_t timestamp_ns = hesai_packet::get_timestamp_ns(packet);
    const functional_safety_t & fs = packet.fs;

    // Prove to dependent modules that we are receiving data frequently.
    // This has nothing to do with the validity of the data we are receiving.
    if (on_alive_) on_alive_();

    // A corrupted packet is not an error and shall simply be ignored.
    if (!hesai_packet::is_crc_valid(fs)) return;

    // The sensor sends functional safety data with every packet,
    // but it only changes at a fixed rate e.g. every 5 ms.
    bool changed = has_changed(fs);

    // Data not changing at that fixed rate signals the sensor's fault reporting system being stuck.
    bool is_stuck = !changed && is_overdue(timestamp_ns);
    if (on_stuck_) {
      on_stuck_(is_stuck);
    }

    // If there is no update, the data is ignored.
    if (!changed) return;

    // From here on, it is guaranteed that data has changed.
    FunctionalSafetySeverity severity = fs.severity();

    // Error codes are queued on the sensor side, and with each cycle, the next queued code is
    // sent. Call back only when all codes of a queue have been received, otherwise continue
    // accumulating.
    std::optional<FunctionalSafetyErrorCodes> accumulated_codes =
      try_accumulate_error_codes(timestamp_ns, fs);
    if (accumulated_codes) {
      if (on_status_) on_status_(severity, std::move(*accumulated_codes));
    }

    // Store the changed, received values for the next iteration
    last_changed_timestamp_ns_ = timestamp_ns;
    last_value_ = fs;
  }

  void set_alive_callback(alive_cb_t on_alive) override { on_alive_ = std::move(on_alive); }

  void set_stuck_callback(stuck_cb_t on_stuck) override { on_stuck_ = std::move(on_stuck); }

  void set_status_callback(status_cb_t on_status) override { on_status_ = std::move(on_status); }

private:
  bool has_changed(const functional_safety_t & current_value)
  {
    // From Hesai's safety manuals:
    // The rolling counter has to change every N ms (where N is sensor-specific). If it does
    // not, treat the fault report system as stuck

    // In reality:
    // For non-360 deg FoVs, there is a phase where no packets are sent by the sensor, which
    // can be longer than those N ms. The rolling counter seems to not increase in those dead
    // zones, but other information, such as the fault code currently reported might change.
    // Thus, we treat *any* change in the data as a signal that the fault report system is alive.
    bool anything_changed = current_value != last_value_;
    return anything_changed;
  }

  [[nodiscard]] bool is_overdue(uint64_t timestamp_ns) const
  {
    // While the data shall change every N ms, in non-360 deg FoVs, there is a phase where no
    // packets are sent by the sensor. To comply with any FoV, we have to allow for 100 ms (assuming
    // a 10 Hz scan rate) before  reporting the system as stuck.
    return (timestamp_ns - last_changed_timestamp_ns_) >= 100'000'000;
  }

  std::optional<FunctionalSafetyErrorCodes> try_accumulate_error_codes(
    uint64_t timestamp_ns, const functional_safety_t & fs)
  {
    uint8_t n_codes = fs.total_fault_code_num();

    // There is no reported error, report an empty error list
    if (n_codes == 0) {
      current_error_codes_ = {};
      return {FunctionalSafetyErrorCodes{}};
    }

    uint8_t i_code = fs.fault_code_id();
    uint16_t code = fs.fault_code;

    // In non-360 deg FoVs, there might be a phase where no packets are sent. Once the packet
    // stream resumes, the old fault queue is gone and a new one (which might be identical or
    // entirely different) is active. This invalidates our buffer.
    bool has_time_jumped =
      (timestamp_ns - last_changed_timestamp_ns_) >= 2 * functional_safety_t::update_cycle_ns;
    // If there is an unexpected skip between fault code indices, invalidate the buffer.
    bool is_expected_index = i_code == current_error_codes_.size() + 1;
    bool is_accumulator_valid = !has_time_jumped && is_expected_index;

    if (!is_accumulator_valid) {
      current_error_codes_ = {};
      // If we are in the middle of a fault queue, we are missing all codes up to the current
      // index. Wait for the next cycle to begin instead.
      if (i_code != 1) return std::nullopt;
    }

    assert(current_error_codes_.size() < current_error_codes_.capacity());
    current_error_codes_.push_back(code);

    bool has_accumulated = current_error_codes_.size() == n_codes;
    if (has_accumulated) {
      std::optional<FunctionalSafetyErrorCodes> result = {std::move(current_error_codes_)};
      current_error_codes_ = {};
      return result;
    }

    return std::nullopt;
  }

  uint64_t last_changed_timestamp_ns_{};
  functional_safety_t last_value_;

  FunctionalSafetyErrorCodes current_error_codes_;

  alive_cb_t on_alive_;
  stuck_cb_t on_stuck_;
  status_cb_t on_status_;
};

}  // namespace nebula::drivers
