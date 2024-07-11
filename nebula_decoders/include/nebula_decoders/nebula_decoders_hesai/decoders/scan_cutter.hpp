// Copyright 2024 TIER IV, Inc.
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace nebula::drivers
{

namespace scan_cutter::detail
{

enum class Parity : bool { Tik, Tok };
struct ChannelState
{
  Parity parity;
  bool in_fov;
};

inline void toggleParity(Parity & parity)
{
  parity = (parity == Parity::Tik) ? Parity::Tok : Parity::Tik;
}

inline bool operator==(const ChannelState & lhs, const ChannelState & rhs)
{
  return lhs.parity == rhs.parity && lhs.in_fov == rhs.in_fov;
}

}  // namespace scan_cutter::detail

using scan_cutter::detail::ChannelState;
using scan_cutter::detail::Parity;
using std::rel_ops::operator!=;

template <size_t ChannelN, typename AngleCorrectorT>
class ScanCutter
{
public:
  explicit ScanCutter(const AngleCorrectorT & angle_corrector) : angle_corrector_(angle_corrector)
  {
    channel_states_.fill({Parity::Tik, true});
  }

  struct ScanCutResult
  {
    bool cloud_ended;
    bool cloud_started;
  };

  ScanCutResult update(uint32_t current_azimuth)
  {
    auto last_azimuth = last_azimuth_;
    last_azimuth_ = current_azimuth;

    if (last_azimuth == -1) return {false, false};

    bool any_crossed_start = false;
    bool any_crossed_end = false;

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      ChannelState & channel_state = channel_states_[channel_id];

      bool crossed_scan_start =
        angle_corrector_.didChannelPassFovStart(current_azimuth, last_azimuth, channel_id);
      bool crossed_scan_end =
        angle_corrector_.didChannelPassFovEnd(current_azimuth, last_azimuth, channel_id);

      any_crossed_start |= crossed_scan_start;
      any_crossed_end |= crossed_scan_end;

      channel_state.in_fov = angle_corrector_.isChannelInFov(current_azimuth, channel_id);
      if (crossed_scan_end) {
        toggleParity(channel_state.parity);
      }
    }

    bool previously_in_transition = in_transition_;
    in_transition_ = false;
    const ChannelState & reference_state = channel_states_[0];
    for (const ChannelState & state : channel_states_) {
      if (state != reference_state) in_transition_ = true;
    }

    ScanCutResult result{false, false};

    if (any_crossed_end && !in_transition_) result.cloud_ended = true;
    if (any_crossed_start && !previously_in_transition) result.cloud_started = true;

    if (result.cloud_ended) {
      scan_cutter::detail::toggleParity(scan_parity_);
    }

    return result;
  }

  bool isInCurrentScan(size_t channel_id)
  {
    return scan_parity_ == channel_states_[channel_id].parity;
  }

private:
  /// @brief The last azimuth processed
  int last_azimuth_ = -1;  // Dummy value to signal last_phase_ has not been set yet
  bool in_transition_ = false;

  Parity scan_parity_ = Parity::Tik;

  /// @brief For each channel that passed over the FoV end n times, this value is (n % 2)
  std::array<ChannelState, ChannelN> channel_states_;

  const AngleCorrectorT & angle_corrector_;
};

}  // namespace nebula::drivers
