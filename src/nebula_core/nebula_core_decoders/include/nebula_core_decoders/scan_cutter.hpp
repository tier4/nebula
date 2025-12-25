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

#include "nebula_core_decoders/angles.hpp"
#include "nebula_core_decoders/scan_cutter/fsm_cut_at_fov_end.hpp"
#include "nebula_core_decoders/scan_cutter/fsm_cut_in_fov.hpp"
#include "nebula_core_decoders/scan_cutter/types.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <optional>
#include <variant>

namespace nebula::drivers
{

template <size_t NChannels>
struct CorrectedAzimuths
{
  std::array<int32_t, NChannels> azimuths;
  size_t min_correction_index;
  size_t max_correction_index;
};

template <size_t NChannels, uint32_t AngleUnit>
class ScanCutter
{
public:
  static constexpr int32_t max_angle = 360 * AngleUnit;
  static constexpr uint8_t n_buffers = 2;
  using buffer_index_t = scan_cutter::buffer_index_t;

  using publish_callback_t = std::function<void(buffer_index_t)>;
  using set_timestamp_callback_t = std::function<void(buffer_index_t)>;

  template <typename T>
  using AllSame = scan_cutter::AllSame<T>;
  using Different = scan_cutter::Different;

  using ChannelBufferState = scan_cutter::ChannelBufferState;
  using ChannelFovState = scan_cutter::ChannelFovState;
  using TransitionActions = scan_cutter::TransitionActions;

  struct State
  {
    /// The buffer being scheduled for emission after the next cut.
    buffer_index_t current_buffer_index{};
    /// The last processed azimuth for each channel.
    std::array<int32_t, NChannels> channel_last_azimuths;
    /// Whether each channel is in the FoV.
    std::array<bool, NChannels> channels_in_fov;
    /// The buffer index for each channel.
    std::array<buffer_index_t, NChannels> channel_buffer_indices;

    ChannelBufferState buffer_state;
    ChannelFovState fov_state;

    [[nodiscard]] bool does_block_intersect_cut() const
    {
      return std::holds_alternative<Different>(buffer_state);
    }

    [[nodiscard]] bool does_block_intersect_fov() const
    {
      if (std::holds_alternative<Different>(fov_state)) {
        return true;
      }

      assert(std::holds_alternative<AllSame<bool>>(fov_state));
      bool all_in_fov = std::get<AllSame<bool>>(fov_state).value;
      return all_in_fov;
    }
  };

private:
  int32_t cut_angle_out_;
  int32_t fov_start_out_;
  int32_t fov_end_out_;

  publish_callback_t publish_callback_;
  set_timestamp_callback_t set_timestamp_callback_;

  mutable std::ofstream debug_file_;

  std::optional<State> state_;

  [[nodiscard]] static buffer_index_t buffer_index_add(buffer_index_t buffer_index, int32_t offset)
  {
    return static_cast<buffer_index_t>((buffer_index + offset) % n_buffers);
  }

  void initialize_state(const CorrectedAzimuths<NChannels> & corrected_azimuths_out)
  {
    state_ = State{};
    state_->current_buffer_index = 0;
    state_->channel_last_azimuths = corrected_azimuths_out.azimuths;
    state_->channels_in_fov.fill(false);
    state_->channel_buffer_indices.fill(0);

    bool block_intersects_cut = compute_if_block_intersects_cut(corrected_azimuths_out);

    if (!block_intersects_cut) {
      // All points are in the same scan, assign to buffer 0.
      state_->channel_buffer_indices.fill(0);

      bool overlaps_fov = compute_if_block_intersects_fov(corrected_azimuths_out);
      if (overlaps_fov) {
        set_timestamp_callback_(0);
      }

      state_->buffer_state = compute_buffer_state();
      state_->fov_state = compute_fov_state();
      return;
    }

    // Block intersects cut, some points are in the current scan, some in the next.
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      int32_t channel_azimuth_out = corrected_azimuths_out.azimuths[channel_id];
      int32_t start_angle = cut_angle_out_;
      int32_t end_angle = normalize_angle(cut_angle_out_ + (max_angle / 2), max_angle);
      bool point_is_after_cut =
        angle_is_between(start_angle, end_angle, channel_azimuth_out, false, true);
      state_->channel_buffer_indices[channel_id] = point_is_after_cut ? 1 : 0;
      state_->channels_in_fov[channel_id] = is_point_inside_fov(channel_azimuth_out);
    }

    bool overlaps_fov_0 = false;
    bool overlaps_fov_1 = false;
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      if (!state_->channels_in_fov[channel_id]) {
        continue;
      }

      if (state_->channel_buffer_indices[channel_id] == 0) {
        overlaps_fov_0 = true;
      } else {
        overlaps_fov_1 = true;
      }
    }

    if (overlaps_fov_0) {
      set_timestamp_callback_(0);
    }
    if (overlaps_fov_1) {
      set_timestamp_callback_(1);
    }

    state_->buffer_state = compute_buffer_state();
    state_->fov_state = compute_fov_state();
  }

  [[nodiscard]] bool compute_if_block_intersects_cut(
    const CorrectedAzimuths<NChannels> & corrected_azimuths_out) const
  {
    // First, check if all channels are in the same hemisphere as the cut.
    int32_t cut_region_start = normalize_angle(cut_angle_out_ - (max_angle / 4), max_angle);
    int32_t cut_region_end = normalize_angle(cut_angle_out_ + (max_angle / 4), max_angle);

    const std::array<int32_t, NChannels> minmax_azimuths_out = {
      corrected_azimuths_out.azimuths[corrected_azimuths_out.min_correction_index],
      corrected_azimuths_out.azimuths[corrected_azimuths_out.max_correction_index]};

    if (!std::all_of(
          minmax_azimuths_out.begin(), minmax_azimuths_out.end(), [=](int32_t azimuth_out) {
            return angle_is_between(cut_region_start, cut_region_end, azimuth_out);
          })) {
      return false;
    }

    // Note: Cut itself is considered part of the previous scan.
    // azi + cmin <= cut < azi + cmax
    size_t n_points_beyond_cut = std::count_if(
      minmax_azimuths_out.begin(), minmax_azimuths_out.end(), [this](int32_t azimuth_out) {
        int32_t start_angle = cut_angle_out_;
        int32_t end_angle = normalize_angle(cut_angle_out_ + (max_angle / 2), max_angle);
        return angle_is_between(start_angle, end_angle, azimuth_out, false, true);
      });

    return n_points_beyond_cut != 0 && n_points_beyond_cut != NChannels;
  }

  [[nodiscard]] bool has_channel_crossed_cut(int32_t last_azimuth_out, int32_t azimuth_out) const
  {
    // Note: Cut itself is considered part of the previous scan.
    // last <= cut < current
    return angle_is_between(last_azimuth_out, azimuth_out, cut_angle_out_, true, false);
  }

  [[nodiscard]] ChannelBufferState compute_buffer_state() const
  {
    static_assert(NChannels > 0, "NChannels must be greater than 0");
    assert(state_);

    buffer_index_t first_buffer_index = state_->channel_buffer_indices[0];
    bool all_same = std::all_of(
      state_->channel_buffer_indices.begin(), state_->channel_buffer_indices.end(),
      [first_buffer_index](buffer_index_t buffer_index) {
        return buffer_index == first_buffer_index;
      });

    if (all_same) {
      return AllSame<buffer_index_t>{first_buffer_index};
    }

    return Different{};
  }

  [[nodiscard]] bool compute_if_block_intersects_fov(
    const CorrectedAzimuths<NChannels> & corrected_azimuths_out) const
  {
    // start <= azi + cmin AND azi + cmax <= end
    // => start - cmin <= azi <= end - cmax
    const std::array<int32_t, NChannels> minmax_azimuths_out = {
      corrected_azimuths_out.azimuths[corrected_azimuths_out.min_correction_index],
      corrected_azimuths_out.azimuths[corrected_azimuths_out.max_correction_index]};
    return std::any_of(
      minmax_azimuths_out.begin(), minmax_azimuths_out.end(),
      [this](int32_t azimuth_out) { return is_point_inside_fov(azimuth_out); });
  }

  [[nodiscard]] ChannelFovState compute_fov_state() const
  {
    static_assert(NChannels > 0, "NChannels must be greater than 0");
    assert(state_);

    bool first_point_inside_fov = state_->channels_in_fov[0];
    bool all_same = std::all_of(
      state_->channels_in_fov.begin(), state_->channels_in_fov.end(),
      [this, first_point_inside_fov](bool channel_in_fov) {
        return channel_in_fov == first_point_inside_fov;
      });

    if (all_same) {
      return AllSame<bool>{first_point_inside_fov};
    }

    return Different{};
  }

  [[nodiscard]] bool has_jumped_cut(
    const ChannelBufferState & buffer_state_before,
    const ChannelBufferState & buffer_state_after) const
  {
    // The cut has been jumped if:
    // 1. All channels were in the same buffer before (AllSame)
    // 2. All channels are in the same buffer after (AllSame)
    // 3. The buffer indices are different (jumped from one buffer to another)
    //
    // If these conditions are met, all channels must have crossed the cut angle simultaneously,
    // which means the azimuth step was large enough to skip the cut for all channels.

    if (!std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_before)) {
      return false;
    }

    if (!std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_after)) {
      return false;
    }

    buffer_index_t buffer_before = std::get<AllSame<buffer_index_t>>(buffer_state_before).value;
    buffer_index_t buffer_after = std::get<AllSame<buffer_index_t>>(buffer_state_after).value;

    return buffer_before != buffer_after;
  }

  [[nodiscard]] bool is_point_inside_fov(int32_t azimuth_out) const
  {
    // 360deg FoV
    if (fov_start_out_ == fov_end_out_) {
      return true;
    }

    // Both start and end are inclusive.
    // start <= azi <= end
    return angle_is_between(fov_start_out_, fov_end_out_, azimuth_out);
  }

  ChannelFovState aggregate_fov_state(const ChannelFovState & fov_state, bool channel_in_fov) const
  {
    const auto * all_same_opt = std::get_if<AllSame<bool>>(&fov_state);
    if (all_same_opt && all_same_opt->value == channel_in_fov) {
      return fov_state;
    }

    return Different{};
  }

  ChannelBufferState aggregate_buffer_state(
    const ChannelBufferState & buffer_state, buffer_index_t buffer_index) const
  {
    const auto * all_same_opt = std::get_if<AllSame<buffer_index_t>>(&buffer_state);
    if (all_same_opt && all_same_opt->value == buffer_index) {
      return buffer_state;
    }

    return Different{};
  }

  void update_state(const CorrectedAzimuths<NChannels> & corrected_azimuths_out)
  {
    assert(state_);

    ChannelFovState fov_state{};
    ChannelBufferState buffer_state{};

    const auto & az = corrected_azimuths_out.azimuths;
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      int32_t channel_azimuth_out = az[channel_id];
      bool channel_in_fov = is_point_inside_fov(channel_azimuth_out);
      state_->channels_in_fov[channel_id] = channel_in_fov;

      if (has_channel_crossed_cut(state_->channel_last_azimuths[channel_id], channel_azimuth_out)) {
        state_->channel_buffer_indices[channel_id] =
          buffer_index_add(state_->channel_buffer_indices[channel_id], 1);
      }

      state_->channel_last_azimuths[channel_id] = channel_azimuth_out;

      buffer_index_t buffer_index = state_->channel_buffer_indices[channel_id];
      if (channel_id == corrected_azimuths_out.min_correction_index) {
        fov_state = AllSame<bool>{channel_in_fov};
        buffer_state = AllSame<buffer_index_t>{buffer_index};
      } else if (channel_id == corrected_azimuths_out.max_correction_index) {
        fov_state = aggregate_fov_state(fov_state, channel_in_fov);
        buffer_state = aggregate_buffer_state(buffer_state, buffer_index);
      }
    }

    state_->buffer_state = buffer_state;
    state_->fov_state = fov_state;
  }

public:
  ScanCutter(
    int32_t cut_angle_out, int32_t fov_start_out, int32_t fov_end_out,
    publish_callback_t publish_callback, set_timestamp_callback_t set_timestamp_callback)
  : cut_angle_out_(normalize_angle(cut_angle_out, max_angle)),
    fov_start_out_(normalize_angle(fov_start_out, max_angle)),
    fov_end_out_(normalize_angle(fov_end_out, max_angle)),
    publish_callback_(std::move(publish_callback)),
    set_timestamp_callback_(std::move(set_timestamp_callback)),
    debug_file_("scan_cutter_debug.csv")
  {
    if (!publish_callback_) {
      throw std::invalid_argument("publish_callback cannot be nullptr");
    }
    if (!set_timestamp_callback_) {
      throw std::invalid_argument("set_timestamp_callback cannot be nullptr");
    }

    bool has_non_360_fov = fov_start_out_ != fov_end_out_;

    if (has_non_360_fov && !angle_is_between(fov_start_out_, fov_end_out_, cut_angle_out_)) {
      throw std::invalid_argument("Cut angle must be within FoV");
    }

    if (has_non_360_fov && cut_angle_out_ == fov_start_out_) {
      throw std::invalid_argument("Cut angle cannot coincide with FoV start");
    }
  }

  const State & step(const CorrectedAzimuths<NChannels> & corrected_azimuths_out)
  {
    if (!state_) {
      initialize_state(corrected_azimuths_out);
      return *state_;
    }

    assert(state_);

    // Determine operating mode
    bool has_360_fov = fov_start_out_ == fov_end_out_;
    bool cuts_at_fov_end = cut_angle_out_ == fov_end_out_ && !has_360_fov;

    // Capture state before update
    ChannelBufferState buffer_state_before = state_->buffer_state;
    ChannelFovState fov_state_before = state_->fov_state;

    // Update channel buffer indices and FoV status
    update_state(corrected_azimuths_out);

    // Capture state after update
    ChannelBufferState buffer_state_after = state_->buffer_state;
    ChannelFovState fov_state_after = state_->fov_state;

    // Step the appropriate state machine based on operating mode
    TransitionActions actions{};

    if (cuts_at_fov_end) {
      // Use the complex 8-state FSM for cut at FoV end
      actions = FsmCutAtFovEnd::step(
        buffer_state_before, buffer_state_after, fov_state_before, fov_state_after,
        state_->current_buffer_index);
    } else {
      // Use the simple 4-state FSM for cut in FoV (including 360° FoV)
      actions =
        FsmCutInFov::step(buffer_state_before, buffer_state_after, state_->current_buffer_index);
    }

    if (actions.reset_timestamp_buffer) {
      buffer_index_t buf = *actions.reset_timestamp_buffer;
      set_timestamp_callback_(buf);
    }

    if (actions.emit_scan_buffer) {
      buffer_index_t buf = *actions.emit_scan_buffer;
      publish_callback_(buf);
      // Reset the timestamp tracking for the published buffer so it can be set again
      state_->current_buffer_index = buffer_index_add(state_->current_buffer_index, 1);
    }

    return *state_;
  }
};

}  // namespace nebula::drivers
