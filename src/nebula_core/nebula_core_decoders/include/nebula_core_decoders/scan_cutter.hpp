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

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <ostream>
#include <variant>

namespace nebula::drivers
{

template <size_t NChannels, uint32_t AngleUnit>
class ScanCutter
{
public:
  static constexpr int32_t max_angle = 360 * AngleUnit;
  static constexpr uint8_t n_buffers = 2;
  using buffer_index_t = uint8_t;
  using publish_callback_t = std::function<void(buffer_index_t)>;
  using set_timestamp_callback_t = std::function<void(buffer_index_t)>;

private:
  struct State
  {
    int32_t last_azimuth_enc;
    buffer_index_t current_buffer_index;
    std::array<int32_t, NChannels> channel_last_azimuths;
    std::array<bool, NChannels> channels_in_fov;
    std::array<buffer_index_t, NChannels> channel_buffer_indices;
  };

  template <typename T>
  struct AllSame
  {
    T value;
  };

  struct Different
  {
  };

  using ChannelBufferState = std::variant<AllSame<buffer_index_t>, Different>;
  using ChannelFovState = std::variant<AllSame<bool>, Different>;

  int32_t cut_angle_out_;
  int32_t fov_start_out_;
  int32_t fov_end_out_;

  mutable std::ofstream debug_file_;

  publish_callback_t publish_callback_;
  set_timestamp_callback_t set_timestamp_callback_;

  std::optional<State> state_;

  [[nodiscard]] static buffer_index_t buffer_index_add(buffer_index_t buffer_index, int32_t offset)
  {
    return (static_cast<int32_t>(buffer_index) + offset) % n_buffers;
  }

  void debug_print(
    size_t step_count, bool is_case_1, bool is_case_2, bool is_case_3, bool should_publish,
    bool has_360_fov, bool cuts_at_fov_end, bool reset_timestamp_on_fov_start,
    bool was_out_of_fov_before, bool is_fully_in_fov_after, bool is_partially_in_fov_after,
    bool entered_fov, bool should_reset_timestamp) const
  {
    // return;
    static size_t print_count = 0;

    if (print_count == 0) {
      debug_file_ << "print_count,step_count,is_case_1,is_case_2,is_case_3,should_publish,"
                     "has_360_fov,cuts_at_fov_end,reset_timestamp_on_fov_start,"
                     "was_out_of_fov_before,is_fully_in_fov_after,is_partially_in_fov_after,"
                     "entered_fov,should_reset_timestamp,"
                     "cut_angle_out_,fov_start_out_,fov_end_out_,current_buffer_index,";
      for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
        debug_file_ << "channel_buffer_indices[" << channel_id << "],";
      }
      debug_file_ << "last_azimuth_enc,";
      for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
        debug_file_ << "channel_last_azimuths[" << channel_id << "],";
      }
      debug_file_ << "\n" << std::flush;
    }

    print_count++;

    debug_file_ << print_count << "," << step_count << "," << (is_case_1 ? "1" : "0") << ","
                << (is_case_2 ? "1" : "0") << "," << (is_case_3 ? "1" : "0") << ","
                << (should_publish ? "1" : "0") << "," << (has_360_fov ? "1" : "0") << ","
                << (cuts_at_fov_end ? "1" : "0") << ","
                << (reset_timestamp_on_fov_start ? "1" : "0") << ","
                << (was_out_of_fov_before ? "1" : "0") << "," << (is_fully_in_fov_after ? "1" : "0")
                << "," << (is_partially_in_fov_after ? "1" : "0") << ","
                << (entered_fov ? "1" : "0") << "," << (should_reset_timestamp ? "1" : "0") << ",";

    debug_file_ << cut_angle_out_ << ",";
    debug_file_ << fov_start_out_ << ",";
    debug_file_ << fov_end_out_ << ",";

    debug_file_ << (state_->current_buffer_index ? "1" : "0") << ",";

    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      buffer_index_t buffer_index = state_->channel_buffer_indices[channel_id];
      debug_file_ << (buffer_index ? "1" : "0") << ",";
    }

    debug_file_ << state_->last_azimuth_enc << ",";
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      debug_file_ << state_->channel_last_azimuths[channel_id] << ",";
    }

    debug_file_ << std::endl;
  }

  void initialize_state(
    int32_t block_azimuth_enc, const std::array<int32_t, NChannels> & channel_azimuths_out)
  {
    state_ = State{
      block_azimuth_enc, 0, channel_azimuths_out, {false}, {0},
    };

    bool block_intersects_cut = does_block_intersect_cut(channel_azimuths_out);

    if (!block_intersects_cut) {
      // All points are in the same scan, assign to buffer 0.
      state_->channel_buffer_indices.fill(0);
      set_timestamp_callback_(0);
      return;
    }

    // Block intersects cut, some points are in the current scan, some in the next.
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      int32_t channel_azimuth_out = normalize_angle(channel_azimuths_out[channel_id], max_angle);
      int32_t start_angle = cut_angle_out_;
      int32_t end_angle = normalize_angle(cut_angle_out_ + (max_angle / 2), max_angle);
      bool point_is_after_cut =
        angle_is_between(start_angle, end_angle, channel_azimuth_out, false, true);
      state_->channel_buffer_indices[channel_id] = point_is_after_cut ? 1 : 0;
      state_->channels_in_fov[channel_id] = is_point_inside_fov(channel_azimuth_out);
    }

    set_timestamp_callback_(0);
    set_timestamp_callback_(1);
  }

  [[nodiscard]] bool has_channel_crossed_cut(int32_t last_azimuth_out, int32_t azimuth_out) const
  {
    // Note: Cut itself is considered part of the previous scan.
    // last <= cut < current
    last_azimuth_out = normalize_angle(last_azimuth_out, max_angle);
    azimuth_out = normalize_angle(azimuth_out, max_angle);
    return angle_is_between(last_azimuth_out, azimuth_out, cut_angle_out_, true, false);
  }

  [[nodiscard]] ChannelBufferState get_channel_buffer_state() const
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

  [[nodiscard]] ChannelFovState get_channel_fov_state() const
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
  }

  [[nodiscard]] bool is_point_inside_fov(int32_t azimuth_out) const
  {
    // Both start and end are inclusive.
    // start <= azi <= end
    azimuth_out = normalize_angle(azimuth_out, max_angle);
    return angle_is_between(fov_start_out_, fov_end_out_, azimuth_out);
  }

  [[nodiscard]] bool does_block_intersect_fov(
    const std::array<int32_t, NChannels> & channel_azimuths_out) const
  {
    // start <= azi + cmin AND azi + cmax <= end
    // => start - cmin <= azi <= end - cmax
    return std::any_of(
      channel_azimuths_out.begin(), channel_azimuths_out.end(),
      [this](int32_t azimuth_out) { return is_point_inside_fov(azimuth_out); });
  }

  [[nodiscard]] bool does_block_intersect_cut(
    const std::array<int32_t, NChannels> & channel_azimuths_out) const
  {
    // Note: Cut itself is considered part of the previous scan.
    // azi + cmin <= cut < azi + cmax
    size_t n_points_beyond_cut = std::count_if(
      channel_azimuths_out.begin(), channel_azimuths_out.end(), [this](int32_t azimuth_out) {
        azimuth_out = normalize_angle(azimuth_out, max_angle);
        int32_t start_angle = cut_angle_out_;
        int32_t end_angle = normalize_angle(cut_angle_out_ + (max_angle / 2), max_angle);
        return angle_is_between(start_angle, end_angle, azimuth_out, false, true);
      });

    return n_points_beyond_cut != 0 && n_points_beyond_cut != NChannels;
  }

  const std::array<buffer_index_t, NChannels> & step(
    int32_t block_azimuth_enc, const std::array<int32_t, NChannels> & channel_azimuths_out)
  {
    static size_t step_count = 0;
    step_count++;

    if (!state_) {
      initialize_state(block_azimuth_enc, channel_azimuths_out);
    }

    ChannelBufferState buffer_state_before = get_channel_buffer_state();
    ChannelFovState fov_state_before = get_channel_fov_state();

    // Update channel buffer indices
    for (size_t channel_id = 0; channel_id < NChannels; ++channel_id) {
      int32_t channel_azimuth_out = channel_azimuths_out[channel_id];
      bool channel_in_fov = is_point_inside_fov(channel_azimuth_out);
      state_->channels_in_fov[channel_id] = channel_in_fov;

      if (has_channel_crossed_cut(state_->channel_last_azimuths[channel_id], channel_azimuth_out)) {
        state_->channel_buffer_indices[channel_id] =
          buffer_index_add(state_->channel_buffer_indices[channel_id], 1);
      }

      state_->channel_last_azimuths[channel_id] = channel_azimuth_out;
    }

    ChannelBufferState buffer_state_after = get_channel_buffer_state();
    ChannelFovState fov_state_after = get_channel_fov_state();

    // There are three distinct semantic cases to consider for cut transitions:
    // 1. Channels were all in the current scan, and some of them entered the next scan.
    //    | ch# | before | after |
    //    |  0  |   0    |   0   |
    //    |  1  |   0    |   1   |
    //    Action: set timestamp of the next buffer
    //
    // 2. Some channels were still in the current scan, and now all are in the next scan.
    //    | ch# | before | after |
    //    |  0  |   0    |   1   |
    //    |  1  |   1    |   1   |
    //    Action: publish the current buffer
    //
    // 3. Channels were all in the current scan, and all entered the next scan.
    //    | ch# | before | after |
    //    |  0  |   0    |   1   |
    //    |  1  |   0    |   1   |
    //    Action: publish the current buffer and set timestamp of the next buffer
    //
    // All other cases require no action:
    // - all channels were and are in the same scan
    // - channels are were not and are not all in the same scan

    bool is_case_1 =
      (std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_before) &&
       std::holds_alternative<Different>(buffer_state_after));

    bool is_case_2 =
      (std::holds_alternative<Different>(buffer_state_before) &&
       std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_after));

    bool is_case_3 = (std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_before) &&
                      std::holds_alternative<AllSame<buffer_index_t>>(buffer_state_after)) &&
                     std::get<AllSame<buffer_index_t>>(buffer_state_before).value !=
                       std::get<AllSame<buffer_index_t>>(buffer_state_after).value;

    bool should_publish = is_case_2 || is_case_3;

    // Publish buffer if channels were not all in the same scan, but are now.
    // In other words, we were in the cut region before, but are not anymore.
    if (should_publish) {
      publish_callback_(state_->current_buffer_index);
      state_->current_buffer_index = buffer_index_add(state_->current_buffer_index, 1);
    }

    bool has_360_fov = fov_start_out_ == fov_end_out_;
    bool cuts_at_fov_end = cut_angle_out_ == fov_end_out_;

    bool reset_timestamp_on_fov_start = !has_360_fov && cuts_at_fov_end;

    bool was_out_of_fov_before =
      (std::holds_alternative<AllSame<bool>>(fov_state_before) &&
       !static_cast<bool>(std::get<AllSame<bool>>(fov_state_before).value));

    bool is_fully_in_fov_after =
      (std::holds_alternative<AllSame<bool>>(fov_state_after) &&
       static_cast<bool>(std::get<AllSame<bool>>(fov_state_after).value));

    bool is_partially_in_fov_after = (std::holds_alternative<Different>(fov_state_after));

    bool entered_fov =
      (was_out_of_fov_before && (is_fully_in_fov_after || is_partially_in_fov_after));
    bool should_reset_timestamp = (reset_timestamp_on_fov_start && entered_fov) ||
                                  (!reset_timestamp_on_fov_start && (is_case_1 || is_case_3));

    // Set timestamp of next buffer if channels were in the same scan before, but are not anymore.
    // In other words, we were not in the cut region before, but are now.
    if (should_reset_timestamp) {
      int32_t buffer_index_to_set{};
      if (reset_timestamp_on_fov_start) {
        buffer_index_to_set = state_->current_buffer_index;
      } else {
        buffer_index_to_set = buffer_index_add(state_->current_buffer_index, 1);
      }

      set_timestamp_callback_(buffer_index_to_set);
    }

    debug_print(
      step_count, is_case_1, is_case_2, is_case_3, should_publish, has_360_fov, cuts_at_fov_end,
      reset_timestamp_on_fov_start, was_out_of_fov_before, is_fully_in_fov_after,
      is_partially_in_fov_after, entered_fov, should_reset_timestamp);

    state_->last_azimuth_enc = block_azimuth_enc;

    return state_->channel_buffer_indices;
  }
};

}  // namespace nebula::drivers
