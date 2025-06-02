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

#include "nebula_decoders/nebula_decoders_common/angles.hpp"

#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_types.hpp>
#include <nebula_common/util/expected.hpp>
#include <nebula_common/util/string_conversions.hpp>

#include <boost/range/algorithm/fill.hpp>

#include <sys/types.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

namespace nebula::drivers::point_filters
{

/// @brief The blockage state of a single laser firing.
enum class BlockageState : uint8_t {
  /// The return has provably been blocked by a too-close object.
  BLOCKAGE = 0,
  /// The return has provably not been blocked by anything.
  NO_BLOCKAGE = 1,
  /// Neither blockage nor no-blockage can be proven.
  UNSURE = 2,
};

class BlockageMaskPlugin;

class BlockageMask
{
public:
  BlockageMask(
    AngleRange<int32_t, MilliDegrees> azimuth_range_mdeg, uint32_t bin_size_mdeg,
    uint16_t n_channels)
  : azimuth_range_rad_{  // a
    deg2rad(azimuth_range_mdeg.start / 1000.),  // a
    deg2rad(azimuth_range_mdeg.end / 1000.)},
    bin_size_rad_{deg2rad(bin_size_mdeg / 1000.)},
    n_channels_{n_channels}
  {
    mask_.resize(n_channels_ * get_width());
  }

  void update(double azimuth_rad, uint16_t channel, BlockageState blockage)
  {
    auto index = get_index(channel, azimuth_rad);
    if (!index.has_value()) {
      return;
    }

    // saturate instead of rolling over
    uint8_t bin_n_blocked = mask_[index.value()];
    if (bin_n_blocked < UINT8_MAX && blockage == BlockageState::BLOCKAGE) {
      bin_n_blocked++;
    }

    mask_[index.value()] = bin_n_blocked;
  }

  [[nodiscard]] const std::vector<uint8_t> & get_mask() const { return mask_; }

  [[nodiscard]] size_t get_width() const
  {
    return static_cast<size_t>(std::ceil(azimuth_range_rad_.extent() / bin_size_rad_));
  }

  [[nodiscard]] size_t get_height() const { return n_channels_; }

private:
  [[nodiscard]] util::expected<size_t, std::monostate> get_bin_index(double azimuth_rad) const
  {
    double azimuth_from_start = azimuth_rad - azimuth_range_rad_.start;
    azimuth_from_start = normalize_angle(azimuth_from_start, Radians::circle_modulus);

    auto index = static_cast<size_t>(std::floor(azimuth_from_start / bin_size_rad_));

    if (index >= get_width()) {
      return std::monostate{};
    }

    return index;
  }

  [[nodiscard]] util::expected<size_t, std::monostate> get_channel_index(uint16_t channel) const
  {
    if (channel >= n_channels_) {
      return std::monostate{};
    }
    return channel;
  }

  [[nodiscard]] util::expected<size_t, std::monostate> get_index(
    uint16_t channel, double azimuth_rad) const
  {
    auto channel_index = get_channel_index(channel);
    auto bin_index = get_bin_index(azimuth_rad);

    if (!channel_index.has_value() || !bin_index.has_value()) {
      return std::monostate{};
    }

    return (channel_index.value() * get_width()) + bin_index.value();
  }

  void reset() { boost::range::fill(mask_, 0); }

  AngleRange<double, Radians> azimuth_range_rad_;
  double bin_size_rad_;
  uint16_t n_channels_;

  std::vector<uint8_t> mask_;

  friend class BlockageMaskPlugin;
};

class BlockageMaskPlugin
{
public:
  using callback_t = std::function<void(const BlockageMask & blockage_mask, double timestamp_s)>;

  explicit BlockageMaskPlugin(uint32_t bin_width_mdeg) : bin_width_mdeg_(bin_width_mdeg) {}

  void set_callback(callback_t callback) { callback_ = std::move(callback); }

  /**
   * @brief Trigger the callback with the completed mask, and reset the mask
   *
   * @param mask The mask to pass to the callback and reset
   */
  void callback_and_reset(BlockageMask & mask, double timestamp_s)
  {
    if (callback_) {
      callback_(mask, timestamp_s);
    }

    mask.reset();
  }

  [[nodiscard]] uint32_t get_bin_width_mdeg() const { return bin_width_mdeg_; }

private:
  uint32_t bin_width_mdeg_;
  callback_t callback_;
};

}  // namespace nebula::drivers::point_filters
