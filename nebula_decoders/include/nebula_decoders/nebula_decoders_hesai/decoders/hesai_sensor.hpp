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

#include "nebula_decoders/nebula_decoders_common/point_filters/downsample_mask.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_calibration_based.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_correction_based.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"

#include <nebula_common/nebula_common.hpp>

#include <algorithm>
#include <type_traits>
#include <vector>

namespace nebula::drivers
{

enum class AngleCorrectionType { CALIBRATION, CORRECTION };

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT, AngleCorrectionType AngleCorrection = AngleCorrectionType::CALIBRATION>
class HesaiSensor
{
private:
  /// @brief Whether the unit given by return_idx is the strongest (in terms of intensity) among all
  /// return_units
  /// @param return_idx The index of the unit in return_units
  /// @param return_units The vector of all the units corresponding to the same return group (i.e.
  /// length 2 for dual-return with both units having the same channel but coming from different
  /// blocks)
  /// @return true if the reflectivity of the unit is strictly greater than that of all other units
  /// in return_units, false otherwise
  static bool is_strongest_return(
    uint32_t return_idx,
    const std::vector<const typename PacketT::body_t::block_t::unit_t *> & return_units)
  {
    for (unsigned int i = 0; i < return_units.size(); ++i) {
      if (i == return_idx) {
        continue;
      }

      if (return_units[return_idx]->reflectivity < return_units[i]->reflectivity) {
        return false;
      }
    }

    return true;
  };

  /// @brief Whether the unit given by return_idx is a duplicate of any other unit in return_units
  /// @param return_idx The unit's index in the return_units vector
  /// @param return_units The vector of all the units corresponding to the same return group (i.e.
  /// length 2 for dual-return with both units having the same channel but coming from different
  /// blocks)
  /// @return true if the unit is identical to any other one in return_units, false otherwise
  static bool is_duplicate_return(
    uint32_t return_idx,
    const std::vector<const typename PacketT::body_t::block_t::unit_t *> & return_units)
  {
    for (unsigned int i = 0; i < return_units.size(); ++i) {
      if (i == return_idx) {
        continue;
      }

      if (
        return_units[return_idx]->distance == return_units[i]->distance &&
        return_units[return_idx]->reflectivity == return_units[i]->reflectivity) {
        return true;
      }
    }

    return false;
  };

public:
  using packet_t = PacketT;
  using angle_corrector_t = typename std::conditional<
    (AngleCorrection == AngleCorrectionType::CALIBRATION),
    AngleCorrectorCalibrationBased<PacketT::n_channels, PacketT::degree_subdivisions>,
    AngleCorrectorCorrectionBased<PacketT::n_channels, PacketT::degree_subdivisions>>::type;

  HesaiSensor() = default;
  virtual ~HesaiSensor() = default;

  /// @brief Computes the exact relative time between the timestamp of the given packet and the one
  /// of the point identified by the given block and channel, in nanoseconds
  /// @param block_id The point's block id
  /// @param channel_id The point's channel id
  /// @param packet The packet
  /// @return The relative time offset in nanoseconds
  virtual int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const PacketT & packet) = 0;

  /// @brief For a given start block index, find the earliest (lowest) relative time offset of any
  /// point in the packet in or after the start block
  /// @param start_block_id The index of the block in and after which to consider points
  /// @param packet The packet
  /// @return The lowest point time offset (relative to the packet timestamp) of any point in or
  /// after the start block, in nanoseconds
  int get_earliest_point_time_offset_for_block(uint32_t start_block_id, const PacketT & packet)
  {
    unsigned int n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int min_offset_ns = 0x7FFFFFFF;  // MAXINT (max. positive value)

    for (uint32_t block_id = start_block_id; block_id < start_block_id + n_returns; ++block_id) {
      for (uint32_t channel_id = 0; channel_id < PacketT::n_channels; ++channel_id) {
        min_offset_ns = std::min(
          min_offset_ns, get_packet_relative_point_time_offset(block_id, channel_id, packet));
      }
    }

    return min_offset_ns;
  }

  /// @brief Get the return type of the point given by return_idx
  ///
  /// For duplicate points, the return type is reported as @ref ReturnType::IDENTICAL for all
  /// identical points. For DUAL_LAST_STRONGEST and DUAL_FIRST_STRONGEST, if the last/first point is
  /// also the strongest, it will be returned as @ref ReturnType::LAST_STRONGEST / @ref
  /// ReturnType::FIRST_STRONGEST respectively, with the second point being reported as @ref
  /// ReturnType::SECONDSTRONGEST.
  ///
  /// @param return_mode The sensor's currently active return mode
  /// @param return_idx The block index of the point within the group of blocks that make up the
  /// return group (e.g. either 0 or 1 for dual return)
  /// @param return_units The units corresponding to all the returns in the group. These are usually
  /// from the same column across adjascent blocks.
  /// @return The return type of the point
  virtual ReturnType get_return_type(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const typename PacketT::body_t::block_t::unit_t *> & return_units)
  {
    if (is_duplicate_return(return_idx, return_units)) {
      return ReturnType::IDENTICAL;
    }

    switch (return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_SECOND:
        return ReturnType::SECOND;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;
      case hesai_packet::return_mode::SINGLE_LAST:
        return ReturnType::LAST;
      case hesai_packet::return_mode::DUAL_LAST_STRONGEST:
        if (is_strongest_return(return_idx, return_units)) {
          return return_idx == 0 ? ReturnType::LAST_STRONGEST : ReturnType::STRONGEST;
        } else {
          return return_idx == 0 ? ReturnType::LAST : ReturnType::SECONDSTRONGEST;
        }
      case hesai_packet::return_mode::DUAL_FIRST_SECOND:
        return return_idx == 0 ? ReturnType::FIRST : ReturnType::SECOND;
      case hesai_packet::return_mode::DUAL_FIRST_LAST:
        return return_idx == 0 ? ReturnType::FIRST : ReturnType::LAST;
      case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
        if (is_strongest_return(return_idx, return_units)) {
          return return_idx == 0 ? ReturnType::FIRST_STRONGEST : ReturnType::STRONGEST;
        } else {
          return return_idx == 0 ? ReturnType::FIRST : ReturnType::SECONDSTRONGEST;
        }
      case hesai_packet::return_mode::DUAL_STRONGEST_SECONDSTRONGEST:
        return return_idx == 0 ? ReturnType::STRONGEST : ReturnType::SECONDSTRONGEST;
      case hesai_packet::return_mode::TRIPLE_FIRST_LAST_STRONGEST:
        switch (return_idx) {
          case 0:
            return ReturnType::FIRST;
          case 1:
            return ReturnType::LAST;
          case 2:
            return ReturnType::STRONGEST;
          default:
            return ReturnType::UNKNOWN;
        }
      default:
        return ReturnType::UNKNOWN;
    }
  }

  [[nodiscard]] virtual point_filters::DitherTransform get_dither_transform() const
  {
    return point_filters::g_default_dither_transform;
  }
};

}  // namespace nebula::drivers
