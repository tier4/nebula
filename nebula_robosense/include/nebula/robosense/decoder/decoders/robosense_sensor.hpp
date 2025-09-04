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

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/angle_corrector_calibration_based.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT, typename InfoPacketT>
class RobosenseSensor
{
public:
  typedef PacketT packet_t;
  typedef InfoPacketT info_t;
  typedef class AngleCorrectorCalibrationBased<PacketT::n_channels, PacketT::degree_subdivisions>
    angle_corrector_t;

  RobosenseSensor() = default;
  virtual ~RobosenseSensor() = default;

  /// @brief Computes the exact relative time between the timestamp of the given packet and the one
  /// of the point identified by the given block and channel, in nanoseconds
  /// @param block_id The point's block id
  /// @param channel_id The point's channel id
  /// @param sensor_configuration The sensor configuration
  /// @return The relative time offset in nanoseconds
  virtual int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id,
    const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration) = 0;

  /// @brief For a given start block index, find the earliest (lowest) relative time offset of any
  /// point in the packet in or after the start block
  /// @param start_block_id The index of the block in and after which to consider points
  /// @param sensor_configuration The sensor configuration
  /// @return The lowest point time offset (relative to the packet timestamp) of any point in or
  /// after the start block, in nanoseconds
  int get_earliest_point_time_offset_for_block(
    uint32_t start_block_id,
    const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration)
  {
    const auto n_returns = robosense_packet::get_n_returns(sensor_configuration->return_mode);
    int min_offset_ns = std::numeric_limits<int>::max();

    for (uint32_t block_id = start_block_id; block_id < start_block_id + n_returns; ++block_id) {
      for (uint32_t channel_id = 0; channel_id < PacketT::n_channels; ++channel_id) {
        min_offset_ns = std::min(
          min_offset_ns,
          get_packet_relative_point_time_offset(block_id, channel_id, sensor_configuration));
      }
    }

    return min_offset_ns;
  }

  /// @brief Whether the unit given by return_idx is a duplicate of any other unit in return_units
  /// @param return_idx The unit's index in the return_units vector
  /// @param return_units The vector of all the units corresponding to the same return group (i.e.
  /// length 2 for dual-return with both units having the same channel but coming from different
  /// blocks)
  /// @return true if the unit is identical to any other one in return_units, false otherwise
  static bool is_duplicate(
    uint32_t return_idx,
    const std::vector<const typename PacketT::body_t::block_t::unit_t *> & return_units)
  {
    for (unsigned int i = 0; i < return_units.size(); ++i) {
      if (i == return_idx) {
        continue;
      }

      if (
        return_units[return_idx]->distance.value() == return_units[i]->distance.value() &&
        return_units[return_idx]->reflectivity.value() == return_units[i]->reflectivity.value()) {
        return true;
      }
    }

    return false;
  }

  /// @brief Get the return type of the point given by return_idx
  ///
  /// @param return_mode The sensor's currently active return mode
  /// @param return_idx The block index of the point within the group of blocks that make up the
  /// return group (e.g. either 0 or 1 for dual return)
  /// @param return_units The units corresponding to all the returns in the group. These are usually
  /// from the same column across adjascent blocks.
  /// @return The return type of the point
  virtual ReturnType get_return_type(
    ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const typename PacketT::body_t::block_t::unit_t *> & return_units)
  {
    if (is_duplicate(return_idx, return_units)) {
      return ReturnType::IDENTICAL;
    }

    switch (return_mode) {
      case ReturnMode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case ReturnMode::SINGLE_LAST:
        return ReturnType::LAST;
      case ReturnMode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;
      case ReturnMode::DUAL:
        if (return_idx == 0) {
          return ReturnType::STRONGEST;
        } else {
          return ReturnType::LAST;
        }
      default:
        return ReturnType::UNKNOWN;
    }
  }

  virtual ReturnMode get_return_mode(const info_t & info_packet) = 0;

  virtual RobosenseCalibrationConfiguration get_sensor_calibration(const info_t & info_packet) = 0;

  virtual bool get_sync_status(const info_t & info_packet) = 0;

  virtual std::map<std::string, std::string> get_sensor_info(const info_t & info_packet) = 0;
};
}  // namespace nebula::drivers
