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

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::drivers
{
template <typename SensorT>
class RobosenseDecoder : public RobosenseScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::RobosenseSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;
  /// @brief The last azimuth processed
  int last_phase_;
  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_;
  /// @brief Whether a full scan has been processed
  bool has_scanned_;

  rclcpp::Logger logger_;

  /// @brief Validates and parses MsopPacket. Currently only checks size, not checksums etc.
  /// @param msop_packet The incoming MsopPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & msop_packet)
  {
    if (msop_packet.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch: " << msop_packet.size() << " | Expected at least: "
                                          << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, msop_packet.data(), sizeof(typename SensorT::packet_t))) {
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group
  void convert_returns(size_t start_block_id, size_t n_blocks)
  {
    uint64_t packet_timestamp_ns = robosense_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units;

    for (size_t channel_id = 0; channel_id < SensorT::packet_t::n_channels; ++channel_id) {
      // Find the units corresponding to the same return group as the current one.
      // These are used to find duplicates in multi-return mode.
      return_units.clear();
      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        return_units.push_back(
          &packet_.body.blocks[block_offset + start_block_id].units[channel_id]);
      }

      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        auto & unit = *return_units[block_offset];

        if (unit.distance.value() == 0) {
          continue;
        }

        auto distance = get_distance(unit);

        if (distance < SensorT::min_range || distance > SensorT::max_range) {
          continue;
        }

        auto return_type =
          sensor_.get_return_type(sensor_configuration_->return_mode, block_offset, return_units);

        // Keep only last of multiple identical points
        if (return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
          continue;
        }

        // Keep only last (if any) of multiple points that are too close
        if (block_offset != n_blocks - 1) {
          bool is_below_multi_return_threshold = false;

          for (size_t return_idx = 0; return_idx < n_blocks; ++return_idx) {
            if (return_idx == block_offset) {
              continue;
            }

            if (
              fabsf(get_distance(*return_units[return_idx]) - distance) <
              sensor_configuration_->dual_return_distance_threshold) {
              is_below_multi_return_threshold = true;
              break;
            }
          }

          if (is_below_multi_return_threshold) {
            continue;
          }
        }

        NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity.value();
        point.time_stamp =
          get_point_time_relative(packet_timestamp_ns, block_offset + start_block_id, channel_id);

        point.return_type = static_cast<uint8_t>(return_type);

        auto corrected_angle_data =
          angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        point.channel = corrected_angle_data.corrected_channel_id;

        // The raw_azimuth and channel are only used as indices, sin/cos functions use the precise
        // corrected angles
        float xyDistance = distance * corrected_angle_data.cos_elevation;
        point.x = xyDistance * corrected_angle_data.cos_azimuth;
        point.y = -xyDistance * corrected_angle_data.sin_azimuth;
        point.z = distance * corrected_angle_data.sin_elevation;

        // The driver wrapper converts to degrees, expects radians
        point.azimuth = corrected_angle_data.azimuth_rad;
        point.elevation = corrected_angle_data.elevation_rad;

        decode_pc_->emplace_back(point);
      }
    }
  }

  /// @brief Checks whether the last processed block was the last block of a scan
  /// @param current_phase The azimuth of the last processed block
  /// @return Whether the scan has completed
  bool check_scan_completed(int current_phase)
  {
    return angle_corrector_.has_scanned(current_phase, last_phase_);
  }

  /// @brief Get the distance of the given unit in meters
  /// @param unit The unit to get the distance from
  /// @return The distance in meters
  float get_distance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance.value() * robosense_packet::get_dis_unit(packet_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param packet_timestamp_ns The timestamp of the current MsopPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t get_point_time_relative(uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns =
      sensor_.get_packet_relative_point_time_offset(block_id, channel_id, sensor_configuration_);
    auto packet_to_scan_offset_ns =
      static_cast<uint32_t>(packet_timestamp_ns - decode_scan_timestamp_ns_);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  /// calibration_configuration is set)
  explicit RobosenseDecoder(
    const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & calibration_configuration)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(calibration_configuration),
    logger_(rclcpp::get_logger("RobosenseDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, sensor_configuration_);

    decode_pc_.reset(new NebulaPointCloud);
    output_pc_.reset(new NebulaPointCloud);

    decode_pc_->reserve(SensorT::max_scan_buffer_points);
    output_pc_->reserve(SensorT::max_scan_buffer_points);
  }

  int unpack(const std::vector<uint8_t> & msop_packet) override
  {
    if (!parse_packet(msop_packet)) {
      return -1;
    }

    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = robosense_packet::get_timestamp_ns(packet_);
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    // For the dual return mode, the packet contains two blocks with the same azimuth, one for each
    // return. For the single return mode, the packet contains only one block per azimuth.
    // So, if the return mode is dual, we process two blocks per iteration, otherwise one.
    const size_t n_returns = robosense_packet::get_n_returns(sensor_configuration_->return_mode);
    int current_azimuth;

    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      current_azimuth =
        (360 * SensorT::packet_t::degree_subdivisions +
         packet_.body.blocks[block_id].get_azimuth() -
         static_cast<int>(
           sensor_configuration_->scan_phase * SensorT::packet_t::degree_subdivisions)) %
        (360 * SensorT::packet_t::degree_subdivisions);

      bool scan_completed = check_scan_completed(current_azimuth);
      if (scan_completed) {
        std::swap(decode_pc_, output_pc_);
        decode_pc_->clear();
        has_scanned_ = true;
        output_scan_timestamp_ns_ = decode_scan_timestamp_ns_;

        // A new scan starts within the current packet, so the new scan's timestamp must be
        // calculated as the packet timestamp plus the lowest time offset of any point in the
        // remainder of the packet
        decode_scan_timestamp_ns_ =
          robosense_packet::get_timestamp_ns(packet_) +
          sensor_.get_earliest_point_time_offset_for_block(block_id, sensor_configuration_);
      }

      convert_returns(block_id, n_returns);
      last_phase_ = current_azimuth;
    }

    return last_phase_;
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace nebula::drivers
