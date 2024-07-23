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

#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/scan_cutter.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula
{
namespace drivers
{

template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

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

  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_ = 0;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_ = 0;
  /// @brief Whether a full scan has been processed
  bool has_scanned_ = false;

  ScanCutter<SensorT::packet_t::N_CHANNELS, typename SensorT::angle_corrector_t> scan_cutter_;

  rclcpp::Logger logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::N_CHANNELS> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::N_BLOCKS>, SensorT::packet_t::MAX_RETURNS>
    block_firing_offset_ns_;

  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                          << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      // FIXME(mojomex) do validation?
      // RCLCPP_DEBUG(logger_, "Packet parsed successfully");
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convertReturns(size_t start_block_id, size_t n_blocks)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units;

    for (size_t channel_id = 0; channel_id < SensorT::packet_t::N_CHANNELS; ++channel_id) {
      // Find the units corresponding to the same return group as the current one.
      // These are used to find duplicates in multi-return mode.
      return_units.clear();
      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        return_units.push_back(
          &packet_.body.blocks[block_offset + start_block_id].units[channel_id]);
      }

      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        auto & unit = *return_units[block_offset];

        // if (unit.distance == 0) {
        //   continue;
        // }

        float distance = 6.0f + block_offset;  // getDistance(unit);

        if (
          distance < SensorT::MIN_RANGE || SensorT::MAX_RANGE < distance ||
          distance < sensor_configuration_->min_range ||
          sensor_configuration_->max_range < distance) {
          // continue;
        }

        auto return_type = sensor_.getReturnType(
          static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
          block_offset, return_units);

        // Keep only last of multiple identical points
        if (
          /* return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1 */ block_offset !=
          0) {
          continue;
        }

        // Keep only last (if any) of multiple points that are too close
        // if (block_offset != n_blocks - 1) {
        //   bool is_below_multi_return_threshold = false;

        //   for (size_t return_idx = 0; return_idx < n_blocks; ++return_idx) {
        //     if (return_idx == block_offset) {
        //       continue;
        //     }

        //     if (
        //       fabsf(getDistance(*return_units[return_idx]) - distance) <
        //       sensor_configuration_->dual_return_distance_threshold) {
        //       is_below_multi_return_threshold = true;
        //       break;
        //     }
        //   }

        //   if (is_below_multi_return_threshold) {
        //     continue;
        //   }
        // }

        if (!angle_corrector_.isChannelInFov(raw_azimuth, channel_id)) {
          continue;
        }

        bool in_current_scan = scan_cutter_.isInCurrentScan(channel_id);

        auto pc = in_current_scan ? decode_pc_ : output_pc_;
        auto scan_timestamp_ns =
          in_current_scan ? decode_scan_timestamp_ns_ : output_scan_timestamp_ns_;

        CorrectedAngleData corrected_angle_data =
          angle_corrector_.getCorrectedAngleData(raw_azimuth, channel_id);

        NebulaPoint & point = pc->emplace_back();
        point.distance = distance;
        point.intensity = in_current_scan;
        point.time_stamp = getPointTimeRelative(
          scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);

        point.return_type = static_cast<uint8_t>(return_type);
        point.channel = channel_id;

        // The raw_azimuth and channel are only used as indices, sin/cos functions use the precise
        // corrected angles
        float xy_distance = distance * corrected_angle_data.cos_elevation;
        point.x = xy_distance * corrected_angle_data.sin_azimuth;
        point.y = xy_distance * corrected_angle_data.cos_azimuth;
        point.z = distance * corrected_angle_data.sin_elevation;

        // The driver wrapper converts to degrees, expects radians
        point.azimuth = corrected_angle_data.azimuth_rad;
        point.elevation = corrected_angle_data.elevation_rad;
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  float getDistance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * hesai_packet::get_dis_unit(packet_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t getPointTimeRelative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns =
      sensor_.getPacketRelativePointTimeOffset(block_id, channel_id, packet_);
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

  /**
   * @brief Return the appropriate scan cut azimuth based on whether the configured FoV is 360
   * degrees (use scan phase for cutting) or less (use max angle for cutting).
   *
   * @return uint32_t The scan cut azimuth in 10ths of degrees
   */
  [[nodiscard]] std::pair<float, float> getFovAnglesRad() const
  {
    double start_angle{};
    double end_angle{};

    if (
      sensor_configuration_->cloud_min_angle == 0 &&
      sensor_configuration_->cloud_max_angle == 360) {
      auto angle_deg = sensor_configuration_->cut_angle;
      start_angle = end_angle = angle_deg;
    } else {
      start_angle = sensor_configuration_->cloud_min_angle;
      end_angle = sensor_configuration_->cloud_max_angle;
    }

    return std::make_pair(deg2rad(start_angle), deg2rad(end_angle));
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(correction_data, getFovAnglesRad().first, getFovAnglesRad().second),
    scan_cutter_(angle_corrector_),
    logger_(rclcpp::get_logger("HesaiDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, *sensor_configuration_);

    decode_pc_ = std::make_shared<NebulaPointCloud>();
    output_pc_ = std::make_shared<NebulaPointCloud>();

    decode_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
    output_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
  }

  int unpack(const std::vector<uint8_t> & packet) override
  {
    static int starts = 0;
    static int ends = 0;
    if (!parsePacket(packet)) {
      return -1;
    }

    // This is the first scan, set scan timestamp to whatever packet arrived first
    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_) +
                                  sensor_.getEarliestPointTimeOffsetForBlock(0, packet_);
    }

    if (has_scanned_) {
      output_pc_->clear();
      has_scanned_ = false;
    }

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::N_BLOCKS; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();
      convertReturns(block_id, n_returns);

      auto cut_events = scan_cutter_.update(block_azimuth);

      if (cut_events.cloud_ended) {
        RCLCPP_INFO(logger_, "[%5d]   END on: %7.3f", ++ends, block_azimuth / 100.);
        std::swap(decode_pc_, output_pc_);
        std::swap(decode_scan_timestamp_ns_, output_scan_timestamp_ns_);
        has_scanned_ = true;
      }

      if (cut_events.cloud_started) {
        RCLCPP_INFO(logger_, "[%5d] START on: %7.3f", ++starts, block_azimuth / 100.);
        decode_scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_) +
                                    sensor_.getEarliestPointTimeOffsetForBlock(block_id, packet_);
      }
    }

    return 0;  // todo
  }

  bool hasScanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace drivers
}  // namespace nebula
