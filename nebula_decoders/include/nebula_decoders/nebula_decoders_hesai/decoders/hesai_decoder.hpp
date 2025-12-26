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
#include "nebula_decoders/nebula_decoders_common/point_filters/downsample_mask.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
  struct ScanCutAngles
  {
    float fov_min;
    float fov_max;
    float scan_emit_angle;
  };

private:
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

  ScanCutAngles scan_cut_angles_;
  uint32_t last_azimuth_ = 0;

  std::shared_ptr<loggers::Logger> logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::n_channels> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::n_blocks>, SensorT::packet_t::max_returns>
    block_firing_offset_ns_;

  std::optional<point_filters::DownsampleMaskFilter> mask_filter_;

  // ============ PERFORMANCE OPTIMIZATION: Pre-allocated buffers ============
  /// @brief Pre-allocated return unit pointers to avoid heap allocation per channel
  std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units_buf_;
  /// @brief Pre-computed distances for current return group
  std::array<float, SensorT::packet_t::max_returns> distances_buf_;
  // =========================================================================

  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      NEBULA_LOG_STREAM(
        logger_->error, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                                 << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      // FIXME(mojomex) do validation?
      // RCLCPP_DEBUG(logger_, "Packet parsed successfully");
      return true;
    }

    logger_->error("Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convert_returns(size_t start_block_id, size_t n_blocks)
  {
    // ====== OPTIMIZATION: Cache packet-level values once ======
    const uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    const uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();
    const float dis_unit = hesai_packet::get_dis_unit(packet_);  // Cache once per call
    const auto return_mode =
      static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode);
    const float dual_return_threshold = sensor_configuration_->dual_return_distance_threshold;
    const float cfg_min_range = sensor_configuration_->min_range;
    const float cfg_max_range = sensor_configuration_->max_range;
    const bool is_inside_overlap = angle_corrector_.is_inside_overlap(last_azimuth_, raw_azimuth);
    const bool has_mask_filter = mask_filter_.has_value();

    // Pre-fetch block pointers for this return group
    const auto * const blocks = &packet_.body.blocks[start_block_id];

    for (size_t channel_id = 0; channel_id < SensorT::packet_t::n_channels; ++channel_id) {
      // ====== OPTIMIZATION: Use pre-allocated vector (resize is O(1) when capacity sufficient) ======
      return_units_buf_.resize(n_blocks);
      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        return_units_buf_[block_offset] = &blocks[block_offset].units[channel_id];
        // Pre-compute distance to avoid redundant calculations in inner loop
        distances_buf_[block_offset] =
          return_units_buf_[block_offset]->distance * dis_unit;
      }

      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        const auto & unit = *return_units_buf_[block_offset];

        // Use pre-computed distance
        const float distance = distances_buf_[block_offset];

        // Early exit for zero distance (common case)
        if (__builtin_expect(unit.distance == 0, 0)) {
          continue;
        }

        // ====== OPTIMIZATION: Combined range check with likely/unlikely hints ======
        if (__builtin_expect(
              distance < SensorT::min_range || distance > SensorT::max_range ||
                distance < cfg_min_range || distance > cfg_max_range,
              0)) {
          continue;
        }

        // Use pre-allocated vector (no heap allocation due to reserve in constructor)
        const auto return_type = sensor_.get_return_type(
          return_mode, block_offset, return_units_buf_);

        // Keep only last of multiple identical points
        if (return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
          continue;
        }

        // ====== OPTIMIZATION: Use pre-computed distances for threshold check ======
        if (block_offset != n_blocks - 1) {
          bool is_below_multi_return_threshold = false;

          for (size_t return_idx = 0; return_idx < n_blocks; ++return_idx) {
            if (return_idx == block_offset) {
              continue;
            }
            // Use pre-computed distance instead of calling get_distance()
            if (fabsf(distances_buf_[return_idx] - distance) < dual_return_threshold) {
              is_below_multi_return_threshold = true;
              break;
            }
          }

          if (is_below_multi_return_threshold) {
            continue;
          }
        }

        const CorrectedAngleData corrected_angle_data =
          angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        const float azimuth = corrected_angle_data.azimuth_rad;

        if (!angle_is_between(scan_cut_angles_.fov_min, scan_cut_angles_.fov_max, azimuth)) {
          continue;
        }

        bool in_current_scan = true;
        if (
          is_inside_overlap &&
          angle_is_between(
            scan_cut_angles_.scan_emit_angle, scan_cut_angles_.scan_emit_angle + deg2rad(20),
            azimuth)) {
          in_current_scan = false;
        }

        const auto pc = in_current_scan ? decode_pc_ : output_pc_;
        const uint64_t scan_timestamp_ns =
          in_current_scan ? decode_scan_timestamp_ns_ : output_scan_timestamp_ns_;

        // ====== OPTIMIZATION: Direct member assignment ======
        NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity;
        point.time_stamp = get_point_time_relative(
          scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);
        point.return_type = static_cast<uint8_t>(return_type);
        point.channel = channel_id;

        const float xy_distance = distance * corrected_angle_data.cos_elevation;
        point.x = xy_distance * corrected_angle_data.sin_azimuth;
        point.y = xy_distance * corrected_angle_data.cos_azimuth;
        point.z = distance * corrected_angle_data.sin_elevation;
        point.azimuth = corrected_angle_data.azimuth_rad;
        point.elevation = corrected_angle_data.elevation_rad;

        if (!has_mask_filter || !mask_filter_->excluded(point)) {
          pc->emplace_back(point);
        }
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  float get_distance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * hesai_packet::get_dis_unit(packet_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t get_point_time_relative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns =
      sensor_.get_packet_relative_point_time_offset(block_id, channel_id, packet_);
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data,
    const std::shared_ptr<loggers::Logger> & logger)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(
      correction_data, sensor_configuration_->cloud_min_angle,
      sensor_configuration_->cloud_max_angle, sensor_configuration_->cut_angle),
    scan_cut_angles_(
      {deg2rad(sensor_configuration_->cloud_min_angle),
       deg2rad(sensor_configuration_->cloud_max_angle), deg2rad(sensor_configuration_->cut_angle)}),
    logger_(logger)
  {
    decode_pc_ = std::make_shared<NebulaPointCloud>();
    output_pc_ = std::make_shared<NebulaPointCloud>();

    // Pre-allocate return units buffer to avoid heap allocation during decoding
    return_units_buf_.reserve(SensorT::packet_t::max_returns);

    if (sensor_configuration->downsample_mask_path) {
      mask_filter_ = point_filters::DownsampleMaskFilter(
        sensor_configuration->downsample_mask_path.value(), SensorT::fov_mdeg.azimuth,
        SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels,
        logger_->child("Downsample Mask"), true, sensor_.get_dither_transform());
    }

    decode_pc_->reserve(SensorT::max_scan_buffer_points);
    output_pc_->reserve(SensorT::max_scan_buffer_points);
  }

  int unpack(const std::vector<uint8_t> & packet) override
  {
    if (!parse_packet(packet)) {
      return -1;
    }

    // This is the first scan, set scan timestamp to whatever packet arrived first
    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_) +
                                  sensor_.get_earliest_point_time_offset_for_block(0, packet_);
    }

    if (has_scanned_) {
      output_pc_->clear();
      has_scanned_ = false;
    }

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();

      if (angle_corrector_.passed_timestamp_reset_angle(last_azimuth_, block_azimuth)) {
        uint64_t new_scan_timestamp_ns =
          hesai_packet::get_timestamp_ns(packet_) +
          sensor_.get_earliest_point_time_offset_for_block(block_id, packet_);

        if (sensor_configuration_->cut_angle == sensor_configuration_->cloud_max_angle) {
          // In the non-360 deg case, if the cut angle and FoV end coincide, the old pointcloud has
          // already been swapped and published before the timestamp reset angle is reached. Thus,
          // the `decode` pointcloud is now empty and will be decoded to. Reset its timestamp.
          decode_scan_timestamp_ns_ = new_scan_timestamp_ns;
          decode_pc_->clear();
        } else {
          /// When not cutting at the end of the FoV (i.e. the FoV is 360 deg or a cut occurs
          /// somewhere within a non-360 deg FoV), the current scan is still being decoded to the
          /// `decode` pointcloud but at the same time, points for the next pointcloud are arriving
          /// and will be decoded to the `output` pointcloud (please forgive the naming for now).
          /// Thus, reset the output pointcloud's timestamp.
          output_scan_timestamp_ns_ = new_scan_timestamp_ns;
        }
      }

      if (!angle_corrector_.is_inside_fov(last_azimuth_, block_azimuth)) {
        last_azimuth_ = block_azimuth;
        continue;
      }

      convert_returns(block_id, n_returns);

      if (angle_corrector_.passed_emit_angle(last_azimuth_, block_azimuth)) {
        // The current `decode` pointcloud is ready for publishing, swap buffers to continue with
        // the last `output` pointcloud as the `decode pointcloud.
        std::swap(decode_pc_, output_pc_);
        std::swap(decode_scan_timestamp_ns_, output_scan_timestamp_ns_);
        has_scanned_ = true;
      }

      last_azimuth_ = block_azimuth;
    }

    return last_azimuth_;
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace nebula::drivers
