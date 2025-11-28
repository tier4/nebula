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
#include "nebula_decoders/nebula_decoders_common/point_filters/blockage_mask.hpp"
#include "nebula_decoders/nebula_decoders_common/point_filters/downsample_mask.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/packet_loss_detector.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_types.hpp>
#include <nebula_common/util/stopwatch.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
  static constexpr size_t n_channels = SensorT::packet_t::n_channels;

private:
  struct ScanCutAngles
  {
    float fov_min;
    float fov_max;
    float scan_emit_angle;
  };

  struct DecodeFrame
  {
    NebulaPointCloudPtr pointcloud;
    uint64_t scan_timestamp_ns{0};
    std::optional<point_filters::BlockageMask> blockage_mask;
  };

  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief A function that is called on each decoded pointcloud frame
  pointcloud_callback_t pointcloud_callback_;

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief Decodes functional safety data for supported sensors
  std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>
    functional_safety_decoder_;

  std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> packet_loss_detector_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;

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

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin_;

  /// @brief Decoded data of the frame currently being decoded to
  DecodeFrame decode_frame_;
  /// @brief Decoded data of the frame currently being output
  DecodeFrame output_frame_;

  /// @brief Validates and parse PandarPacket. Checks size and, if present, CRC checksums.
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

    if (!std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      logger_->error("Packet memcopy failed");
      return false;
    }

    return true;
  }

  static size_t idx_at(size_t block_offset, size_t channel_id)
  {
    return block_offset * n_channels + channel_id;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  template <size_t n_blocks>
  void convert_returns(size_t start_block_id)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::array<uint16_t, n_channels * n_blocks> distances;
    std::array<uint8_t, n_channels * n_blocks> intensities;
    std::array<bool, n_channels * n_blocks> valid_mask;

    for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
      for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
        auto & unit = packet_.body.blocks[block_offset + start_block_id].units[channel_id];
        distances[idx_at(block_offset, channel_id)] = unit.distance;
        intensities[idx_at(block_offset, channel_id)] = unit.reflectivity;
        valid_mask[idx_at(block_offset, channel_id)] = unit.distance != 0;
      }
    }

    std::array<float, n_channels * n_blocks> distances_m;
    for (size_t idx = 0; idx < distances.size(); ++idx) {
      distances_m[idx] = get_distance(distances[idx]);
      valid_mask[idx] = distances_m[idx] >= SensorT::min_range &&
                        distances_m[idx] <= SensorT::max_range &&
                        distances_m[idx] >= sensor_configuration_->min_range &&
                        distances_m[idx] <= sensor_configuration_->max_range;
    }

    std::array<ReturnType, n_channels * n_blocks> return_types;
    auto return_mode = static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode);
    for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
      if constexpr (n_blocks == 1) {
        return_types[channel_id] = sensor_.get_return_type_single(return_mode);
      } else if constexpr (n_blocks == 2) {
        size_t idx0 = idx_at(0, channel_id);
        size_t idx1 = idx_at(1, channel_id);
        auto [first, second] = sensor_.get_return_type_dual(
          return_mode,                                                   //
          std::array<uint8_t, 2>{intensities[idx0], intensities[idx1]},  //
          std::array<uint16_t, 2>{distances[idx0], distances[idx1]});
        return_types[idx0] = first;
        return_types[idx1] = second;
      } else if constexpr (n_blocks == 3) {
        size_t idx0 = idx_at(0, channel_id);
        size_t idx1 = idx_at(1, channel_id);
        size_t idx2 = idx_at(2, channel_id);
        auto [first, second, third] = sensor_.get_return_type_triple(
          return_mode,  //
          std::array<uint8_t, 3>{intensities[idx0], intensities[idx1], intensities[idx2]},
          std::array<uint16_t, 3>{distances[idx0], distances[idx1], distances[idx2]});
        return_types[idx0] = first;
        return_types[idx1] = second;
        return_types[idx2] = third;
      }
    }

    if constexpr (n_blocks > 1) {
      for (size_t block_offset = 0; block_offset < n_blocks - 1; ++block_offset) {
        for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
          size_t idx = idx_at(block_offset, channel_id);
          size_t idx_last = idx_at(n_blocks - 1, channel_id);
          bool is_duplicate = return_types[idx] == ReturnType::IDENTICAL;
          bool is_too_close = distances_m[idx] - distances_m[idx_last] <
                              sensor_configuration_->dual_return_distance_threshold;
          valid_mask[idx] &= !is_duplicate && !is_too_close;
        }
      }
    }

    std::array<float, n_channels> azimuth_rad;
    std::array<float, n_channels> elevation_rad;
    std::array<float, n_channels> sin_azimuth;
    std::array<float, n_channels> cos_azimuth;
    std::array<float, n_channels> sin_elevation;
    std::array<float, n_channels> cos_elevation;

    for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
      CorrectedAngleData corrected_angle_data =
        angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
      azimuth_rad[channel_id] = corrected_angle_data.azimuth_rad;
      elevation_rad[channel_id] = corrected_angle_data.elevation_rad;
      sin_azimuth[channel_id] = corrected_angle_data.sin_azimuth;
      cos_azimuth[channel_id] = corrected_angle_data.cos_azimuth;
      sin_elevation[channel_id] = corrected_angle_data.sin_elevation;
      cos_elevation[channel_id] = corrected_angle_data.cos_elevation;
    }

    for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
      bool in_fov = angle_is_between(
        scan_cut_angles_.fov_min, scan_cut_angles_.fov_max, azimuth_rad[channel_id]);
      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        valid_mask[idx_at(block_offset, channel_id)] &= in_fov;
      }
    }

    std::array<bool, n_channels> in_current_scan;
    for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
      in_current_scan[channel_id] =
        angle_corrector_.is_inside_overlap(last_azimuth_, raw_azimuth) &&
        angle_is_between(
          scan_cut_angles_.scan_emit_angle, scan_cut_angles_.scan_emit_angle + deg2rad(20),
          azimuth_rad[channel_id]);
    }

    for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
      for (size_t channel_id = 0; channel_id < n_channels; ++channel_id) {
        size_t idx = idx_at(block_offset, channel_id);
        if (!valid_mask[idx]) {
          continue;
        }

        auto & frame = in_current_scan[channel_id] ? decode_frame_ : output_frame_;

        if (frame.blockage_mask) {
          frame.blockage_mask->update(
            azimuth_rad[channel_id], channel_id, sensor_.get_blockage_type(distances_m[idx]));
        }

        NebulaPoint point;
        point.distance = distances_m[idx];
        point.intensity = intensities[idx];
        point.time_stamp = get_point_time_relative(
          frame.scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);
        point.return_type = static_cast<uint8_t>(return_types[idx]);
        point.channel = channel_id;
        point.azimuth = azimuth_rad[channel_id];
        point.elevation = elevation_rad[channel_id];
        point.x = distances_m[idx] * cos_elevation[channel_id] * sin_azimuth[channel_id];
        point.y = distances_m[idx] * cos_elevation[channel_id] * cos_azimuth[channel_id];
        point.z = distances_m[idx] * sin_elevation[channel_id];

        if (!mask_filter_ || !mask_filter_->excluded(point)) {
          frame.pointcloud->emplace_back(point);
        }
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  [[nodiscard]] float get_distance(uint16_t distance) const
  {
    return distance * hesai_packet::get_dis_unit(packet_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing
  /// time offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  [[nodiscard]] uint32_t get_point_time_relative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id,
    size_t channel_id) const
  {
    auto point_to_packet_offset_ns =
      sensor_.get_packet_relative_point_time_offset(block_id, channel_id, packet_);
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

  DecodeFrame initialize_frame() const
  {
    DecodeFrame frame = {std::make_shared<NebulaPointCloud>(), 0, std::nullopt};
    frame.pointcloud->reserve(SensorT::max_scan_buffer_points);

    if (blockage_mask_plugin_) {
      frame.blockage_mask = point_filters::BlockageMask(
        SensorT::fov_mdeg.azimuth, blockage_mask_plugin_->get_bin_width_mdeg(),
        SensorT::packet_t::n_channels);
    }

    return frame;
  }

  /// @brief Called when a scan is complete, published and then clears the output frame.
  void on_scan_complete()
  {
    double scan_timestamp_s = static_cast<double>(output_frame_.scan_timestamp_ns) * 1e-9;

    if (pointcloud_callback_) {
      pointcloud_callback_(output_frame_.pointcloud, scan_timestamp_s);
    }

    if (blockage_mask_plugin_ && output_frame_.blockage_mask) {
      blockage_mask_plugin_->callback_and_reset(
        output_frame_.blockage_mask.value(), scan_timestamp_s);
    }

    output_frame_.pointcloud->clear();
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data,
    const std::shared_ptr<loggers::Logger> & logger,
    const std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>> &
      functional_safety_decoder,
    const std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> &
      packet_loss_detector,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(
      correction_data, sensor_configuration_->cloud_min_angle,
      sensor_configuration_->cloud_max_angle, sensor_configuration_->cut_angle),
    functional_safety_decoder_(functional_safety_decoder),
    packet_loss_detector_(packet_loss_detector),
    scan_cut_angles_(
      {deg2rad(sensor_configuration_->cloud_min_angle),
       deg2rad(sensor_configuration_->cloud_max_angle), deg2rad(sensor_configuration_->cut_angle)}),
    logger_(logger),
    blockage_mask_plugin_(std::move(blockage_mask_plugin)),
    decode_frame_(initialize_frame()),
    output_frame_(initialize_frame())
  {
    if (sensor_configuration->downsample_mask_path) {
      mask_filter_ = point_filters::DownsampleMaskFilter(
        sensor_configuration->downsample_mask_path.value(), SensorT::fov_mdeg.azimuth,
        SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels,
        logger_->child("Downsample Mask"), true, sensor_.get_dither_transform());
    }
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = std::move(callback);
  }

  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    util::Stopwatch decode_watch;

    if (!parse_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns(), 0}, DecodeError::PACKET_PARSE_FAILED};
    }

    if (packet_loss_detector_) {
      packet_loss_detector_->update(packet_);
    }

    // Even if the checksums of other parts of the packet are invalid, functional safety info
    // is still checked. This is a null-op for sensors that do not support functional safety.
    if (functional_safety_decoder_) {
      functional_safety_decoder_->update(packet_);
    }

    // FYI: This is where the CRC would be checked. Since this caused performance issues in the
    // past, and since the frame check sequence of the packet is already checked by the NIC, we
    // skip it here.

    // This is the first scan, set scan timestamp to whatever packet arrived first
    if (decode_frame_.scan_timestamp_ns == 0) {
      decode_frame_.scan_timestamp_ns =
        hesai_packet::get_timestamp_ns(packet_) +
        sensor_.get_earliest_point_time_offset_for_block(0, packet_);
    }

    bool did_scan_complete = false;

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();

      if (angle_corrector_.passed_timestamp_reset_angle(last_azimuth_, block_azimuth)) {
        uint64_t new_scan_timestamp_ns =
          hesai_packet::get_timestamp_ns(packet_) +
          sensor_.get_earliest_point_time_offset_for_block(block_id, packet_);

        if (sensor_configuration_->cut_angle == sensor_configuration_->cloud_max_angle) {
          // In the non-360 deg case, if the cut angle and FoV end coincide, the old pointcloud
          // has already been swapped and published before the timestamp reset angle is reached.
          // Thus, the `decode` pointcloud is now empty and will be decoded to. Reset its
          // timestamp.
          decode_frame_.scan_timestamp_ns = new_scan_timestamp_ns;
          decode_frame_.pointcloud->clear();
        } else {
          // When not cutting at the end of the FoV (i.e. the FoV is 360 deg or a cut occurs
          // somewhere within a non-360 deg FoV), the current scan is still being decoded to the
          // `decode` pointcloud but at the same time, points for the next pointcloud are arriving
          // and will be decoded to the `output` pointcloud (please forgive the naming for now).
          // Thus, reset the output pointcloud's timestamp.
          output_frame_.scan_timestamp_ns = new_scan_timestamp_ns;
        }
      }

      if (!angle_corrector_.is_inside_fov(last_azimuth_, block_azimuth)) {
        last_azimuth_ = block_azimuth;
        continue;
      }

      switch (n_returns) {
        case 1:
          convert_returns<1>(block_id);
          break;
        case 2:
          convert_returns<2>(block_id);
          break;
        case 3:
          convert_returns<3>(block_id);
          break;
        default:
          throw std::runtime_error("Unsupported number of returns: " + std::to_string(n_returns));
      }

      if (angle_corrector_.passed_emit_angle(last_azimuth_, block_azimuth)) {
        // The current `decode` pointcloud is ready for publishing, swap buffers to continue with
        // the `output` pointcloud as the `decode` pointcloud.
        std::swap(decode_frame_, output_frame_);
        did_scan_complete = true;
      }

      last_azimuth_ = block_azimuth;
    }

    uint64_t decode_duration_ns = decode_watch.elapsed_ns();
    uint64_t callbacks_duration_ns = 0;

    if (did_scan_complete) {
      util::Stopwatch callback_watch;
      on_scan_complete();
      callbacks_duration_ns = callback_watch.elapsed_ns();
    }

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    metadata.did_scan_complete = did_scan_complete;
    return {PerformanceCounters{decode_duration_ns, callbacks_duration_ns}, metadata};
  }
};

}  // namespace nebula::drivers
