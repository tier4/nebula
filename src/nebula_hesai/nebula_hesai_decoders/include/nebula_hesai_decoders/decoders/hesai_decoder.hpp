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

#include "nebula_core_decoders/point_filters/blockage_mask.hpp"
#include "nebula_core_decoders/point_filters/downsample_mask.hpp"
#include "nebula_core_decoders/scan_cutter.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

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
private:
  constexpr static uint32_t angle_unit = SensorT::packet_t::degree_subdivisions;
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

  /// @brief Keeps track of scan cutting state
  ScanCutter<SensorT::packet_t::n_channels, SensorT::packet_t::degree_subdivisions> scan_cutter_;

  std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>
    functional_safety_decoder_;
  std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> packet_loss_detector_;

  typename SensorT::packet_t packet_;
  uint64_t callback_time_ns_{0};

  std::shared_ptr<loggers::Logger> logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::n_channels> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::n_blocks>, SensorT::packet_t::max_returns>
    block_firing_offset_ns_;

  std::optional<point_filters::DownsampleMaskFilter> mask_filter_;

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin_;

  std::array<DecodeFrame, 2> frame_buffers_{initialize_frame(), initialize_frame()};

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

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convert_returns(
    size_t start_block_id, size_t n_blocks,
    const typename decltype(scan_cutter_)::State & scan_state)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units;

    // If the blockage mask plugin is not present, we can return early if distance checks fail
    const bool filters_can_return_early = !blockage_mask_plugin_;

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

        bool point_is_valid = true;

        if (unit.distance == 0) {
          point_is_valid = false;
        }

        float distance = get_distance(unit);

        if (
          distance < SensorT::min_range || SensorT::max_range < distance ||
          distance < sensor_configuration_->min_range ||
          sensor_configuration_->max_range < distance) {
          point_is_valid = false;
        }

        auto return_type = sensor_.get_return_type(
          static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
          block_offset, return_units);

        // Keep only last of multiple identical points
        if (return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
          point_is_valid = false;
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
            point_is_valid = false;
          }
        }

        if (filters_can_return_early && !point_is_valid) {
          continue;
        }

        if (!scan_state.channels_in_fov[channel_id]) {
          continue;
        }

        CorrectedAngleData corrected_angle_data =
          angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        auto & frame = frame_buffers_[scan_state.channel_buffer_indices[channel_id]];

        float azimuth = corrected_angle_data.azimuth_rad;
        if (frame.blockage_mask) {
          frame.blockage_mask->update(
            azimuth, channel_id, sensor_.get_blockage_type(unit.distance));
        }

        if (!point_is_valid) {
          continue;
        }

        NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity;
        point.time_stamp = get_point_time_relative(
          frame.scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);

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

        if (!mask_filter_ || !mask_filter_->excluded(point)) {
          frame.pointcloud->emplace_back(point);
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
  void on_scan_complete(uint8_t buffer_index)
  {
    auto & completed_frame = frame_buffers_[buffer_index];
    constexpr uint64_t nanoseconds_per_second = 1'000'000'000ULL;
    double scan_timestamp_s =
      static_cast<double>(completed_frame.scan_timestamp_ns / nanoseconds_per_second) +
      (static_cast<double>(completed_frame.scan_timestamp_ns % nanoseconds_per_second) / 1e9);

    if (pointcloud_callback_) {
      util::Stopwatch stopwatch;
      pointcloud_callback_(completed_frame.pointcloud, scan_timestamp_s);
      callback_time_ns_ = stopwatch.elapsed_ns();
    }

    if (blockage_mask_plugin_ && completed_frame.blockage_mask) {
      blockage_mask_plugin_->callback_and_reset(
        completed_frame.blockage_mask.value(), scan_timestamp_s);
    }

    completed_frame.pointcloud->clear();
  }

  void on_set_timestamp(uint8_t buffer_index)
  {
    auto & frame = frame_buffers_[buffer_index];
    frame.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    frame.scan_timestamp_ns += sensor_.get_earliest_point_time_offset_for_block(0, packet_);
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
    angle_corrector_(correction_data),
    scan_cutter_(
      sensor_configuration_->cut_angle * angle_unit,
      sensor_configuration_->cloud_min_angle * angle_unit,
      sensor_configuration_->cloud_max_angle * angle_unit,
      [this](uint8_t buffer_index) { on_scan_complete(buffer_index); },
      [this](uint8_t buffer_index) { on_set_timestamp(buffer_index); }),
    functional_safety_decoder_(functional_safety_decoder),
    packet_loss_detector_(packet_loss_detector),
    logger_(logger),
    blockage_mask_plugin_(std::move(blockage_mask_plugin))
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
    callback_time_ns_ = 0;

    if (!parse_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
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
    // past, and since the frame check sequence of the packet is already checked by the NIC, we skip
    // it here.

    bool did_scan_complete = false;

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();

      auto channel_azimuths_out = angle_corrector_.get_corrected_azimuths(block_azimuth);
      const auto & scan_state = scan_cutter_.step(channel_azimuths_out);

      if (scan_state.does_block_intersect_fov()) {
        convert_returns(block_id, n_returns, scan_state);
      }
    }

    uint64_t decode_duration_ns = decode_watch.elapsed_ns();

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    metadata.did_scan_complete = did_scan_complete;
    return {PerformanceCounters{decode_duration_ns - callback_time_ns_}, metadata};
  }
};

}  // namespace nebula::drivers
