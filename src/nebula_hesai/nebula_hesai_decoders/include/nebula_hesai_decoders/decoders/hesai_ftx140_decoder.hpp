// Copyright 2026 TIER IV, Inc.
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
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct PreHeaderFtx
{
  /// @brief Start of Packet, 0xEEFF
  uint16_t sop;
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t status_info_version;
  /// @brief 1 for IMU packets, 0 for point cloud packets
  uint8_t packet_type;
};

struct HeaderFtx
{
  uint16_t total_columns;
  uint16_t total_rows;
  uint8_t column_resolution;
  uint8_t row_resolution;
  uint8_t echo;
  uint8_t dis_unit;
  /// @brief 1 or 3 if the sensor is configured to output its rows bottom-up
  uint8_t configuration_number;
  uint16_t channel_num;
  uint8_t reserved[8];
};

struct UnitFtx
{
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t background;
  uint8_t confidence;
};

struct TailFtx
{
  uint8_t reserved[7];
  /// @brief Column of the packet's subzone, 0 to 15
  uint8_t column_id;
  /// @brief Row of the packet's subzone, 0 to 31
  uint8_t row_id;
  /// @brief Frame counter, incremented at the end of each frame
  uint8_t frame_id;
  uint8_t operational_mode;
  uint8_t return_mode;
  /// @brief Frame period in milliseconds
  uint16_t frame_period;
  SecondsSinceEpoch date_time;
  uint32_t timestamp;
  uint8_t factory_information;
  uint32_t udp_sequence;
};

/// @brief Point cloud packet of the FTX140 (UDP protocol 7.3). Each packet contains one subzone of
/// 6 rows x 16 columns of the sensor's 192 x 256 channel array.
struct PacketFtx
{
  static constexpr size_t n_channels = 96;

  PreHeaderFtx pre_header;
  HeaderFtx header;
  UnitFtx units[n_channels];
  TailFtx tail;
  uint8_t functional_safety_and_cybersecurity[20];
};

#pragma pack(pop)

static_assert(sizeof(PacketFtx) == 554);

}  // namespace hesai_packet

/// @brief Decoder for the FTX140 solid-state lidar
///
/// Unlike the rotating sensors, the FTX140 reports pixels of a channel array instead of azimuth
/// blocks. Each pixel's direction is looked up from the calibration and frames are cut when the
/// frame ID changes. IMU and fault message packets, which arrive on the same port, are ignored.
class HesaiFtx140Decoder : public HesaiScanDecoder
{
public:
  using packet_t = hesai_packet::PacketFtx;

  static constexpr size_t n_rows = HesaiFtxCalibration::max_rows;
  static constexpr size_t n_columns = HesaiFtxCalibration::max_columns;
  static constexpr size_t subzone_rows = 6;
  static constexpr size_t subzone_columns = 16;
  /// @brief In zoom mode, only the central 16 x 10 subzones are output
  static constexpr uint16_t zoom_rows = 16 * subzone_rows;
  static constexpr uint16_t zoom_columns = 10 * subzone_columns;
  static constexpr size_t zoom_row_offset = 8 * subzone_rows;
  static constexpr size_t zoom_column_offset = 2 * subzone_columns;

  HesaiFtx140Decoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const HesaiFtxCalibration> & calibration,
    const std::shared_ptr<loggers::Logger> & logger,
    std::shared_ptr<PacketLossDetectorTypedBase<packet_t>> packet_loss_detector)
  : sensor_configuration_(sensor_configuration),
    logger_(logger),
    packet_loss_detector_(std::move(packet_loss_detector)),
    pointcloud_(std::make_shared<NebulaPointCloud>())
  {
    if (!calibration) {
      throw std::runtime_error("FTX140 decoder requires FTX calibration data");
    }

    pointcloud_->reserve(n_rows * n_columns);
    pixels_.resize(n_rows * n_columns);
    for (size_t row = 0; row < n_rows; ++row) {
      for (size_t column = 0; column < n_columns; ++column) {
        pixels_[row * n_columns + column] = make_pixel(calibration->pixel_direction(row, column));
      }
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
    did_scan_complete_ = false;

    if (!is_point_cloud_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_IGNORED};
    }

    if (packet.size() < sizeof(packet_t)) {
      NEBULA_LOG_STREAM(
        logger_->error,
        "Packet size mismatch: " << packet.size() << " | Expected at least: " << sizeof(packet_t));
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
    }
    std::memcpy(&packet_, packet.data(), sizeof(packet_t));

    if (packet_loss_detector_) {
      packet_loss_detector_->update(packet_);
    }

    const auto & header = packet_.header;
    const bool is_full = header.total_rows == n_rows && header.total_columns == n_columns;
    const bool is_zoom = header.total_rows == zoom_rows && header.total_columns == zoom_columns;
    if (header.channel_num != packet_t::n_channels || !(is_full || is_zoom)) {
      NEBULA_LOG_STREAM(
        logger_->error, "Unsupported FTX packet layout: " << header.total_columns << " x "
                                                          << header.total_rows << " pixels, "
                                                          << header.channel_num << " channels");
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
    }

    // Switching between normal and zoom mode changes the frame layout, so drop the partial frame
    if (current_layout_ && *current_layout_ != is_zoom) {
      reset_frame();
    }
    current_layout_ = is_zoom;

    if (current_frame_id_ && packet_.tail.frame_id != *current_frame_id_) {
      complete_frame();
    }
    current_frame_id_ = packet_.tail.frame_id;

    const uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    if (!frame_timestamp_ns_) {
      frame_timestamp_ns_ = packet_timestamp_ns;
    }

    convert_units(is_zoom, packet_timestamp_ns);

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = packet_timestamp_ns;
    metadata.did_scan_complete = did_scan_complete_;
    return {PerformanceCounters{decode_watch.elapsed_ns() - callback_time_ns_}, metadata};
  }

private:
  struct Pixel
  {
    bool valid{false};
    CorrectedAngleData angles{};
  };

  static Pixel make_pixel(const std::array<float, 3> & direction)
  {
    const auto [x, y, z] = direction;
    const float xy = std::hypot(x, y);
    if (xy == 0 && z == 0) {
      return {};
    }

    Pixel pixel;
    pixel.valid = true;
    pixel.angles.azimuth_rad = normalize_angle(std::atan2(x, y), 2 * M_PIf);
    pixel.angles.elevation_rad = std::atan2(z, xy);
    pixel.angles.sin_azimuth = xy > 0 ? x / xy : 0.f;
    pixel.angles.cos_azimuth = xy > 0 ? y / xy : 1.f;
    pixel.angles.sin_elevation = std::sin(pixel.angles.elevation_rad);
    pixel.angles.cos_elevation = std::cos(pixel.angles.elevation_rad);
    return pixel;
  }

  static bool is_point_cloud_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(hesai_packet::PreHeaderFtx)) {
      return false;
    }
    hesai_packet::PreHeaderFtx pre_header{};
    std::memcpy(&pre_header, packet.data(), sizeof(pre_header));
    constexpr uint8_t imu_packet_type = 1;
    return pre_header.sop == 0xFFEE && pre_header.protocol_major == 7 &&
           pre_header.protocol_minor == 3 && pre_header.packet_type != imu_packet_type;
  }

  [[nodiscard]] ReturnType get_return_type() const
  {
    switch (packet_.tail.return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;
      case hesai_packet::return_mode::SINGLE_LAST:
        return ReturnType::LAST;
      default:
        return ReturnType::UNKNOWN;
    }
  }

  void convert_units(bool is_zoom, uint64_t packet_timestamp_ns)
  {
    const size_t row_offset = is_zoom ? zoom_row_offset : 0;
    const size_t column_offset = is_zoom ? zoom_column_offset : 0;
    const bool rows_bottom_up =
      packet_.header.configuration_number == 1 || packet_.header.configuration_number == 3;
    const double dis_unit = hesai_packet::get_dis_unit(packet_);
    const auto return_type = static_cast<uint8_t>(get_return_type());
    const auto point_time = static_cast<uint32_t>(packet_timestamp_ns - *frame_timestamp_ns_);

    const float cloud_min_rad = deg2rad(static_cast<float>(sensor_configuration_->cloud_min_angle));
    const float cloud_max_rad = deg2rad(static_cast<float>(sensor_configuration_->cloud_max_angle));

    for (size_t i = 0; i < packet_t::n_channels; ++i) {
      const auto & unit = packet_.units[i];
      if (unit.distance == 0) {
        continue;
      }

      // Units are ordered column-major within the subzone
      size_t row = packet_.tail.row_id * subzone_rows + i % subzone_rows + row_offset;
      const size_t column =
        packet_.tail.column_id * subzone_columns + i / subzone_rows + column_offset;
      if (row >= n_rows || column >= n_columns) {
        continue;
      }
      // The calibration (and thus the reported channel) uses the default row order, as in the SDK
      if (rows_bottom_up) {
        row = n_rows - 1 - row;
      }

      const auto & pixel = pixels_[row * n_columns + column];
      if (!pixel.valid) {
        continue;
      }

      const auto distance = static_cast<float>(unit.distance * dis_unit);
      if (
        distance < sensor_configuration_->min_range ||
        sensor_configuration_->max_range < distance) {
        continue;
      }

      const auto & angles = pixel.angles;
      if (!angle_is_between(cloud_min_rad, cloud_max_rad, angles.azimuth_rad)) {
        continue;
      }

      NebulaPoint point;
      const float xy_distance = distance * angles.cos_elevation;
      point.x = xy_distance * angles.sin_azimuth;
      point.y = xy_distance * angles.cos_azimuth;
      point.z = distance * angles.sin_elevation;
      point.distance = distance;
      point.intensity = unit.reflectivity;
      point.return_type = return_type;
      point.channel = static_cast<uint16_t>(row);
      point.azimuth = angles.azimuth_rad;
      point.elevation = angles.elevation_rad;
      point.time_stamp = point_time;
      pointcloud_->emplace_back(point);
    }
  }

  void complete_frame()
  {
    if (frame_timestamp_ns_) {
      did_scan_complete_ = true;
      if (pointcloud_callback_) {
        constexpr uint64_t nanoseconds_per_second = 1'000'000'000ULL;
        const double timestamp_s =
          static_cast<double>(*frame_timestamp_ns_ / nanoseconds_per_second) +
          static_cast<double>(*frame_timestamp_ns_ % nanoseconds_per_second) / 1e9;
        util::Stopwatch callback_watch;
        pointcloud_callback_(pointcloud_, timestamp_s);
        callback_time_ns_ += callback_watch.elapsed_ns();
      }
    }
    reset_frame();
  }

  void reset_frame()
  {
    pointcloud_->clear();
    frame_timestamp_ns_.reset();
    current_frame_id_.reset();
  }

  const std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<loggers::Logger> logger_;
  std::shared_ptr<PacketLossDetectorTypedBase<packet_t>> packet_loss_detector_;
  pointcloud_callback_t pointcloud_callback_;

  /// @brief Angles of each pixel, indexed by `row * n_columns + column`
  std::vector<Pixel> pixels_;

  packet_t packet_{};
  NebulaPointCloudPtr pointcloud_;
  std::optional<uint64_t> frame_timestamp_ns_;
  std::optional<uint8_t> current_frame_id_;
  /// @brief Whether the current frame is in zoom mode
  std::optional<bool> current_layout_;

  uint64_t callback_time_ns_{0};
  bool did_scan_complete_{false};
};

}  // namespace nebula::drivers
