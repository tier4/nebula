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

#ifndef NEBULA_OUSTER_DECODER_HPP
#define NEBULA_OUSTER_DECODER_HPP

#include "nebula_ouster_decoders/ouster_metadata.hpp"

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_decoders/angles.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Error codes returned during packet decoding.
enum class DecodeError : uint8_t {
  PACKET_FORMAT_INVALID,  ///< Packet size did not match expected lidar or IMU packet.
  CALLBACK_NOT_SET,       ///< Decoder cannot publish scans because callback is missing.
  EMPTY_PACKET,           ///< Received packet had zero bytes.
};

/// @brief Convert DecodeError to a stable string literal for logging.
const char * to_cstr(DecodeError error);

/// @brief Metadata extracted from a decoded packet.
struct PacketMetadata
{
  uint64_t packet_timestamp_ns{};  ///< Sensor-local timestamp of the packet (ns).
  bool did_scan_complete{false};   ///< True if this packet closed out a scan.
};

/// @brief Performance metrics for packet decoding.
struct PerformanceCounters
{
  uint64_t decode_time_ns{0};    ///< Time spent decoding the packet (ns).
  uint64_t callback_time_ns{0};  ///< Time spent in the pointcloud callback (ns).
};

/// @brief Result of decoding a single packet.
struct PacketDecodeResult
{
  PerformanceCounters performance_counters;
  util::expected<PacketMetadata, DecodeError> metadata_or_error;

  PacketDecodeResult() : performance_counters{}, metadata_or_error(DecodeError::CALLBACK_NOT_SET) {}
};

/// @brief Decoded IMU sample in SI units (m/s^2 for accel, rad/s for gyro).
struct OusterImuSample
{
  uint64_t timestamp_ns{};
  float accel_x{}, accel_y{}, accel_z{};  // m/s^2
  float gyro_x{}, gyro_y{}, gyro_z{};     // rad/s
};

/// @brief Decoder that parses Ouster UDP packets and publishes Nebula point clouds and IMU data.
/// Implemented with a custom packet parser — no dependency on ouster-sdk.
class OusterDecoder
{
public:
  /// @brief Callback type for publishing complete point clouds.
  using pointcloud_callback_t =
    std::function<void(const NebulaPointCloudPtr & pointcloud, double timestamp_s)>;

  /// @brief Callback type for IMU samples. Fires once per IMU UDP packet (~100 Hz).
  using imu_callback_t = std::function<void(const OusterImuSample & sample)>;

  /// @param fov Angular crop in sensor spherical coordinates (degrees).
  /// @param metadata Parsed Ouster metadata (beam angles, packet geometry, profile).
  /// @param pointcloud_cb Callback invoked when a full scan is assembled.
  OusterDecoder(
    FieldOfView<float, Degrees> fov, OusterMetadata metadata,
    pointcloud_callback_t pointcloud_cb);

  ~OusterDecoder();

  OusterDecoder(const OusterDecoder &) = delete;
  OusterDecoder & operator=(const OusterDecoder &) = delete;
  OusterDecoder(OusterDecoder && other) noexcept;
  OusterDecoder & operator=(OusterDecoder && other) noexcept;

  /// @brief Decode a single UDP packet.
  /// @param packet Raw packet bytes from the sensor.
  /// @return PacketDecodeResult with metadata on success, or DecodeError on failure.
  /// @post performance_counters.decode_time_ns is always set.
  [[nodiscard]] PacketDecodeResult unpack(const std::vector<uint8_t> & packet);

  /// @brief Replace the callback used for completed scans.
  void set_pointcloud_callback(pointcloud_callback_t pointcloud_cb);

  /// @brief Register an IMU callback. Pass nullptr to disable IMU output.
  void set_imu_callback(imu_callback_t imu_cb);

  /// @brief Access the metadata this decoder was configured with.
  [[nodiscard]] const OusterMetadata & metadata() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_OUSTER_DECODER_HPP
