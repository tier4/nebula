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

#ifndef NEBULA_OUSTER_METADATA_HPP
#define NEBULA_OUSTER_METADATA_HPP

#include "nebula_ouster_decoders/ouster_packet.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace nebula::drivers
{

/// @brief Sensor metadata parsed from Ouster's sensor_info JSON document.
/// @details Only the fields actually needed for decoding are kept. The JSON layout is documented
/// in Ouster's software user manual (sensor_info endpoint).
struct OusterMetadata
{
  // Beam geometry (length == pixels_per_column, i.e. 128 for OS-128).
  std::vector<double> beam_altitude_angles_deg;
  std::vector<double> beam_azimuth_angles_deg;

  // Column stagger: integer column offset applied per beam when reassembling a scan.
  std::vector<int32_t> pixel_shift_by_row;

  // Sensor frame origin offset (mm) — added to each XYZ along +Z of lidar frame.
  double lidar_origin_to_beam_origin_mm{0.0};

  // Scan geometry
  uint32_t columns_per_frame{1024};   ///< e.g. 512, 1024, 2048
  uint32_t columns_per_packet{16};    ///< typically 16 for modern firmware
  uint32_t pixels_per_column{128};    ///< number of beams (rows)

  // UDP packet format
  ouster_packet::UdpProfileLidar udp_profile_lidar{
    ouster_packet::UdpProfileLidar::RNG19_RFL8_SIG16_NIR16};

  // Derived packet sizes (bytes).
  size_t lidar_packet_size_bytes{0};
  size_t imu_packet_size_bytes{ouster_packet::k_imu_packet_size};

  // Informational — not required for decoding.
  std::string sensor_prod_line;   ///< e.g. "OS-1-128"
  std::string lidar_mode;          ///< e.g. "1024x10"
  std::string firmware_version;

  /// @brief Number of returns encoded per pixel (1 for single, 2 for dual).
  [[nodiscard]] uint8_t num_returns() const
  {
    return ouster_packet::get_layout(udp_profile_lidar).returns;
  }

  /// @brief Recompute lidar_packet_size_bytes from the profile, columns-per-packet, and pixels.
  void update_derived_sizes()
  {
    lidar_packet_size_bytes =
      ouster_packet::lidar_packet_size(udp_profile_lidar, columns_per_packet, pixels_per_column);
  }
};

/// @brief Parse a sensor_info JSON document (the output of /api/v1/sensor/metadata/sensor_info) into
/// an OusterMetadata struct.
/// @throws std::runtime_error when required fields are missing or malformed.
OusterMetadata parse_ouster_metadata(const std::string & sensor_info_json);

/// @brief Fetch sensor metadata from a live Ouster sensor via HTTP (uses Nebula's HTTP client).
/// @param sensor_ip IPv4 address (or hostname) of the sensor.
/// @param timeout_ms HTTP receive timeout in milliseconds.
/// @return JSON string as returned by the sensor.
std::string fetch_ouster_metadata_http(const std::string & sensor_ip, int timeout_ms = 2000);

}  // namespace nebula::drivers

#endif  // NEBULA_OUSTER_METADATA_HPP
