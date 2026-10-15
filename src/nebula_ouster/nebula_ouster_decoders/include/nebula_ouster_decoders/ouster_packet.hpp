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

#ifndef NEBULA_OUSTER_PACKET_HPP
#define NEBULA_OUSTER_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace nebula::drivers::ouster_packet
{

// Ouster uses little-endian wire format for all multibyte fields.

/// @brief Ouster UDP lidar profile identifiers from metadata JSON.
enum class UdpProfileLidar : uint8_t {
  LEGACY,
  RNG19_RFL8_SIG16_NIR16,
  RNG19_RFL8_SIG16_NIR16_DUAL,
  RNG15_RFL8_NIR8,
};

/// @brief Byte sizes of packet-level headers and footers by profile.
struct ProfileLayout
{
  size_t packet_header_size;
  size_t column_header_size;
  size_t pixel_size;
  size_t packet_footer_size;
  uint8_t returns;  ///< Returns per pixel (1 for single, 2 for dual).
};

/// @brief Layout constants for LEGACY profile.
/// Legacy packet has no dedicated header/footer; every column carries its own 16-byte header and
/// 16-byte footer, with 12-byte pixels.
constexpr ProfileLayout k_legacy_layout{
  /*packet_header_size=*/0,
  /*column_header_size=*/16,
  /*pixel_size=*/12,
  /*packet_footer_size=*/0,
  /*returns=*/1};

/// @brief Layout constants for RNG19_RFL8_SIG16_NIR16 (single return).
/// 32-byte packet header + 12-byte column header + 12-byte column footer + 32-byte packet footer.
/// Each pixel is 12 bytes: range(4) + reflectivity(1) + reserved(1) + signal(2) + near_ir(2) + pad(2).
constexpr ProfileLayout k_rng19_single_layout{
  /*packet_header_size=*/32,
  /*column_header_size=*/12,
  /*pixel_size=*/12,
  /*packet_footer_size=*/32,
  /*returns=*/1};

/// @brief Layout constants for RNG19_RFL8_SIG16_NIR16_DUAL (dual return).
/// Same framing as single-return profile but each pixel contains two returns: range(4)+refl(1)+
/// pad(1)+signal(2) repeated twice, plus near_ir(2) and alignment pad.
constexpr ProfileLayout k_rng19_dual_layout{
  /*packet_header_size=*/32,
  /*column_header_size=*/12,
  /*pixel_size=*/24,
  /*packet_footer_size=*/32,
  /*returns=*/2};

/// @brief Pick the layout for a given profile.
inline ProfileLayout get_layout(UdpProfileLidar profile)
{
  switch (profile) {
    case UdpProfileLidar::LEGACY:
      return k_legacy_layout;
    case UdpProfileLidar::RNG19_RFL8_SIG16_NIR16:
      return k_rng19_single_layout;
    case UdpProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL:
      return k_rng19_dual_layout;
    case UdpProfileLidar::RNG15_RFL8_NIR8:
      // Not supported for OS-128 default modes; fall back to single-return framing.
      return k_rng19_single_layout;
  }
  return k_rng19_single_layout;
}

/// @brief Size of the per-column footer in the LEGACY profile (4 bytes). Modern profiles have no
/// per-column footer; they use a single packet-level footer instead.
constexpr size_t k_legacy_column_footer_size = 4;

/// @brief Total bytes of a lidar packet for a given profile + columns per packet + beams.
inline size_t lidar_packet_size(
  UdpProfileLidar profile, size_t columns_per_packet, size_t pixels_per_column)
{
  const auto layout = get_layout(profile);
  if (profile == UdpProfileLidar::LEGACY) {
    return columns_per_packet *
           (layout.column_header_size + layout.pixel_size * pixels_per_column +
            k_legacy_column_footer_size);
  }
  return layout.packet_header_size +
         columns_per_packet * (layout.column_header_size + layout.pixel_size * pixels_per_column) +
         layout.packet_footer_size;
}

constexpr size_t k_imu_packet_size = 48;

/// @brief Read a little-endian integer from a raw byte buffer.
template <typename T>
inline T read_le(const uint8_t * src)
{
  T v{};
  std::memcpy(&v, src, sizeof(T));
  return v;
}

/// @brief Column header fields shared across modern Ouster profiles.
struct ColumnHeader
{
  uint64_t timestamp_ns;   ///< Sensor-local timestamp (ns).
  uint16_t measurement_id; ///< Column index [0, columns_per_frame).
  uint16_t status;         ///< Bit 0 = valid measurement.
};

/// @brief Parse a column header from the packet buffer (modern profiles, 12 bytes).
inline ColumnHeader parse_column_header(const uint8_t * src)
{
  ColumnHeader h{};
  h.timestamp_ns = read_le<uint64_t>(src);
  h.measurement_id = read_le<uint16_t>(src + 8);
  h.status = read_le<uint16_t>(src + 10);
  return h;
}

/// @brief Parse a LEGACY column header (16 bytes) — timestamp, measurement_id, encoder, status.
inline ColumnHeader parse_legacy_column_header(const uint8_t * src)
{
  ColumnHeader h{};
  h.timestamp_ns = read_le<uint64_t>(src);
  h.measurement_id = read_le<uint16_t>(src + 8);
  // bytes 10..11 frame_id, bytes 12..15 encoder_count, status bit lives in packet footer for
  // legacy; approximate validity by a non-zero timestamp.
  h.status = h.timestamp_ns != 0U ? 0x01U : 0x00U;
  return h;
}

/// @brief Frame id for modern profiles.
inline uint16_t parse_packet_frame_id(const uint8_t * packet_begin)
{
  // byte 2..3: frame_id (little endian)
  return read_le<uint16_t>(packet_begin + 2);
}

/// @brief Pixel accessor helpers for RNG19 single-return profile.
struct Rng19Single
{
  /// @brief Range in millimeters; the low 19 bits of the 32-bit word hold range, upper bits hold
  /// flags. Ouster defines range unit = 1 mm.
  static uint32_t range_mm(const uint8_t * pixel)
  {
    return read_le<uint32_t>(pixel) & 0x7FFFFU;
  }
  static uint8_t reflectivity(const uint8_t * pixel) { return pixel[4]; }
  static uint16_t signal(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 6); }
  static uint16_t near_ir(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 8); }
};

/// @brief Pixel accessor helpers for RNG19 dual-return profile.
struct Rng19Dual
{
  static uint32_t range1_mm(const uint8_t * pixel)
  {
    return read_le<uint32_t>(pixel) & 0x7FFFFU;
  }
  static uint8_t reflectivity1(const uint8_t * pixel) { return pixel[4]; }
  static uint16_t signal1(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 6); }

  static uint32_t range2_mm(const uint8_t * pixel)
  {
    return read_le<uint32_t>(pixel + 8) & 0x7FFFFU;
  }
  static uint8_t reflectivity2(const uint8_t * pixel) { return pixel[12]; }
  static uint16_t signal2(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 14); }

  static uint16_t near_ir(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 18); }
};

/// @brief Pixel accessor helpers for LEGACY profile (12 bytes per pixel).
/// Layout: range(4, only 20 low bits) + reflectivity(2) + signal(2) + near_ir(2) + reserved(2).
struct Legacy
{
  static uint32_t range_mm(const uint8_t * pixel)
  {
    return read_le<uint32_t>(pixel) & 0xFFFFFU;  // 20 bits of range in legacy
  }
  static uint16_t reflectivity(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 4); }
  static uint16_t signal(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 6); }
  static uint16_t near_ir(const uint8_t * pixel) { return read_le<uint16_t>(pixel + 8); }
};

/// @brief Parsed IMU packet (48 bytes, always the same format).
struct ImuPacket
{
  uint64_t sys_timestamp_ns;
  uint64_t accel_timestamp_ns;
  uint64_t gyro_timestamp_ns;
  float accel_x_g;  ///< Acceleration in units of g
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;  ///< Angular velocity in degrees per second
  float gyro_y_dps;
  float gyro_z_dps;
};

inline ImuPacket parse_imu_packet(const uint8_t * src)
{
  ImuPacket p{};
  p.sys_timestamp_ns = read_le<uint64_t>(src);
  p.accel_timestamp_ns = read_le<uint64_t>(src + 8);
  p.gyro_timestamp_ns = read_le<uint64_t>(src + 16);
  std::memcpy(&p.accel_x_g, src + 24, 4);
  std::memcpy(&p.accel_y_g, src + 28, 4);
  std::memcpy(&p.accel_z_g, src + 32, 4);
  std::memcpy(&p.gyro_x_dps, src + 36, 4);
  std::memcpy(&p.gyro_y_dps, src + 40, 4);
  std::memcpy(&p.gyro_z_dps, src + 44, 4);
  return p;
}

}  // namespace nebula::drivers::ouster_packet

#endif  // NEBULA_OUSTER_PACKET_HPP
