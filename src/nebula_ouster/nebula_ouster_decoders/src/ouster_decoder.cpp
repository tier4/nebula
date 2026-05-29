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

#include "nebula_ouster_decoders/ouster_decoder.hpp"

#include "nebula_ouster_decoders/ouster_packet.hpp"
#include "nebula_ouster_decoders/ouster_xyz_lut.hpp"

#include <nebula_core_common/nebula_common.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace nebula::drivers
{

namespace
{
constexpr float k_g_to_m_s2 = 9.80665f;
constexpr float k_deg_to_rad = static_cast<float>(M_PI) / 180.0f;

uint64_t now_steady_ns(std::chrono::steady_clock::time_point start)
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start)
      .count());
}

/// @brief Wrap azimuth into [0, 360).
float normalize_azimuth_deg(float az_deg)
{
  az_deg = std::fmod(az_deg, 360.0f);
  if (az_deg < 0.0f) az_deg += 360.0f;
  return az_deg;
}
}  // namespace

const char * to_cstr(const DecodeError error)
{
  switch (error) {
    case DecodeError::PACKET_FORMAT_INVALID:
      return "packet format invalid";
    case DecodeError::CALLBACK_NOT_SET:
      return "pointcloud callback is not set";
    case DecodeError::EMPTY_PACKET:
      return "packet is empty";
  }
  return "unknown decode error";
}

struct OusterDecoder::Impl
{
  FieldOfView<float, Degrees> fov;
  OusterMetadata metadata;
  OusterXyzLut lut;
  pointcloud_callback_t pointcloud_callback;
  imu_callback_t imu_callback;

  // Packet processing must be serialized — the HW interface may fire the callback from two
  // different socket threads (lidar + IMU ports). Shared mutable state (scan_cloud,
  // prev_measurement_id, prev_frame_id, scan_first_ts_ns) would otherwise race.
  std::mutex decode_mutex;

  // Current scan state.
  NebulaPointCloudPtr scan_cloud;  ///< Accumulating cloud.
  uint64_t scan_first_ts_ns{0};    ///< Sensor timestamp of first valid column in the scan.
  int32_t prev_measurement_id{-1}; ///< Last column index seen, -1 before first packet.
  int32_t prev_frame_id{-1};       ///< Last frame_id from packet header, -1 before first packet.

  Impl(FieldOfView<float, Degrees> fov_in, OusterMetadata md, pointcloud_callback_t cb)
  : fov(fov_in),
    metadata(std::move(md)),
    lut(metadata),
    pointcloud_callback(std::move(cb)),
    scan_cloud(std::make_shared<NebulaPointCloud>())
  {
    // Pre-allocate for a typical OS-128 frame (1024 columns x 128 beams x up to 2 returns).
    scan_cloud->reserve(
      metadata.columns_per_frame * metadata.pixels_per_column * metadata.num_returns());
  }

  /// @brief Start a fresh scan, emitting the previous one via callback.
  /// @param completing_column_id The measurement_id that triggered scan completion (for logging).
  void emit_and_reset_scan()
  {
    if (pointcloud_callback && !scan_cloud->empty()) {
      const double timestamp_s = static_cast<double>(scan_first_ts_ns) * 1e-9;
      pointcloud_callback(scan_cloud, timestamp_s);
    }
    scan_cloud = std::make_shared<NebulaPointCloud>();
    scan_cloud->reserve(
      metadata.columns_per_frame * metadata.pixels_per_column * metadata.num_returns());
    scan_first_ts_ns = 0;
  }

  /// @brief Append one pixel to the current scan, applying FoV filter and return-type tag.
  void push_point(
    uint32_t range_mm, size_t beam_idx, size_t measurement_id, uint8_t intensity,
    ReturnType return_type, uint32_t point_time_offset_ns)
  {
    if (range_mm == 0U) return;

    float x{}, y{}, z{}, azimuth_rad{}, elevation_rad{};
    lut.compute(
      range_mm, beam_idx, measurement_id, x, y, z, azimuth_rad, elevation_rad);

    const float azimuth_deg = normalize_azimuth_deg(azimuth_rad / k_deg_to_rad);
    if (azimuth_deg < fov.azimuth.start || azimuth_deg > fov.azimuth.end) return;

    const float elevation_deg = elevation_rad / k_deg_to_rad;
    if (elevation_deg < fov.elevation.start || elevation_deg > fov.elevation.end) return;

    NebulaPoint pt{};
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = intensity;
    pt.return_type = static_cast<uint8_t>(return_type);
    pt.channel = static_cast<uint16_t>(beam_idx);
    pt.azimuth = azimuth_rad;
    pt.elevation = elevation_rad;
    pt.distance = static_cast<float>(range_mm) * 1e-3f;
    pt.time_stamp = point_time_offset_ns;
    scan_cloud->push_back(pt);
  }

  /// @brief Parse one lidar UDP packet and add its points to the current scan.
  /// @return true if this packet closed out a scan.
  bool process_lidar_packet(const std::vector<uint8_t> & packet)
  {
    using namespace ouster_packet;
    const auto layout = get_layout(metadata.udp_profile_lidar);
    const size_t cpp = metadata.columns_per_packet;
    const size_t ppc = metadata.pixels_per_column;
    const bool dual = metadata.num_returns() == 2;
    const bool legacy = metadata.udp_profile_lidar == UdpProfileLidar::LEGACY;

    // Parse packet header (modern profiles only — LEGACY has no packet header).
    int32_t this_frame_id = -1;
    if (!legacy && packet.size() >= layout.packet_header_size) {
      this_frame_id =
        static_cast<int32_t>(ouster_packet::parse_packet_frame_id(packet.data()));
    }

    const uint8_t * p = packet.data();
    p += layout.packet_header_size;

    bool scan_completed = false;

    // Detect frame boundary via frame_id change (most reliable for Ouster firmware).
    if (this_frame_id >= 0 && prev_frame_id >= 0 && this_frame_id != prev_frame_id) {
      emit_and_reset_scan();
      scan_completed = true;
      prev_measurement_id = -1;
    }
    if (this_frame_id >= 0) prev_frame_id = this_frame_id;

    for (size_t col = 0; col < cpp; ++col) {
      const ColumnHeader header =
        legacy ? parse_legacy_column_header(p) : parse_column_header(p);
      p += layout.column_header_size;

      const uint8_t * pixel_data = p;
      p += layout.pixel_size * ppc;

      // Skip legacy per-column footer (4 bytes). Modern profiles have a single packet footer at
      // the end handled after the loop.
      if (legacy) {
        p += 4;
      }

      const bool valid = (header.status & 0x01) != 0U;
      if (!valid) continue;

      // Detect frame boundary: measurement_id wrapped back to a lower value.
      const int32_t mid = static_cast<int32_t>(header.measurement_id);
      if (prev_measurement_id >= 0 && mid < prev_measurement_id) {
        emit_and_reset_scan();
        scan_completed = true;
      }
      prev_measurement_id = mid;

      if (scan_first_ts_ns == 0) scan_first_ts_ns = header.timestamp_ns;

      const uint32_t point_time_offset_ns =
        (header.timestamp_ns >= scan_first_ts_ns)
          ? static_cast<uint32_t>((header.timestamp_ns - scan_first_ts_ns) & 0xFFFFFFFFU)
          : 0U;

      // Iterate beams (rows) within this column.
      for (size_t beam = 0; beam < ppc; ++beam) {
        const uint8_t * pixel = pixel_data + beam * layout.pixel_size;

        if (!dual) {
          uint32_t range_mm{};
          uint8_t refl{};
          if (legacy) {
            range_mm = Legacy::range_mm(pixel);
            refl = static_cast<uint8_t>(Legacy::reflectivity(pixel) & 0xFFU);
          } else {
            range_mm = Rng19Single::range_mm(pixel);
            refl = Rng19Single::reflectivity(pixel);
          }
          push_point(
            range_mm, beam, static_cast<size_t>(mid), refl, ReturnType::STRONGEST,
            point_time_offset_ns);
        } else {
          const uint32_t r1 = Rng19Dual::range1_mm(pixel);
          const uint8_t refl1 = Rng19Dual::reflectivity1(pixel);
          push_point(
            r1, beam, static_cast<size_t>(mid), refl1, ReturnType::FIRST, point_time_offset_ns);

          const uint32_t r2 = Rng19Dual::range2_mm(pixel);
          const uint8_t refl2 = Rng19Dual::reflectivity2(pixel);
          push_point(
            r2, beam, static_cast<size_t>(mid), refl2, ReturnType::LAST, point_time_offset_ns);
        }
      }
    }

    return scan_completed;
  }
};

OusterDecoder::OusterDecoder(
  FieldOfView<float, Degrees> fov, OusterMetadata metadata,
  pointcloud_callback_t pointcloud_cb)
: impl_(std::make_unique<Impl>(fov, std::move(metadata), std::move(pointcloud_cb)))
{
}

OusterDecoder::~OusterDecoder() = default;
OusterDecoder::OusterDecoder(OusterDecoder &&) noexcept = default;
OusterDecoder & OusterDecoder::operator=(OusterDecoder &&) noexcept = default;

PacketDecodeResult OusterDecoder::unpack(const std::vector<uint8_t> & packet)
{
  const auto decode_begin = std::chrono::steady_clock::now();
  PacketDecodeResult result;

  if (!impl_->pointcloud_callback) {
    result.metadata_or_error = DecodeError::CALLBACK_NOT_SET;
    result.performance_counters.decode_time_ns = now_steady_ns(decode_begin);
    return result;
  }

  if (packet.empty()) {
    result.metadata_or_error = DecodeError::EMPTY_PACKET;
    result.performance_counters.decode_time_ns = now_steady_ns(decode_begin);
    return result;
  }

  const size_t lidar_size = impl_->metadata.lidar_packet_size_bytes;
  const size_t imu_size = impl_->metadata.imu_packet_size_bytes;

  PacketMetadata metadata{};

  if (packet.size() == lidar_size) {
    metadata.did_scan_complete = impl_->process_lidar_packet(packet);
    metadata.packet_timestamp_ns = impl_->scan_first_ts_ns;
  } else if (packet.size() == imu_size) {
    if (impl_->imu_callback) {
      const auto raw = ouster_packet::parse_imu_packet(packet.data());
      OusterImuSample s{};
      s.timestamp_ns = raw.sys_timestamp_ns;
      s.accel_x = raw.accel_x_g * k_g_to_m_s2;
      s.accel_y = raw.accel_y_g * k_g_to_m_s2;
      s.accel_z = raw.accel_z_g * k_g_to_m_s2;
      s.gyro_x = raw.gyro_x_dps * k_deg_to_rad;
      s.gyro_y = raw.gyro_y_dps * k_deg_to_rad;
      s.gyro_z = raw.gyro_z_dps * k_deg_to_rad;
      impl_->imu_callback(s);
    }
    metadata.packet_timestamp_ns = 0;
    metadata.did_scan_complete = false;
  } else {
    result.metadata_or_error = DecodeError::PACKET_FORMAT_INVALID;
    result.performance_counters.decode_time_ns = now_steady_ns(decode_begin);
    return result;
  }

  result.metadata_or_error = metadata;
  result.performance_counters.decode_time_ns = now_steady_ns(decode_begin);
  return result;
}

void OusterDecoder::set_pointcloud_callback(pointcloud_callback_t pointcloud_cb)
{
  impl_->pointcloud_callback = std::move(pointcloud_cb);
}

void OusterDecoder::set_imu_callback(imu_callback_t imu_cb)
{
  impl_->imu_callback = std::move(imu_cb);
}

const OusterMetadata & OusterDecoder::metadata() const
{
  return impl_->metadata;
}

}  // namespace nebula::drivers
