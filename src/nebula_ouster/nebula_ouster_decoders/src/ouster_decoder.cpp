// Copyright 2025 TIER IV, Inc.
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

#include "nebula_ouster_decoders/ouster_lidar_scan_conversions.hpp"

#include <ouster/core/lidar_frame.h>
#include <ouster/core/packet.h>
#include <ouster/core/types.h>
#include <ouster/core/xyzlut.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorInfo;

struct OusterDecoder::Impl
{
  FieldOfView<float, Degrees> fov;
  pointcloud_callback_t pointcloud_callback;
  std::shared_ptr<SensorInfo> sensor_info;
  std::shared_ptr<PacketFormat> packet_format;
  ouster::sdk::core::FrameBatcher batcher;
  ouster::sdk::core::LidarFrame lidar_scan;
  ouster::sdk::core::XYZLut xyz_lut;

  Impl(
    FieldOfView<float, Degrees> fov_in, const OusterCalibrationData & calibration,
    pointcloud_callback_t cb)
  : fov(fov_in),
    pointcloud_callback(std::move(cb)),
    sensor_info(calibration.sensor_info),
    packet_format(calibration.packet_format),
    batcher(sensor_info),
    lidar_scan(sensor_info),
    xyz_lut(*sensor_info, false)
  {
  }
};

const char * to_cstr(const DecodeError error)
{
  switch (error) {
    case DecodeError::PACKET_FORMAT_INVALID:
      return "packet format invalid";
    case DecodeError::CALLBACK_NOT_SET:
      return "pointcloud callback is not set";
    case DecodeError::EMPTY_PACKET:
      return "packet is empty";
    default:
      return "unknown decode error";
  }
}

OusterDecoder::OusterDecoder(
  FieldOfView<float, Degrees> fov, const OusterCalibrationData & calibration,
  pointcloud_callback_t pointcloud_cb)
: impl_(std::make_unique<Impl>(fov, calibration, std::move(pointcloud_cb)))
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
    result.performance_counters.decode_time_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - decode_begin)
                              .count());
    return result;
  }

  if (packet.empty()) {
    result.metadata_or_error = DecodeError::EMPTY_PACKET;
    result.performance_counters.decode_time_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - decode_begin)
                              .count());
    return result;
  }

  const auto & pf = impl_->batcher.pf;
  if (
    packet.size() != pf.lidar_packet_size && packet.size() != pf.imu_packet_size &&
    packet.size() != pf.zone_packet_size) {
    result.metadata_or_error = DecodeError::PACKET_FORMAT_INVALID;
    result.performance_counters.decode_time_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - decode_begin)
                              .count());
    return result;
  }

  bool complete = false;
  // TODO(unaal) refactor the code so we don't have to copy packet data.
  if (packet.size() == pf.lidar_packet_size) {
    ouster::sdk::core::LidarPacket lidar_pkt(impl_->packet_format);
    std::memcpy(lidar_pkt.buf.data(), packet.data(), packet.size());
    complete = impl_->batcher.batch(lidar_pkt, impl_->lidar_scan);
  } else if (packet.size() == pf.imu_packet_size) {
    ouster::sdk::core::ImuPacket imu_pkt(impl_->packet_format);
    std::memcpy(imu_pkt.buf.data(), packet.data(), packet.size());
    complete = impl_->batcher.batch(imu_pkt, impl_->lidar_scan);
  } else if (packet.size() == pf.zone_packet_size) {
    ouster::sdk::core::ZonePacket zone_pkt(impl_->packet_format);
    std::memcpy(zone_pkt.buf.data(), packet.data(), packet.size());
    complete = impl_->batcher.batch(zone_pkt, impl_->lidar_scan);
  }

  PacketMetadata metadata{};
  metadata.packet_timestamp_ns =
    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count());
  metadata.did_scan_complete = complete;

  if (complete) {
    const auto callback_begin = std::chrono::steady_clock::now();
    NebulaPointCloudPtr cloud =
      nebula_point_cloud_from_lidar_scan(impl_->lidar_scan, impl_->xyz_lut, impl_->fov);
    uint64_t scan_ts = 0U;
    try {
      scan_ts = impl_->lidar_scan.timestamp()(impl_->lidar_scan.get_first_valid_column());
    } catch (const std::runtime_error &) {
      // No valid columns in this scan; leave the timestamp at zero.
    }
    const double timestamp_s = scan_ts != 0U ? static_cast<double>(scan_ts) * 1e-9 : 0.0;
    impl_->pointcloud_callback(cloud, timestamp_s);
    result.performance_counters.callback_time_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - callback_begin)
                              .count());
  }

  result.metadata_or_error = metadata;
  result.performance_counters.decode_time_ns =
    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - decode_begin)
                            .count());
  return result;
}

void OusterDecoder::set_pointcloud_callback(pointcloud_callback_t pointcloud_cb)
{
  impl_->pointcloud_callback = std::move(pointcloud_cb);
}

size_t OusterDecoder::lidar_packet_size() const
{
  return impl_->packet_format->lidar_packet_size;
}

}  // namespace nebula::drivers
