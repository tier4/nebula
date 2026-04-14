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

#ifndef NEBULA_SEYOND_DECODER_HPP
#define NEBULA_SEYOND_DECODER_HPP

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_packet.hpp>

#include <functional>
#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Result of unpacking a single Seyond packet
struct SeyondPacketDecodeResult
{
  uint64_t sensor_timestamp_ns;
  size_t points_unpacked;
};

/// @brief Decoder for Seyond LiDAR data packets
class SeyondDecoder
{
public:
  using pointcloud_callback_t = std::function<void(NebulaPointCloudPtr, uint64_t)>;

  explicit SeyondDecoder(
    const SeyondSensorConfiguration & config, pointcloud_callback_t pointcloud_cb,
    const SeyondCalibrationData & calibration = SeyondCalibrationData{});

  /// @brief Unpack a raw Seyond packet and return the timestamp and number of points.
  SeyondPacketDecodeResult unpack(const std::vector<uint8_t> & packet_data);

private:
  void parse_falcon_k(const SeyondDataPacket * packet);
  void parse_robin_w_e1x(const SeyondDataPacket * packet);
  void parse_robin_compact(const SeyondDataPacket * packet);
  void parse_hummingbird_d1(const SeyondDataPacket * packet);

  void add_point(
    float x, float y, float z, uint8_t intensity, uint16_t channel, uint32_t timestamp_ns);

  SeyondSensorConfiguration config_;
  SeyondCalibrationData calibration_;
  pointcloud_callback_t pointcloud_callback_;
  NebulaPointCloudPtr current_scan_cloud_;
  uint64_t current_scan_frame_idx_{0};
  uint64_t current_scan_start_timestamp_ns_{0};
  bool has_current_scan_frame_{false};
};

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_DECODER_HPP
