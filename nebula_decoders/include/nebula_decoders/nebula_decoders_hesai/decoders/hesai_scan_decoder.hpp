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

#ifndef NEBULA_WS_HESAI_SCAN_DECODER_HPP
#define NEBULA_WS_HESAI_SCAN_DECODER_HPP

#include "nebula_decoders/nebula_decoders_common/point_filters/blockage_mask.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/point_types.hpp>
#include <nebula_common/util/expected.hpp>

#include <vector>

namespace nebula::drivers
{
/// @brief Errors that can occur during packet decoding
enum class DecodeError : uint8_t {
  PACKET_PARSE_FAILED,  ///< Failed to parse packet structure
  CRC_CHECK_FAILED,     ///< CRC validation failed
  DRIVER_NOT_OK,        ///< Driver status is not OK
  INVALID_PACKET_SIZE,  ///< Packet size is invalid
};

/// @brief Metadata for a decoded packet
struct PacketMetadata
{
  /// @brief Timestamp included in the packet payload in nanoseconds
  uint64_t packet_timestamp_ns;
  /// @brief Last azimuth processed by the decoder
  uint32_t last_azimuth;
};

struct DecodeFrame
{
  uint64_t timestamp_ns{0};
  NebulaPointCloudPtr pointcloud;
  std::optional<point_filters::BlockageMask> blockage_mask;
};

/// @brief Base class for Hesai LiDAR decoder
class HesaiScanDecoder
{
public:
  using frame_callback_t = std::function<void(const DecodeFrame & pointcloud)>;

  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;

  /// @brief Parses PandarPacket and add its points to the point cloud
  /// @param packet The incoming PandarPacket
  /// @return Metadata on success, or decode error on failure
  virtual nebula::util::expected<PacketMetadata, DecodeError> unpack(
    const std::vector<uint8_t> & packet) = 0;

  virtual void set_frame_callback(frame_callback_t callback) = 0;
};
}  // namespace nebula::drivers

#endif  // NEBULA_WS_HESAI_SCAN_DECODER_HPP
