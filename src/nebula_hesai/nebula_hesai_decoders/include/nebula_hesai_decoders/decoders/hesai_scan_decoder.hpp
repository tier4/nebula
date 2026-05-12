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

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <cstdint>
#include <vector>

namespace nebula::drivers
{
/// @brief Errors that can occur during packet decoding
enum class DecodeError : uint8_t {
  PACKET_PARSE_FAILED,  ///< Failed to parse packet structure
  DRIVER_NOT_OK,        ///< Driver status is not OK
  INVALID_PACKET_SIZE,  ///< Packet size is invalid
};

/// @brief Metadata for a decoded packet
struct PacketMetadata
{
  /// @brief Timestamp included in the packet payload in nanoseconds
  uint64_t packet_timestamp_ns{};
  /// @brief Whether a scan completed with this packet
  bool did_scan_complete{false};
};

/// @brief Performance information about decode timings
struct PerformanceCounters
{
  uint64_t decode_time_ns{0};
};

struct PacketDecodeResult
{
  /// @brief Performance information about decode and callback timings
  PerformanceCounters performance_counters;
  /// @brief Metadata or error information about the decoded packet
  util::expected<PacketMetadata, DecodeError> metadata_or_error;
};

/// @brief Base class for Hesai LiDAR decoder
class HesaiScanDecoder
{
public:
  using pointcloud_callback_t =
    std::function<void(const NebulaPointCloudPtr & pointcloud, double timestamp_s)>;

  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;

  /// @brief Parses PandarPacket and add its points to the point cloud
  /// @param packet The incoming PandarPacket
  /// @return Metadata on success, or decode error on failure. Performance counters are always
  /// returned.
  virtual PacketDecodeResult unpack(const std::vector<uint8_t> & packet) = 0;

  virtual void set_pointcloud_callback(pointcloud_callback_t callback) = 0;
};
}  // namespace nebula::drivers

#endif  // NEBULA_WS_HESAI_SCAN_DECODER_HPP
