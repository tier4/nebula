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

#ifndef NEBULA_SAMPLE_SCAN_DECODER_HPP
#define NEBULA_SAMPLE_SCAN_DECODER_HPP

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_sample_common/sample_common.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace nebula::drivers
{
/// @brief Error codes returned during packet decoding
enum class DecodeError : uint8_t {
  PACKET_PARSE_FAILED,  ///< Failed to parse packet data (invalid format, checksum error, etc.)
  DRIVER_NOT_OK,        ///< Driver is not in a valid state to decode packets
  INVALID_PACKET_SIZE,  ///< Packet size doesn't match expected size for this sensor
};

/// @brief Metadata extracted from a decoded packet
struct PacketMetadata
{
  uint64_t packet_timestamp_ns{};  ///< Timestamp of the packet in nanoseconds
  bool did_scan_complete{false};   ///< True if this packet completed a scan
};

/// @brief Performance metrics for packet decoding
struct PerformanceCounters
{
  uint64_t decode_time_ns{0};    ///< Time spent decoding the packet (ns)
  uint64_t callback_time_ns{0};  ///< Time spent in the pointcloud callback (ns)
};

/// @brief Result of decoding a single packet
/// @details Contains either successful metadata or an error, plus performance counters
struct PacketDecodeResult
{
  PerformanceCounters performance_counters;                       ///< Timing information
  util::expected<PacketMetadata, DecodeError> metadata_or_error;  ///< Decode result or error
};

/// @brief Base interface for packet decoders
/// @details Implement this interface to decode raw UDP packets into point clouds.
/// The decoder is responsible for:
/// - Parsing binary packet data according to the sensor's protocol
/// - Accumulating points until a full scan is complete
/// - Calling the pointcloud callback when a scan is ready
class SampleScanDecoder
{
public:
  /// @brief Callback type for publishing complete point clouds
  /// @param pointcloud The decoded point cloud
  /// @param timestamp_s Timestamp of the scan in seconds
  using pointcloud_callback_t =
    std::function<void(const NebulaPointCloudPtr & pointcloud, double timestamp_s)>;

  SampleScanDecoder(SampleScanDecoder && c) = delete;
  SampleScanDecoder & operator=(SampleScanDecoder && c) = delete;
  SampleScanDecoder(const SampleScanDecoder & c) = delete;
  SampleScanDecoder & operator=(const SampleScanDecoder & c) = delete;

  virtual ~SampleScanDecoder() = default;
  SampleScanDecoder() = default;

  /// @brief Decode a single UDP packet
  /// @param packet Raw packet data received from the sensor
  /// @return PacketDecodeResult containing metadata or error, plus performance counters
  /// @details This is the main decoding function. Implement sensor-specific parsing logic here.
  /// Parse the packet, extract points, and accumulate them. When a full scan is complete,
  /// call the pointcloud callback and set did_scan_complete = true.
  virtual PacketDecodeResult unpack(const std::vector<uint8_t> & packet) = 0;

  /// @brief Register a callback to receive complete point clouds
  /// @param callback Function to call when a full scan is decoded
  /// @details The decoder calls this callback when a complete scan is ready
  virtual void set_pointcloud_callback(pointcloud_callback_t callback) = 0;
};
}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_SCAN_DECODER_HPP
