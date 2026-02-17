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

#ifndef NEBULA_SAMPLE_DECODER_HPP
#define NEBULA_SAMPLE_DECODER_HPP

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/angles.hpp>
#include <nebula_core_common/util/expected.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace nebula::drivers
{

/// @brief Error codes returned during packet decoding
enum class DecodeError : uint8_t {
  PACKET_FORMAT_INVALID,  ///< Packet content did not match the expected format
  CALLBACK_NOT_SET,       ///< Decoder cannot publish scans because callback is missing
  EMPTY_PACKET,           ///< Received packet had zero bytes
};

const char * to_cstr(DecodeError error);

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

/// @brief Concrete implementation of the Sample packet decoder
/// @details This class implements the SampleScanDecoder interface.
/// Implement the following in this class:
/// - Parse raw UDP packets according to your sensor's protocol specification
/// - Perform validation checks
/// - Extract point data (x, y, z, intensity, timestamp, etc.)
/// - Accumulate points until a full scan is complete
/// - Call the pointcloud callback when a scan is ready
class SampleDecoder
{
public:
  /// @brief Callback type for publishing complete point clouds
  /// @param pointcloud The decoded point cloud
  /// @param timestamp_s Timestamp of the scan in seconds
  using pointcloud_callback_t =
    std::function<void(const NebulaPointCloudPtr & pointcloud, double timestamp_s)>;

  /// @brief Constructor
  /// @param fov Field of view to crop the point cloud to
  /// @details Initialize any internal state needed for decoding (e.g., point accumulation buffers)
  explicit SampleDecoder(
    FieldOfView<float, Degrees> fov /*, other decoder args */, pointcloud_callback_t pointcloud_cb);

  /// @brief Decode a single UDP packet
  /// @param packet Raw packet bytes from the sensor
  /// @return PacketDecodeResult with metadata or error
  /// @details Implement your sensor's packet parsing logic here:
  /// 1. Validate packet size and checksum
  /// 2. Parse packet header (timestamp, azimuth, etc.)
  /// 3. Extract point data blocks
  /// 4. Convert raw data to 3D points (apply calibration if needed)
  /// 5. Accumulate points in the current scan
  /// 6. Detect scan completion
  /// 7. If scan complete, call pointcloud_callback_ and set did_scan_complete=true
  [[nodiscard]] PacketDecodeResult unpack(const std::vector<uint8_t> & packet);

  void set_pointcloud_callback(pointcloud_callback_t pointcloud_cb);

private:
  static constexpr uint64_t kPacketsPerDummyScan = 10;

  FieldOfView<float, Degrees> fov_;            ///< Field of view used for future point filtering
  pointcloud_callback_t pointcloud_callback_;  ///< Callback for publishing scans
  NebulaPointCloudPtr current_scan_cloud_{new NebulaPointCloud()};
  uint64_t packet_count_{0};
  // Implementation Items: Add member variables for:
  // - Point cloud accumulation buffer
  // - Previous azimuth for scan completion detection
  // - Scan timestamp
  // - Optionally: Double-buffering (two point cloud buffers) for handling scan overlap
  //   (required if azimuth correction is done in Nebula)
  // - Any sensor-specific state
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DECODER_HPP
