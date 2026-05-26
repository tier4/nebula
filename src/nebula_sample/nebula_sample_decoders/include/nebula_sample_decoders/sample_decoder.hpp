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
#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_decoders/angles.hpp>

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

/// @brief Convert DecodeError to a stable string literal for logging.
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
  /// Timing measurements for the decode and callback path.
  PerformanceCounters performance_counters;
  /// Successful metadata or a decode error.
  util::expected<PacketMetadata, DecodeError> metadata_or_error;
};

/// @brief Decoder that accumulates incoming raw packets and marks scan boundaries.
/// @details The tutorial implementation emits empty pointclouds so the ROS pipeline can be
/// exercised without inventing fake sensor geometry.
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
  /// @param pointcloud_cb Callback invoked when a full scan is assembled
  explicit SampleDecoder(
    FieldOfView<float, Degrees> fov /*, other decoder args */, pointcloud_callback_t pointcloud_cb);

  /// @brief Decode a single UDP packet
  /// @param packet Raw packet bytes from the sensor
  /// @return PacketDecodeResult with metadata on success, or DecodeError on failure.
  /// @post performance_counters.decode_time_ns is always set.
  [[nodiscard]] PacketDecodeResult unpack(const std::vector<uint8_t> & packet);

  /// @brief Replace the callback used for completed scans.
  void set_pointcloud_callback(pointcloud_callback_t pointcloud_cb);

private:
  static constexpr uint64_t k_packets_per_sample_scan = 10;

  FieldOfView<float, Degrees> fov_;
  pointcloud_callback_t pointcloud_callback_;
  NebulaPointCloudPtr current_scan_cloud_{std::make_shared<NebulaPointCloud>()};
  uint64_t packet_count_{0};
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DECODER_HPP
