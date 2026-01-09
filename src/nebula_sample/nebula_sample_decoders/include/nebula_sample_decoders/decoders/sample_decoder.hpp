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

#include "nebula_sample_common/sample_common.hpp"
#include "nebula_sample_decoders/decoders/sample_scan_decoder.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Concrete implementation of the Sample packet decoder
/// @details This class implements the SampleScanDecoder interface.
/// Implement the following in this class:
/// - Parse raw UDP packets according to your sensor's protocol specification
/// - Extract point data (x, y, z, intensity, timestamp, etc.)
/// - Accumulate points until a full scan is complete
/// - Call the pointcloud callback when a scan is ready
class SampleDecoder : public SampleScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration Sensor settings (IP, ports, return mode, etc.)
  /// @details Initialize any internal state needed for decoding (e.g., point accumulation buffers)
  explicit SampleDecoder(
    const std::shared_ptr<const SampleSensorConfiguration> & sensor_configuration)
  {
    // Implementation Items: Initialize decoder state
    // - Allocate point cloud buffer
    // - Set up any lookup tables based on sensor configuration
    // - Initialize scan tracking variables
    (void)sensor_configuration;
  }

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
  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    // Implementation Items: Implement packet decoding
    // This is a stub that does nothing - replace with actual parsing logic
    (void)packet;
    return {{}, DecodeError::PACKET_PARSE_FAILED};
  }

  /// @brief Register callback for complete point clouds
  /// @param callback Function to call when a scan is complete
  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = callback;
  }

private:
  pointcloud_callback_t pointcloud_callback_;  ///< Callback for publishing scans
  // Implementation Items: Add member variables for:
  // - Point cloud accumulation buffer
  // - Previous azimuth for scan completion detection
  // - Scan timestamp
  // - Optionally: Double-buffering (two point cloud buffers) for handling scan overlap
  //   (advanced optimization for sensors with configurable FOV)
  // - Any sensor-specific state
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DECODER_HPP
