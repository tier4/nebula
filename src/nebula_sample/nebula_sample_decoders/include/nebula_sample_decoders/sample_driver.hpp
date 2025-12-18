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

#ifndef NEBULA_SAMPLE_DRIVER_HPP
#define NEBULA_SAMPLE_DRIVER_HPP

#include "nebula_core_common/nebula_status.hpp"
#include "nebula_sample_common/sample_common.hpp"
#include "nebula_sample_decoders/decoders/sample_scan_decoder.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Main driver class for the Sample LiDAR
/// @details This is the high-level interface for the sensor driver.
/// It manages the decoder and provides the main API for:
/// - Initializing the driver with sensor configuration
/// - Processing incoming packets
/// - Registering callbacks for point clouds
/// - Querying driver status
class SampleDriver
{
public:
  /// @brief Constructor
  /// @param sensor_configuration Sensor settings
  /// @details Initializes the decoder with the given configuration
  explicit SampleDriver(
    const std::shared_ptr<const SampleSensorConfiguration> & sensor_configuration);

  /// @brief Get the current driver status
  /// @return Status indicating if the driver is ready to process packets
  /// @details Returns OK if initialized properly, error status otherwise
  Status get_status();

  /// @brief Process a single cloud packet
  /// @param packet Raw UDP packet data
  /// @return PacketDecodeResult with metadata or error
  /// @details This is the main entry point for packet processing.
  /// Called by the HW interface when a packet is received.
  /// Delegates to the decoder's unpack() method.
  PacketDecodeResult parse_cloud_packet(const std::vector<uint8_t> & packet);

  /// @brief Register callback for complete point clouds
  /// @param pointcloud_cb Function to call when a scan is complete
  /// @details The callback receives the decoded point cloud and timestamp
  void set_pointcloud_callback(SampleScanDecoder::pointcloud_callback_t pointcloud_cb);

private:
  std::shared_ptr<SampleScanDecoder> scan_decoder_;  ///< Decoder instance
  Status driver_status_;                             ///< Current driver status
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DRIVER_HPP
