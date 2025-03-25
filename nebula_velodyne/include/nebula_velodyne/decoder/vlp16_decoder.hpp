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

#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <array>
#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers::vlp16
{
constexpr uint32_t max_points = 300000;
/// @brief Velodyne LiDAR decoder (VLP16)
class Vlp16Decoder : public VelodyneScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit Vlp16Decoder(
    const std::shared_ptr<const drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);
  /// @brief Parsing and shaping VelodynePacket
  /// @param velodyne_packet
  void unpack(const std::vector<uint8_t> & packet, double packet_seconds) override;
  /// @brief Calculation of points in each packet
  /// @return # of points
  int points_per_packet() override;
  /// @brief Get the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  /// @brief Resetting point cloud buffer
  void reset_pointcloud(double time_stamp) override;
  /// @brief Resetting overflowed point cloud buffer
  void reset_overflow(double time_stamp) override;

private:
  /// @brief Parsing VelodynePacket based on packet structure
  /// @param velodyne_packet
  /// @return Resulting flag
  bool parse_packet(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  float sin_rot_table_[g_rotation_max_units];
  float cos_rot_table_[g_rotation_max_units];
  float rotation_radians_[g_rotation_max_units];
  int phase_;
  int max_pts_;
  double last_block_timestamp_;
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace nebula::drivers::vlp16
