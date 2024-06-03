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

#ifndef NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
#define NEBULA_WS_VELODYNE_SCAN_DECODER_HPP

#include <nebula_common/point_types.hpp>
#include <nebula_common/velodyne/velodyne_calibration_decoder.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <rclcpp/rclcpp.hpp>

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <boost/format.hpp>

#include <angles/angles.h>
#include <pcl/point_cloud.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
/// @brief Base class for Velodyne LiDAR decoder
class VelodyneScanDecoder
{
private:
  size_t processed_packets_{0};
  uint32_t prev_packet_first_azm_phased_{0};
  bool has_scanned_{false};

protected:
  /// @brief Checks if the current packet completes the ongoing scan.
  /// TODO: this has been moved from velodyne_hw_interface.cpp and is a temporary solution until
  /// the Velodyne decoder uses the same structure as Hesai/Robosense
  /// @param packet The packet buffer to extract azimuths from
  /// @param packet_seconds The packet's timestamp in seconds, including the sub-second part
  /// @param phase The sensor's scan phase used for scan cutting
  void check_and_handle_scan_complete(
    const std::vector<uint8_t> & packet, double packet_seconds, const uint32_t phase)
  {
    if (has_scanned_) {
      processed_packets_ = 0;
      reset_pointcloud(packet_seconds);
    }

    has_scanned_ = false;
    processed_packets_++;

    uint32_t packet_first_azm = packet[g_offset_first_azimuth];   // lower word of azimuth block 0
    packet_first_azm |= packet[g_offset_first_azimuth + 1] << 8;  // higher word of azimuth block 0

    uint32_t packet_last_azm = packet[g_offset_last_azimuth];
    packet_last_azm |= packet[g_offset_last_azimuth + 1] << 8;

    const uint32_t max_azi = 360 * g_degree_subdivisions;

    uint32_t packet_first_azm_phased = (max_azi + packet_first_azm - phase) % max_azi;
    uint32_t packet_last_azm_phased = (max_azi + packet_last_azm - phase) % max_azi;

    has_scanned_ =
      processed_packets_ > 1 && (packet_last_azm_phased < packet_first_azm_phased ||
                                 packet_first_azm_phased < prev_packet_first_azm_phased_);

    prev_packet_first_azm_phased_ = packet_first_azm_phased;
  }

  /// @brief Decoded point cloud
  drivers::NebulaPointCloudPtr scan_pc_;
  /// @brief Point cloud overflowing from one cycle
  drivers::NebulaPointCloudPtr overflow_pc_;

  double dual_return_distance_threshold_{};  // Velodyne does this internally, this will not be
                                             // implemented here
  double scan_timestamp_{};

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_configuration_;
  /// @brief Calibration for this decoder
  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_configuration_;

public:
  VelodyneScanDecoder(VelodyneScanDecoder && c) = delete;
  VelodyneScanDecoder & operator=(VelodyneScanDecoder && c) = delete;
  VelodyneScanDecoder(const VelodyneScanDecoder & c) = delete;
  VelodyneScanDecoder & operator=(const VelodyneScanDecoder & c) = delete;

  virtual ~VelodyneScanDecoder() = default;
  VelodyneScanDecoder() = default;

  /// @brief Virtual function for parsing and shaping VelodynePacket
  /// @param pandar_packet
  virtual void unpack(const std::vector<uint8_t> & packet, double packet_seconds) = 0;
  /// @brief Virtual function for parsing VelodynePacket based on packet structure
  /// @param pandar_packet
  /// @return Resulting flag
  virtual bool parse_packet(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) = 0;

  /// @brief Virtual function for getting the flag indicating whether one cycle is ready
  /// @return Readied
  virtual bool hasScanned() = 0;

  /// @brief Virtual function for getting the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
  /// @brief Resetting point cloud buffer
  virtual void reset_pointcloud(double time_stamp) = 0;
  /// @brief Resetting overflowed point cloud buffer
  virtual void reset_overflow(double time_stamp) = 0;
};

}  // namespace nebula::drivers

#endif  // NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
