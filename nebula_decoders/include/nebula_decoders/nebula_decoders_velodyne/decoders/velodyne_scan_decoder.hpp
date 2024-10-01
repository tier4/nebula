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

namespace nebula
{
namespace drivers
{
/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const double ROTATION_RESOLUTION = 0.01;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

static const size_t RETURN_MODE_INDEX = 1204;

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Return Modes **/
static const uint16_t RETURN_MODE_STRONGEST = 55;
static const uint16_t RETURN_MODE_LAST = 56;
static const uint16_t RETURN_MODE_DUAL = 57;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_DURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]

/** Special Defines for VLP32 support **/
static const float VLP32_CHANNEL_DURATION = 2.304f;  // [µs]
static const float VLP32_SEQ_DURATION = 55.296f;     // [µs]

/** Special Definitions for VLS128 support **/
static const float VLP128_DISTANCE_RESOLUTION = 0.004f;  // [m]

/** Special Definitions for VLS128 support **/
// These are used to detect which bank of 32 lasers is in this block
static const uint16_t VLS128_BANK_1 = 0xeeff;
static const uint16_t VLS128_BANK_2 = 0xddff;
static const uint16_t VLS128_BANK_3 = 0xccff;
static const uint16_t VLS128_BANK_4 = 0xbbff;

static const float VLS128_CHANNEL_DURATION =
  2.665f;  // [µs] Channels corresponds to one laser firing
static const float VLS128_SEQ_DURATION =
  53.3f;  // [µs] Sequence is a set of laser firings including recharging

static const size_t OFFSET_FIRST_AZIMUTH = 2;
static const size_t OFFSET_LAST_AZIMUTH = 1102;
static const uint32_t DEGREE_SUBDIVISIONS = 100;

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

/** \brief Velodyne echo types */
enum RETURN_TYPE {
  INVALID = 0,
  SINGLE_STRONGEST = 1,
  SINGLE_LAST = 2,
  DUAL_STRONGEST_FIRST = 3,
  DUAL_STRONGEST_LAST = 4,
  DUAL_WEAK_FIRST = 5,
  DUAL_WEAK_LAST = 6,
  DUAL_ONLY = 7
};

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
  void checkAndHandleScanComplete(
    const std::vector<uint8_t> & packet, double packet_seconds, const uint32_t phase)
  {
    if (has_scanned_) {
      processed_packets_ = 0;
      reset_pointcloud(packet_seconds);
    }

    has_scanned_ = false;
    processed_packets_++;

    uint32_t packet_first_azm = packet[OFFSET_FIRST_AZIMUTH];   // lower word of azimuth block 0
    packet_first_azm |= packet[OFFSET_FIRST_AZIMUTH + 1] << 8;  // higher word of azimuth block 0

    uint32_t packet_last_azm = packet[OFFSET_LAST_AZIMUTH];
    packet_last_azm |= packet[OFFSET_LAST_AZIMUTH + 1] << 8;

    const uint32_t max_azi = 360 * DEGREE_SUBDIVISIONS;

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
  virtual bool parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) = 0;

  /// @brief Virtual function for getting the flag indicating whether one cycle is ready
  /// @return Readied
  bool hasScanned() { return has_scanned_; }

  /// @brief Calculation of points in each packet
  /// @return # of points
  virtual int pointsPerPacket() = 0;

  /// @brief Virtual function for getting the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
  /// @brief Resetting point cloud buffer
  virtual void reset_pointcloud(double time_stamp) = 0;
  /// @brief Resetting overflowed point cloud buffer
  virtual void reset_overflow(double time_stamp) = 0;
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
