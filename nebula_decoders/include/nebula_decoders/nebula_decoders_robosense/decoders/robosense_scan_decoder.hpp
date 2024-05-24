#pragma once

#include "nebula_common/point_types.hpp"
#include "nebula_common/robosense/robosense_common.hpp"

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Robosense LiDAR decoder
class RobosenseScanDecoder
{
public:
  RobosenseScanDecoder(RobosenseScanDecoder && c) = delete;
  RobosenseScanDecoder & operator=(RobosenseScanDecoder && c) = delete;
  RobosenseScanDecoder(const RobosenseScanDecoder & c) = delete;
  RobosenseScanDecoder & operator=(const RobosenseScanDecoder & c) = delete;

  virtual ~RobosenseScanDecoder() = default;
  RobosenseScanDecoder() = default;

  /// @brief Parses RobosensePacket and add its points to the point cloud
  /// @param msop_packet The incoming MsopPacket
  /// @return The last azimuth processed
  virtual int unpack(const std::vector<uint8_t> & msop_packet) = 0;

  /// @brief Indicates whether one full scan is ready
  /// @return Whether a scan is ready
  virtual bool hasScanned() = 0;

  /// @brief Returns the point cloud and timestamp of the last scan
  /// @return A tuple of point cloud and timestamp in nanoseconds
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() = 0;
};

}  // namespace drivers
}  // namespace nebula
