#ifndef NEBULA_WS_HESAI_SCAN_DECODER_HPP
#define NEBULA_WS_HESAI_SCAN_DECODER_HPP

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/point_types.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Hesai LiDAR decoder
class HesaiScanDecoder
{
public:
  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;

  /// @brief Parses PandarPacket and add its points to the point cloud
  /// @param pandar_packet The incoming PandarPacket
  /// @return The last azimuth processed
  virtual int unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;

  /// @brief Indicates whether one full scan is ready
  /// @return Whether a scan is ready
  virtual bool hasScanned() = 0;

  /// @brief Returns the point cloud and timestamp of the last scan
  /// @return A tuple of point cloud and timestamp in nanoseconds
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_HESAI_SCAN_DECODER_HPP
