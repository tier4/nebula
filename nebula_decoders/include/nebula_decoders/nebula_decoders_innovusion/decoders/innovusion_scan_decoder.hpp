#ifndef NEBULA_WS_INNOVUSION_SCAN_DECODER_HPP
#define NEBULA_WS_INNOVUSION_SCAN_DECODER_HPP

#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/point_types.hpp"

#include "innovusion_msgs/msg/innovusion_packet.hpp"
#include "innovusion_msgs/msg/innovusion_scan.hpp"

#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Innovusion LiDAR decoder
class InnovusionScanDecoder
{
public:
  InnovusionScanDecoder(InnovusionScanDecoder && c) = delete;
  InnovusionScanDecoder & operator=(InnovusionScanDecoder && c) = delete;
  InnovusionScanDecoder(const InnovusionScanDecoder & c) = delete;
  InnovusionScanDecoder & operator=(const InnovusionScanDecoder & c) = delete;

  virtual ~InnovusionScanDecoder() = default;
  InnovusionScanDecoder() = default;

  /// @brief Parses InnovusionPacket and add its points to the point cloud
  /// @param innovusion_packet The incoming InnovusionPacket
  /// @return The last azimuth processed
  virtual int unpack(const innovusion_msgs::msg::InnovusionPacket & innovusion_packet) = 0;

  /// @brief Returns the point cloud and timestamp of the last scan
  /// @return A tuple of point cloud and timestamp in nanoseconds
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_INNOVUSION_SCAN_DECODER_HPP
