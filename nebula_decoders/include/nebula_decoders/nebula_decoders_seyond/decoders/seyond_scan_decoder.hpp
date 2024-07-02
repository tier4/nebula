#ifndef NEBULA_WS_SEYOND_SCAN_DECODER_HPP
#define NEBULA_WS_SEYOND_SCAN_DECODER_HPP

#include "nebula_common/point_types.hpp"
#include "nebula_common/seyond/seyond_common.hpp"

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Seyond LiDAR decoder
class SeyondScanDecoder
{
public:
  SeyondScanDecoder(SeyondScanDecoder && c) = delete;
  SeyondScanDecoder & operator=(SeyondScanDecoder && c) = delete;
  SeyondScanDecoder(const SeyondScanDecoder & c) = delete;
  SeyondScanDecoder & operator=(const SeyondScanDecoder & c) = delete;

  virtual ~SeyondScanDecoder() = default;
  SeyondScanDecoder() = default;

  /// @brief Parses NebulaPacket and add its points to the point cloud
  /// @param packet The incoming NebulaPacket
  /// @return The last azimuth processed
  virtual int unpack(const std::vector<uint8_t> & packet) = 0;

  /// @brief Indicates whether one full scan is ready
  /// @return Whether a scan is ready
  virtual bool hasScanned() = 0;

  /// @brief Returns the point cloud and timestamp of the last scan
  /// @return A tuple of point cloud and timestamp in nanoseconds
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_SEYOND_SCAN_DECODER_HPP
