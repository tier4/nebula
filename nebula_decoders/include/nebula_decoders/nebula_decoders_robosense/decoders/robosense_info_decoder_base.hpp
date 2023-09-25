#pragma once

namespace nebula
{
namespace drivers
{

class RobosenseInfoDecoderBase
{
public:
  RobosenseInfoDecoderBase(RobosenseInfoDecoderBase && c) = delete;
  RobosenseInfoDecoderBase & operator=(RobosenseInfoDecoderBase && c) = delete;
  RobosenseInfoDecoderBase(const RobosenseInfoDecoderBase & c) = delete;
  RobosenseInfoDecoderBase & operator=(const RobosenseInfoDecoderBase & c) = delete;

  virtual ~RobosenseInfoDecoderBase() = default;
  RobosenseInfoDecoderBase() = default;

  /// @brief Parses PandarPacket and add its points to the point cloud
  /// @param pandar_packet The incoming PandarPacket
  /// @return The last azimuth processed
  virtual bool parsePacket(const std::vector<uint8_t> & raw_packet) = 0;

  virtual std::map<std::string, std::string> getSensorInfo() = 0;
};

}  // namespace drivers
}  // namespace nebula