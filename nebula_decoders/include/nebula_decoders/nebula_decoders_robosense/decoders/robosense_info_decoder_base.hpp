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

  /// @brief Parses DIFOP and add its telemetry
  /// @param raw_packet The incoming DIFOP packet
  /// @return Whether the packet was parsed successfully
  virtual bool parsePacket(const std::vector<uint8_t> & raw_packet) = 0;

  /// @brief Get the sensor telemetry
  /// @return The sensor telemetry
  virtual std::map<std::string, std::string> getSensorInfo() = 0;

  /// @brief Get the laser return mode
  /// @return The laser return mode
  virtual ReturnMode getReturnMode() = 0;

  /// @brief Get sensor calibration
  /// @return The sensor calibration
  virtual RobosenseCalibrationConfiguration getSensorCalibration() = 0;
};

}  // namespace drivers
}  // namespace nebula