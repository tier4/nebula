#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <map>
#include <string>
#include <optional>

namespace nebula
{
namespace drivers
{
template <typename InfoPacketT>
class SensorInfoBase
{
private:
  static constexpr uint8_t DUAL_RETURN_FLAG = 0x00;
  static constexpr uint8_t STRONGEST_RETURN_FLAG = 0x04;
  static constexpr uint8_t LAST_RETURN_FLAG = 0x05;
  static constexpr uint8_t FIRST_RETURN_FLAG = 0x06;

public:
  typedef InfoPacketT packet_t;

  virtual std::map<std::string, std::string> getSensorInfo(const InfoPacketT & info_packet) const = 0;

  virtual ReturnMode getReturnMode(const InfoPacketT & info_packet) const
  {
    switch (getFieldValue(info_packet.return_mode)) {
      case DUAL_RETURN_FLAG:
        return ReturnMode::DUAL;
      case STRONGEST_RETURN_FLAG:
        return ReturnMode::SINGLE_STRONGEST;
      case LAST_RETURN_FLAG:
        return ReturnMode::SINGLE_LAST;
      case FIRST_RETURN_FLAG:
        return ReturnMode::SINGLE_FIRST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }

  virtual std::optional<RobosenseCalibrationConfiguration> getSensorCalibration(
    const InfoPacketT & info_packet) const = 0;

  virtual bool getSyncStatus(const InfoPacketT & info_packet) const = 0;
};

}  // namespace drivers
}  // namespace nebula