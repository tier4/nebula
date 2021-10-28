#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include <string>
#include <vector>
#include "common/configuration_base.hpp"
#include "common/nebula_status.hpp"
#include "udp_driver/udp_driver.hpp"

namespace nebula
{
namespace drivers
{
class NebulaHwInterfaceBase
{
protected:
  SensorConfigurationBase sensor_configuration_;
  CalibrationConfigurationBase calibration_configuration_;
  virtual Status ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) = 0;

public:
  NebulaHwInterfaceBase(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase & operator=(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase & c) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase & c) = delete;

  NebulaHwInterfaceBase() = default;
  NebulaHwInterfaceBase(
    SensorConfigurationBase & sensor_configuration,
    CalibrationConfigurationBase & calibration_configuration);

  virtual Status CloudInterfaceStart(const SensorConfigurationBase & sensor_configuration) = 0;
  virtual Status CloudInterfaceStop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor specific.

  virtual Status SetSensorConfiguration(const SensorConfigurationBase & sensor_configuration) = 0;
  virtual Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) = 0;
  virtual Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) = 0;
};
NebulaHwInterfaceBase::NebulaHwInterfaceBase(
  SensorConfigurationBase & sensor_configuration,
  CalibrationConfigurationBase & calibration_configuration)
: sensor_configuration_(sensor_configuration), calibration_configuration_(calibration_configuration)
{
}

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_HW_INTERFACE_BASE_H
