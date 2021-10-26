#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include <string>
#include <vector>
#include "configuration_base.hpp"
#include "status.hpp"
#include "udp_driver/udp_driver.hpp"

namespace nebula
{
namespace drivers
{
class NebulaHwInterfaceBase
{
  NebulaHwInterfaceBase(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase & operator=(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase & c) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase & c) = delete;

  NebulaHwInterfaceBase(
    SensorConfigurationBase & sensor_configuration,
    CalibrationConfigurationBase & calibration_configuration);

private:
  virtual STATUS ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer);

public:
  virtual STATUS CloudInterfaceStart(const SensorConfigurationBase & sensor_configuration);
  virtual STATUS CloudInterfaceStop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor specific.

  virtual STATUS SetSensorConfiguration(const SensorConfigurationBase & sensor_configuration) = 0;
  virtual STATUS GetSensorConfiguration(SensorConfigurationBase & sensor_configuration);
  virtual STATUS GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration);
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_HW_INTERFACE_BASE_H
