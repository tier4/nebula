#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include <string>
#include <vector>
#include "configuration_base.hpp"
#include "nebula_status.hpp"
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
  virtual Status ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer);

public:
  virtual Status CloudInterfaceStart(const SensorConfigurationBase & sensor_configuration);
  virtual Status CloudInterfaceStop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor specific.

  virtual Status SetSensorConfiguration(const SensorConfigurationBase & sensor_configuration) = 0;
  virtual Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration);
  virtual Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration);
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_HW_INTERFACE_BASE_H
