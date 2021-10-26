#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include <string>
#include <vector>
#include "udp_driver/udp_driver.hpp"
#include "configuration_base.hpp"
#include "status.hpp"

namespace nebula
{
namespace drivers
{
class NebulaHwInterfaceBase
  {
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase&) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase&) = delete;

  NebulaHwInterfaceBase(SensorConfiguration sensor_configuration, CalibrationConfiguration calibration_configuration);
  private:
    virtual STATUS ReceiveCloudPacketCallback(const std::vector<uint8_t> &buffer);

  public:
    virtual STATUS CloudInterfaceStart(const SensorConfiguration &sensor_configuration);
    virtual STATUS CloudInterfaceStop() = 0;
    // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor specific.

    virtual STATUS SetSensorConfiguration(const SensorConfiguration &sensor_configuration) = 0;
    virtual STATUS GetSensorConfiguration(SensorConfiguration &sensor_configuration);
    virtual STATUS GetCalibrationConfiguration(CalibrationConfiguration &calibration_configuration);
  };

}  // namespace drivers
}  // namespace nebula

#endif //NEBULA_HW_INTERFACE_BASE_H
