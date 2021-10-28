#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include <stdexcept>
#include <string>
#include <vector>
#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"
#include "udp_driver/udp_driver.hpp"

namespace nebula
{
namespace drivers
{
class NebulaHwInterfaceBase
{
protected:
  /**
   * Callback function to receive the Cloud Packet data from the UDP Driver
   * @param buffer buffer containing the data received from the UDP socket
   * @return Status::OK if no error occured.
   */
  virtual Status ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) = 0;

public:
  NebulaHwInterfaceBase(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase & operator=(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase & c) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase & c) = delete;

  NebulaHwInterfaceBase() = default;

  virtual Status CloudInterfaceStart() = 0;
  virtual Status CloudInterfaceStop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor specific.

  virtual Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) = 0;
  virtual Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) = 0;
  virtual Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) = 0;
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_HW_INTERFACE_BASE_H
