#ifndef NEBULA_HW_INTERFACE_WRAPPER_BASE_H
#define NEBULA_HW_INTERFACE_WRAPPER_BASE_H

#include <string>
#include <vector>

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

namespace nebula
{
namespace ros
{
class NebulaHwInterfaceWrapperBase
{
public:
  NebulaHwInterfaceWrapperBase() = default;

  NebulaHwInterfaceWrapperBase(NebulaHwInterfaceWrapperBase && c) = delete;
  NebulaHwInterfaceWrapperBase & operator=(NebulaHwInterfaceWrapperBase && c) = delete;
  NebulaHwInterfaceWrapperBase(const NebulaHwInterfaceWrapperBase & c) = delete;
  NebulaHwInterfaceWrapperBase & operator=(const NebulaHwInterfaceWrapperBase & c) = delete;

  virtual Status StreamStart() = 0;
  virtual Status StreamStop() = 0;
  virtual Status Shutdown() = 0;

protected:
  virtual Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;
  virtual void ReceiveScanDataCallback(std::unique_ptr<std::vector<std::vector<uint8_t>>> scan_buffer) = 0;
  //  void SendDataPacket(const std::vector<uint8_t> &buffer);        // Ideally this will be implemented as specific funtions, GetFanStatus, GetEchoMode
};

}  // namespace ros
}  // namespace nebula

#endif  //NEBULA_HW_INTERFACE_WRAPPER_BASE_H
