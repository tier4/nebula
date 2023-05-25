#ifndef NEBULA_HW_INTERFACE_WRAPPER_BASE_H
#define NEBULA_HW_INTERFACE_WRAPPER_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
/// @brief Base class for hardware interface ros wrapper of each LiDAR
class NebulaHwInterfaceWrapperBase
{
public:
  NebulaHwInterfaceWrapperBase() = default;

  NebulaHwInterfaceWrapperBase(NebulaHwInterfaceWrapperBase && c) = delete;
  NebulaHwInterfaceWrapperBase & operator=(NebulaHwInterfaceWrapperBase && c) = delete;
  NebulaHwInterfaceWrapperBase(const NebulaHwInterfaceWrapperBase & c) = delete;
  NebulaHwInterfaceWrapperBase & operator=(const NebulaHwInterfaceWrapperBase & c) = delete;

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  virtual Status StreamStart() = 0;

  /// @brief Stop point cloud streaming (not used)
  /// @return Resulting status
  virtual Status StreamStop() = 0;

  /// @brief Shutdown (not used)
  /// @return Resulting status
  virtual Status Shutdown() = 0;

protected:
  /// @brief Virtual function for initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;
  //  void SendDataPacket(const std::vector<uint8_t> &buffer);        // Ideally this will be
  //  implemented as specific funtions, GetFanStatus, GetEchoMode

  /// @brief Enable sensor setup during initialization and set_parameters_callback
  bool setup_sensor;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HW_INTERFACE_WRAPPER_BASE_H
