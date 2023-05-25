#ifndef NEBULA_HW_MONITOR_WRAPPER_BASE_H
#define NEBULA_HW_MONITOR_WRAPPER_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
/// @brief Base class for hardware monitor ros wrapper of each LiDAR
class NebulaHwMonitorWrapperBase
{
public:
  NebulaHwMonitorWrapperBase() = default;

  NebulaHwMonitorWrapperBase(NebulaHwMonitorWrapperBase && c) = delete;
  NebulaHwMonitorWrapperBase & operator=(NebulaHwMonitorWrapperBase && c) = delete;
  NebulaHwMonitorWrapperBase(const NebulaHwMonitorWrapperBase & c) = delete;
  NebulaHwMonitorWrapperBase & operator=(const NebulaHwMonitorWrapperBase & c) = delete;

  /// @brief Start monitoring (not used)
  /// @return Resulting status
  virtual Status MonitorStart() = 0;

  /// @brief Stop monitoring (not used)
  /// @return Resulting status
  virtual Status MonitorStop() = 0;

  /// @brief Shutdown (not used)
  /// @return Resulting status
  virtual Status Shutdown() = 0;

protected:
  /// @brief Virtual function for initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HW_MONITOR_WRAPPER_BASE_H
