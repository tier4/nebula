#ifndef NEBULA_HW_MONITOR_WRAPPER_BASE_H
#define NEBULA_HW_MONITOR_WRAPPER_BASE_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
class NebulaHwMonitorWrapperBase
{
public:
  NebulaHwMonitorWrapperBase() = default;

  NebulaHwMonitorWrapperBase(NebulaHwMonitorWrapperBase && c) = delete;
  NebulaHwMonitorWrapperBase & operator=(NebulaHwMonitorWrapperBase && c) = delete;
  NebulaHwMonitorWrapperBase(const NebulaHwMonitorWrapperBase & c) = delete;
  NebulaHwMonitorWrapperBase & operator=(const NebulaHwMonitorWrapperBase & c) = delete;

  virtual Status MonitorStart() = 0;
  virtual Status MonitorStop() = 0;
  virtual Status Shutdown() = 0;

protected:
  virtual Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HW_MONITOR_WRAPPER_BASE_H
