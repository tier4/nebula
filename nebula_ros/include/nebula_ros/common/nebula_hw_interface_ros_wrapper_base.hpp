// Copyright 2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

  /// @brief Start point cloud streaming (Call SensorInterfaceStart of HwInterface)
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
