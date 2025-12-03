// Copyright 2024 TIER IV, Inc.
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

#ifndef NEBULA_SAMPLE_HW_INTERFACE_HPP
#define NEBULA_SAMPLE_HW_INTERFACE_HPP

#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp>
#include <nebula_sample_common/sample_common.hpp>

#include <memory>
#include <vector>

namespace nebula::drivers
{

class SampleHwInterface
{
public:
  SampleHwInterface();

  Status sensor_interface_start();
  Status sensor_interface_stop();

  Status set_sensor_configuration(
    std::shared_ptr<const SampleSensorConfiguration> sensor_configuration);

  Status register_scan_callback(connections::UdpSocket::callback_t scan_callback);

private:
  std::shared_ptr<const SampleSensorConfiguration> sensor_configuration_;
  connections::UdpSocket::callback_t cloud_packet_callback_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_HW_INTERFACE_HPP
