// Copyright 2025 TIER IV, Inc.
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

#include "nebula_sample_hw_interfaces/sample_hw_interface.hpp"

#include <memory>

namespace nebula::drivers
{

SampleHwInterface::SampleHwInterface()
{
}

Status SampleHwInterface::sensor_interface_start()
{
  return Status::OK;
}

Status SampleHwInterface::sensor_interface_stop()
{
  return Status::OK;
}

Status SampleHwInterface::set_sensor_configuration(
  std::shared_ptr<const SampleSensorConfiguration> sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;
  return Status::OK;
}

Status SampleHwInterface::register_scan_callback(connections::UdpSocket::callback_t scan_callback)
{
  cloud_packet_callback_ = scan_callback;
  return Status::OK;
}

}  // namespace nebula::drivers
