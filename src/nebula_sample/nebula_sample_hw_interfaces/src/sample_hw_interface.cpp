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
  // Constructor - initialize any member variables if needed
}

Status SampleHwInterface::sensor_interface_start()
{
  // Implementation Items: Implement sensor interface startup
  // 1. Create UDP socket using connections::UdpSocket
  // 2. Bind to the port specified in sensor_configuration_
  // 3. Start async receive loop
  // 4. When packets arrive, call cloud_packet_callback_ with the packet data
  // 5. Optionally: send HTTP/TCP command to sensor to start scanning

  // Example (pseudo-code):
  // udp_socket_ = std::make_shared<connections::UdpSocket>();
  // udp_socket_->open();
  // udp_socket_->bind(sensor_configuration_->host_ip, sensor_configuration_->data_port);
  // udp_socket_->asyncReceive(cloud_packet_callback_);

  return Status::OK;
}

Status SampleHwInterface::sensor_interface_stop()
{
  // Implementation Items: Implement sensor interface shutdown
  // 1. Stop the receive loop
  // 2. Close UDP socket(s)
  // 3. Optionally: send command to sensor to stop scanning

  // Example (pseudo-code):
  // if (udp_socket_) {
  //   udp_socket_->close();
  // }

  return Status::OK;
}

Status SampleHwInterface::set_sensor_configuration(
  std::shared_ptr<const SampleSensorConfiguration> sensor_configuration)
{
  // Store the sensor configuration for later use
  sensor_configuration_ = sensor_configuration;
  return Status::OK;
}

Status SampleHwInterface::register_scan_callback(connections::UdpSocket::callback_t scan_callback)
{
  // Store the callback to be called when packets arrive
  cloud_packet_callback_ = scan_callback;
  return Status::OK;
}

}  // namespace nebula::drivers
