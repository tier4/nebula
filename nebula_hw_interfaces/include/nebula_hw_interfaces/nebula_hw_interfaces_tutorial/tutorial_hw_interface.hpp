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

#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/my_protocol.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/udp_receiver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/tutorial/tutorial_common.hpp>
#include <nebula_common/util/expected.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <utility>
#include <vector>
namespace nebula::drivers
{

using TutorialConfig = typename nebula::HesaiConfig;
constexpr auto return_mode_from_int = nebula::drivers::return_mode_from_int_hesai;
constexpr auto int_from_return_mode = nebula::drivers::int_from_return_mode_hesai;

const int g_tcp_port = 9347;
const uint8_t g_ptc_command_get_config_info = 0x08;
const uint8_t g_ptc_command_set_spin_rate = 0x17;
const uint8_t g_ptc_command_set_sync_angle = 0x18;
const uint8_t g_ptc_command_set_return_mode = 0x1e;

class TutorialHwInterface
{
private:
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;
  connections::MyProtocolConnection my_protocol_connection_;
  connections::UdpReceiver udp_receiver_;
  connections::UdpReceiver::callback_t cloud_packet_callback_ = nullptr;

public:
  TutorialHwInterface(
    const std::shared_ptr<loggers::Logger> & logger,
    const std::shared_ptr<const TutorialSensorConfiguration> & sensor_configuration);

  /// @brief Retrieve the configuration from the sensor synchronously
  /// @return The retrieved config, or throw if an error occurs
  TutorialConfig get_config();

  /// @brief Get current config from sensor, compare with given config, and send config updates to
  /// sensor where needed
  /// @param updated The updated configuration that should be sent to the sensor
  /// @return Resulting status
  Status compare_and_send_config(const TutorialSensorConfiguration & updated);

  /// @brief Set the RPM of the sensor
  /// @param rpm Spin rate
  /// @return Resulting status
  Status set_spin_rate(uint16_t rpm);

  /// @brief Register a callback that gets called for each received UDP packet
  Status register_on_sensor_packet_callback(connections::UdpReceiver::callback_t scan_callback)
  {
    cloud_packet_callback_ = std::move(scan_callback);
    return Status::OK;
  }

private:
  void on_sensor_packet(std::vector<uint8_t> & buffer) const
  {
    if (!cloud_packet_callback_) {
      return;
    }

    cloud_packet_callback_(buffer);
  }
};
}  // namespace nebula::drivers
