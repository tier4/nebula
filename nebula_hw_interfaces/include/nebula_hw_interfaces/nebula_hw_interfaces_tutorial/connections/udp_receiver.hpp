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

#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_udp_driver/udp_driver.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers::connections
{

class UdpReceiver
{
public:
  using callback_t = typename std::function<void(std::vector<uint8_t> & buffer)>;

  /// @brief Create a receiving UDP connection and forward received packets to `packet_callback`
  /// @param packet_callback The function getting called on each received packet
  UdpReceiver(
    std::shared_ptr<nebula::drivers::loggers::Logger> logger, const std::string & host_ip,
    uint16_t host_port, const callback_t & packet_callback)
  : logger_(logger), udp_driver_(ctx_)
  {
    try {
      udp_driver_.init_receiver(host_ip, host_port);
      udp_driver_.receiver()->open();
      udp_driver_.receiver()->bind();

      udp_driver_.receiver()->asyncReceive(packet_callback);
    } catch (const std::exception & ex) {
      throw std::runtime_error(std::string("Could not open UDP socket: ") + ex.what());
    }
  }

private:
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;

  ::drivers::common::IoContext ctx_{1};
  ::drivers::udp_driver::UdpDriver udp_driver_;
};

}  // namespace nebula::drivers::connections
