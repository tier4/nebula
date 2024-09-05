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

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"

#include <boost_udp_driver/udp_driver.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class UdpReceiver : public ObservableByteStream
{
public:
  /// @brief Create a receiving UDP connection and forward received packets to the registered
  /// callback (if any)
  UdpReceiver(const std::string & host_ip, uint16_t host_port) : ctx_(1), udp_driver_(ctx_)
  {
    try {
      udp_driver_.init_receiver(host_ip, host_port);
      udp_driver_.receiver()->open();
      udp_driver_.receiver()->bind();

      udp_driver_.receiver()->asyncReceive([&](const auto & bytes) { onReceive(bytes); });
    } catch (const std::exception & ex) {
      throw std::runtime_error(std::string("Could not open UDP socket: ") + ex.what());
    }
  }

  void registerBytesCallback(callback_t callback) override
  {
    std::lock_guard lock(mtx_callback_);
    callback_ = std::move(callback);
  }

private:
  void onReceive(const std::vector<uint8_t> & bytes)
  {
    std::lock_guard lock(mtx_callback_);
    if (!callback_) return;
    callback_(bytes);
  }

  ::drivers::common::IoContext ctx_;
  ::drivers::udp_driver::UdpDriver udp_driver_;
  std::mutex mtx_callback_;
  callback_t callback_{};
};

}  // namespace nebula::drivers::connections
