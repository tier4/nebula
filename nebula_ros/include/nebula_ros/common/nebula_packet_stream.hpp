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

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "nebula_msgs/msg/detail/nebula_packets__struct.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace nebula::ros
{

class NebulaPacketStream : public drivers::connections::ObservableByteStream
{
public:
  NebulaPacketStream() = default;

  void register_bytes_callback(callback_t callback) override
  {
    std::lock_guard lock(mtx_callback_);
    callback_ = std::move(callback);
  }

  void on_nebula_packets(std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets)
  {
    std::lock_guard lock(mtx_callback_);
    if (!callback_) return;

    for (auto & packet : packets->packets) {
      callback_(packet.data);
    }
  }

private:
  std::mutex mtx_callback_;
  callback_t callback_;
};

}  // namespace nebula::ros
