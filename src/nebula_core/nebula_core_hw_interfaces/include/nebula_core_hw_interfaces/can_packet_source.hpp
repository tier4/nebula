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

#ifndef NEBULA_CAN_PACKET_SOURCE_HPP
#define NEBULA_CAN_PACKET_SOURCE_HPP

#include <nebula_core_hw_interfaces/connections/can.hpp>
#include <nebula_core_hw_interfaces/packet_source.hpp>

#include <memory>
#include <string>

namespace nebula::drivers
{
class CanPacketSource : public PacketSource
{
public:
  CanPacketSource();
  ~CanPacketSource() override;

  void configure(const std::string & interface_name);
  void set_packet_callback(SensorPacketCallback callback) override;
  void set_error_callback(SensorErrorCallback callback) override;
  void start() override;
  void stop() override;
  bool is_running() const override;

private:
  void on_can_frame(const canfd_frame & frame, const connections::CanSocket::RxMetadata & metadata);

  std::string interface_name_;
  SensorPacketCallback callback_;
  SensorErrorCallback error_callback_;
  std::unique_ptr<connections::CanSocket> socket_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_CAN_PACKET_SOURCE_HPP
