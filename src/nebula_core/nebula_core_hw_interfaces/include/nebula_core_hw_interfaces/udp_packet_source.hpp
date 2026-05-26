// Copyright 2026 TIER IV, Inc.
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

#ifndef NEBULA_UDP_PACKET_SOURCE_HPP
#define NEBULA_UDP_PACKET_SOURCE_HPP

#include <nebula_core_hw_interfaces/connections/udp.hpp>
#include <nebula_core_hw_interfaces/packet_source.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{
class UdpPacketSource : public PacketSource
{
public:
  UdpPacketSource();
  ~UdpPacketSource() override;

  void configure(const std::string & host_ip, uint16_t port);
  void set_packet_callback(SensorPacketCallback callback) override;
  void set_error_callback(SensorErrorCallback callback) override;
  void start() override;
  void stop() override;
  bool is_running() const override;

private:
  void on_udp_packet(
    std::vector<uint8_t> & data, const connections::UdpSocket::RxMetadata & metadata);

  std::string host_ip_;
  uint16_t port_{0};
  SensorPacketCallback callback_;
  SensorErrorCallback error_callback_;
  std::unique_ptr<connections::UdpSocket> socket_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_UDP_PACKET_SOURCE_HPP
