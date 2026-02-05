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

#include "nebula_core_hw_interfaces/connections/udp.hpp"

#include <cstdint>
#include <vector>

int main()
{
  using nebula::drivers::connections::UdpSocket;
  // # --8<-- [start:udp_socket_usage]
  auto socket = UdpSocket::Builder("192.168.1.10", 9000)
                  .set_socket_buffer_size(10'000'000)
                  .limit_to_sender("192.168.10.20", 7000)
                  .bind();

  socket.subscribe([](const std::vector<uint8_t> & data, const UdpSocket::RxMetadata & metadata) {
    (void)data;
    (void)metadata;
  });

  socket.unsubscribe();
  // # --8<-- [end:udp_socket_usage]
  return 0;
}
