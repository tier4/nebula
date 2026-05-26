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

// # --8<-- [start:include]
#include "nebula_core_hw_interfaces/connections/udp.hpp"
// # --8<-- [end:include]

#include <cstdint>
#include <vector>

int main()
{
  // # --8<-- [start:usage]
  using nebula::drivers::connections::UdpSocket;
  auto socket = UdpSocket::Builder("192.168.1.10", 9000)
                  .set_socket_buffer_size(10'000'000)
                  .limit_to_sender("192.168.10.20", 7000)
                  .bind();

  socket.subscribe([](std::vector<uint8_t> & data, const UdpSocket::RxMetadata & metadata) {
    (void)data;
    (void)metadata;
    // Process received data and metadata here. This callback will be executed in the socket's
    // receive thread.
  });

  socket.unsubscribe();
  // # --8<-- [end:usage]
  return 0;
}
