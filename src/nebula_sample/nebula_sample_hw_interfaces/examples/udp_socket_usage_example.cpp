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

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <cstdint>
#include <vector>

namespace nebula::drivers::examples
{

void udp_socket_usage_example()
{
  // This file is intended to be compiled (via CMake) to keep documentation snippets up-to-date.
  // It does not need to be executed.
  auto socket = connections::UdpSocket::Builder("0.0.0.0", static_cast<uint16_t>(0)).bind();

  socket.subscribe(
    [](const std::vector<uint8_t> & data, const connections::UdpSocket::RxMetadata & metadata) {
      (void)data;
      (void)metadata;
    });

  socket.unsubscribe();
}

}  // namespace nebula::drivers::examples
