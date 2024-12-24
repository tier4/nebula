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

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/tcp.hpp"

#include <gmock/gmock.h>

#include <string>
#include <vector>

namespace nebula::drivers::connections
{

class MockTcpSocket : public AbstractTcpSocket
{
public:
  MOCK_METHOD(
    void, init,
    (const std::string & host_ip, uint16_t host_port, const std::string & remote_ip,
     uint16_t remote_port),
    (override));

  MOCK_METHOD(void, bind, (), (override));

  MOCK_METHOD(void, close, (), (override));

  MOCK_METHOD(
    void, async_ptc_request,
    (std::vector<uint8_t> & ptc_packet, header_callback_t cb_header, payload_callback_t cb_payload,
     completion_callback_t cb_completion),
    (override));
};

}  // namespace nebula::drivers::connections
