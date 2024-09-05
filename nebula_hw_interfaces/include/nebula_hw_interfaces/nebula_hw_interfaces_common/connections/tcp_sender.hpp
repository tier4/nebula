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

#include <boost_tcp_driver/tcp_socket.hpp>

#include <boost/asio/buffer.hpp>
#include <boost/asio/completion_condition.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class TcpSender : public WritableByteStream
{
public:
  TcpSender(const std::string & sensor_ip, uint16_t sensor_port)
  : io_service_(1), socket_(io_service_)
  {
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(
      sensor_ip, std::to_string(static_cast<int>(sensor_port)));
    auto endpoint_iterator = resolver.resolve(query);
    boost::asio::connect(socket_, endpoint_iterator);
  }

  void write(std::vector<uint8_t> & bytes) override
  {
    boost::asio::write(socket_, boost::asio::buffer(bytes), boost::asio::transfer_all());
  }

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
};

}  // namespace nebula::drivers::connections
