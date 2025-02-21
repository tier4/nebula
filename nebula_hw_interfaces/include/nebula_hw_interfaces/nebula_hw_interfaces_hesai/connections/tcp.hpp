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

#include <boost_tcp_driver/tcp_driver.hpp>

#include <boost/asio/io_context.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers::connections
{

class TcpError : public std::runtime_error
{
public:
  explicit TcpError(const std::string & msg) : std::runtime_error(msg) {}
};

class AbstractTcpSocket
{
public:
  using header_callback_t = std::function<void(const std::vector<uint8_t> &)>;
  using payload_callback_t = std::function<void(const std::vector<uint8_t> &)>;
  using completion_callback_t = std::function<void()>;

  virtual ~AbstractTcpSocket() = default;

  virtual void init(
    const std::string & host_ip, uint16_t host_port, const std::string & remote_ip,
    uint16_t remote_port) = 0;

  virtual void bind() = 0;

  virtual void close() = 0;

  virtual void async_ptc_request(
    std::vector<uint8_t> & ptc_packet, header_callback_t cb_header, payload_callback_t cb_payload,
    completion_callback_t cb_completion) = 0;
};

class TcpSocket : public AbstractTcpSocket
{
public:
  using callback_t = std::function<void(const std::vector<uint8_t> &)>;

  void init(
    const std::string & host_ip, uint16_t host_port, const std::string & remote_ip,
    uint16_t remote_port) override
  {
    tcp_driver_.init_socket(remote_ip, remote_port, host_ip, host_port);
  }

  void bind() override
  {
    if (!tcp_driver_.isOpen() && !tcp_driver_.open()) {
      throw TcpError("could not open TCP socket for an unknown reason");
    }
  }

  void close() override
  {
    if (tcp_driver_.isOpen()) {
      tcp_driver_.close();
    }
  }

  void async_ptc_request(
    std::vector<uint8_t> & ptc_packet, header_callback_t cb_header, payload_callback_t cb_payload,
    completion_callback_t cb_completion) override
  {
    if (tcp_driver_.GetIOContext()->stopped()) {
      tcp_driver_.GetIOContext()->restart();
    }
    bool success =
      tcp_driver_.asyncSendReceiveHeaderPayload(ptc_packet, cb_header, cb_payload, cb_completion);
    if (!success) {
      throw TcpError("sending the PTC command failed for an unknown reason");
    }
    tcp_driver_.GetIOContext()->run();
  }

private:
  ::drivers::tcp_driver::TcpDriver tcp_driver_{std::make_shared<boost::asio::io_context>(1)};
};

}  // namespace nebula::drivers::connections
