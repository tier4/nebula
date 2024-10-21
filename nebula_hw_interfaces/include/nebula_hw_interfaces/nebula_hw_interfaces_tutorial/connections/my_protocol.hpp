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

#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/util/expected.hpp>

#include <boost/algorithm/string/join.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class MyProtocolConnection
{
public:
  enum class MyProtocolCmdError { NO_ERROR = 0, NOT_IMPLEMENTED };

  using my_protocol_cmd_result_t =
    typename nebula::util::expected<std::vector<uint8_t>, MyProtocolCmdError>;

  MyProtocolConnection(
    std::shared_ptr<nebula::drivers::loggers::Logger> logger, const std::string & sensor_ip,
    uint16_t sensor_port, const std::string & host_ip, uint16_t host_port)
  : logger_(std::move(logger)), ctx_(new boost::asio::io_context(1)), tcp_driver_(ctx_)
  {
    tcp_driver_.init_socket(sensor_ip, sensor_port, host_ip, host_port);
    if (!tcp_driver_.open()) {
      tcp_driver_.closeSync();
      throw std::runtime_error("Could not open TCP connection!");
    }
  }

  /// @brief Send a GET command to the sensor synchronously and return the parsed result
  /// @tparam T The type of the expected result
  /// @param command_id The PTC command ID
  /// @param payload The payload (optional)
  /// @return The parsed result if there is no error, else throw
  template <typename T>
  T get(const uint8_t command_id, const std::vector<uint8_t> & payload = {})
  {
    auto response_or_err = send_receive(command_id, payload);
    auto response =
      response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
    return check_size_and_parse<T>(response);
  }

  /// @brief Send a SET command to the sensor synchronously
  /// @param command_id The PTC command ID
  /// @param payload The payload to set
  void set(const uint8_t command_id, const std::vector<uint8_t> & payload)
  {
    auto response_or_err = send_receive(command_id, payload);
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  }

private:
  my_protocol_cmd_result_t send_receive(
    const uint8_t /* command_id */, const std::vector<uint8_t> & /* payload */)
  {
    return MyProtocolCmdError::NOT_IMPLEMENTED;
  }

  /// @brief Convert an error code to a human-readable string
  /// @param error_code The error code, containing the sensor's error code (if any), along with
  /// flags such as TCP_ERROR_UNRELATED_RESPONSE etc.
  /// @return A string description of all errors in this code
  std::string pretty_print_ptc_error(MyProtocolCmdError error_code)
  {
    switch (error_code) {
      case MyProtocolCmdError::NO_ERROR:
        return "No error";
      case MyProtocolCmdError::NOT_IMPLEMENTED:
        return "Not implemented";
    }
  }

  /// @brief Checks if the data size matches that of the struct to be parsed, and parses the struct.
  /// If data is too small, a std::runtime_error is thrown. If data is too large, a warning is
  /// printed and the struct is parsed with the first sizeof(T) bytes.
  template <typename T>
  T check_size_and_parse(const std::vector<uint8_t> & data)
  {
    if (data.size() != sizeof(T)) {
      throw std::runtime_error("Attempted to parse payload of unsupported size");
    }

    T parsed;
    memcpy(&parsed, data.data(), sizeof(T));
    return parsed;
  }

  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;

  std::shared_ptr<boost::asio::io_context> ctx_;
  ::drivers::tcp_driver::TcpDriver tcp_driver_;
};

}  // namespace nebula::drivers::connections
