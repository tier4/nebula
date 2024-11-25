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

#include "nebula_common/util/expected.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/tcp.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

using nlohmann::json;
using std::string_literals::operator""s;

using message_t = std::vector<uint8_t>;
using conversation_db_t = std::map<message_t, std::vector<message_t>>;

class ParseError : public std::runtime_error
{
public:
  explicit ParseError(const std::string & msg) : std::runtime_error(msg) {}

  ParseError() : std::runtime_error("unknown format") {}
};

namespace impl
{
const size_t g_ptc_header_size = 8;
const std::string_view g_hex_prefix = "0x";

inline util::expected<message_t, ParseError> parse_message(const std::string & json_str)
{
  if (std::string_view json_view = json_str;
      !std::equal(json_view.cbegin(), g_hex_prefix.cbegin(), g_hex_prefix.cend())) {
    return ParseError();
  }

  message_t result;
  for (size_t i = g_hex_prefix.length(); i < json_str.length(); i += 2) {
    std::string hex_pair = json_str.substr(i, 2);
    uint8_t byte = strtoul(hex_pair.c_str(), nullptr, 16);
    if (errno) {
      return ParseError(strerror(errno));
    }
    result.emplace_back(byte);
  }

  return result;
}
};  // namespace impl

inline util::expected<conversation_db_t, ParseError> parse_conversation_db(
  const std::string & json_filename)
{
  std::ifstream ifs(json_filename);
  if (!ifs.is_open()) {
    return ParseError{"failed to open file: " + json_filename};
  }

  json raw_db = json::parse(ifs);
  if (!raw_db.contains("rules") || !raw_db["rules"].is_array()) {
    return ParseError{};
  }

  conversation_db_t result;
  for (const auto & obj : raw_db["rules"]) {
    if (!obj.contains("request") || !obj["request"].is_string()) {
      return ParseError{};
    }

    if (!obj.contains("responses") || !obj["responses"].is_array()) {
      return ParseError{};
    }

    auto request_exp = impl::parse_message(obj["request"]);
    if (!request_exp.has_value()) {
      return request_exp.error();
    }

    std::vector<message_t> responses;
    for (const auto & response : obj["responses"]) {
      if (!response.is_string()) {
        return ParseError{};
      }

      auto parsed_message_exp = impl::parse_message(obj.template get<std::string>());
      if (!parsed_message_exp.has_value()) {
        return parsed_message_exp.error();
      }
      responses.emplace_back(parsed_message_exp.value());
    }

    result.insert({request_exp.value(), responses});
  }

  return result;
}

class ReplayTcpSocket : public AbstractTcpSocket
{
public:
  using ptc_handler_t = std::function<void(
    const message_t & request, const header_callback_t & cb_header,
    const payload_callback_t & cb_payload, const completion_callback_t & cb_completion)>;

  explicit ReplayTcpSocket(ptc_handler_t ptc_handler) : ptc_handler_(std::move(ptc_handler)) {}

  void init(
    const std::string & /* host_ip */, uint16_t /* host_port */,
    const std::string & /* remote_ip */, uint16_t /* remote_port */) override
  {
  }

  void bind() override {}

  void close() override {}

  void async_ptc_request(
    message_t & ptc_packet, header_callback_t cb_header, payload_callback_t cb_payload,
    completion_callback_t cb_completion) override
  {
    ptc_handler_(ptc_packet, cb_header, cb_payload, cb_completion);
  }

private:
  ptc_handler_t ptc_handler_;
};

}  // namespace nebula::drivers::connections
