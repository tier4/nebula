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

#include <nebula_core_common/util/errno.hpp>
#include <nebula_core_common/util/expected.hpp>

#include <arpa/inet.h>

#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>

namespace nebula::drivers::connections
{

class SocketError : public std::exception
{
public:
  explicit SocketError(int err_no) : what_{util::errno_to_string(err_no)} {}

  explicit SocketError(const std::string_view & msg) : what_(msg) {}

  const char * what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

class UsageError : public std::runtime_error
{
public:
  explicit UsageError(const std::string & msg) : std::runtime_error(msg) {}
};

inline util::expected<in_addr, UsageError> parse_ip(const std::string & ip)
{
  in_addr parsed_addr{};
  bool valid = inet_aton(ip.c_str(), &parsed_addr);
  if (!valid) return UsageError("Invalid IP address given");
  return parsed_addr;
}

}  // namespace nebula::drivers::connections
