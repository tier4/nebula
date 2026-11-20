// Copyright 2025 TIER IV, Inc.
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

#include <array>
#include <cstring>
#include <string>
#include <type_traits>

namespace nebula::util
{

inline std::string errno_to_string(int err_no)
{
  static constexpr size_t gnu_max_strerror_length = 1024;
  std::array<char, gnu_max_strerror_length> msg_buf{};  // NOLINT
  // strerror_r returns char * (GNU, glibc in C++) or int (XSI, e.g. macOS)
  auto to_message = [&msg_buf](auto result) -> std::string {
    if constexpr (std::is_same_v<decltype(result), int>) {
      return msg_buf.data();
    } else {
      return result;
    }
  };
  return to_message(strerror_r(err_no, msg_buf.data(), msg_buf.size()));
}

}  // namespace nebula::util
