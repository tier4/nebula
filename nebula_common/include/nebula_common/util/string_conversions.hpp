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

#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>

namespace nebula::util
{

template <typename T, typename = void>
struct IsStreamable : std::false_type
{
};

template <typename T>
struct IsStreamable<T, std::void_t<decltype(std::declval<std::ostream &>() << std::declval<T>())> >
: std::true_type
{
};

template <typename T>
std::enable_if_t<IsStreamable<T>::value, std::string> to_string(const T & value)
{
  std::stringstream ss{};
  ss << value;
  return ss.str();
}

}  // namespace nebula::util
