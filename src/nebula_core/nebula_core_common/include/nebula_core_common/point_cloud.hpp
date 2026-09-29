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

#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace nebula::drivers
{

struct PointField
{
  enum class DataType : uint8_t {
    Int8 = 1,
    UInt8 = 2,
    Int16 = 3,
    UInt16 = 4,
    Int32 = 5,
    UInt32 = 6,
    Float32 = 7,
    Float64 = 8,
  };

  std::string name;
  uint32_t offset;
  DataType datatype;
  uint32_t count;
};

template <typename T, typename = std::void_t<>>
struct IsPointType : std::false_type
{
};

template <typename T>
struct IsPointType<T, std::void_t<decltype(std::declval<T>().fields())>> : std::true_type
{
};

/// Replacement for the deprecated `std::is_pod_v`: true if `T` has standard layout, is trivially
/// copyable and is trivially default-constructible, i.e. it can be (de)serialized via memcpy.
template <typename T>
inline constexpr bool is_pod_like_v =
  std::is_standard_layout_v<T> && std::is_trivially_copyable_v<T> &&
  std::is_trivially_default_constructible_v<T>;

template <typename T>
class PointCloud : public std::vector<T>
{
  static_assert(IsPointType<T>::value, "T must be a valid point type");
  static_assert(is_pod_like_v<T>, "T must be a plain old data (POD) type");
};

}  // namespace nebula::drivers
