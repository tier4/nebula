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

#include <cstddef>
#include <limits>

namespace nebula::util
{

#define BITFIELD_ACCESSOR(type, name, low_bit, high_bit, storage)        \
  [[nodiscard]] type name() const                                        \
  {                                                                      \
    return nebula::util::get_bitfield<type, low_bit, high_bit>(storage); \
  }

/**
 * @brief Access the value of a bitfield, as a user-defined type.
 *
 * Given a variable `storage` of any numeric type `InT`, extract the region specified by
 * `[LowBit, HighBit]` (both inclusive) and cast it to `OutT`.
 * If `InT` is multiple bytes big, the container is treated with native endianness.
 *
 * @tparam OutT The output type. Usually a numeric type or enum
 * @tparam LowBit The start bit of the bitfield (inclusive)
 * @tparam HighBit The end bit of the bitfield (inclusive)
 * @tparam InT The storage type
 * @param storage The variable acting as the storage for the bitfield
 * @return constexpr OutT The extracted value
 */
template <typename OutT, size_t LowBit, size_t HighBit, typename InT>
constexpr OutT get_bitfield(const InT & storage)
{
  constexpr size_t n_bits = HighBit - LowBit + 1;
  constexpr InT all_ones = ~static_cast<InT>(0);
  constexpr InT mask = (n_bits < std::numeric_limits<InT>::digits)
                         ? static_cast<InT>(~static_cast<InT>(all_ones << n_bits))
                         : all_ones;

  InT raw_value = (storage >> LowBit) & mask;
  return static_cast<OutT>(raw_value);
}

}  // namespace nebula::util
