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

#include <boost/crc.hpp>

#include <cstdint>

namespace nebula::drivers
{

/// @brief CRC-16/CCITT-FALSE algorithm
using crc16_ccit_false_t = boost::crc_ccitt_false_t;

/// @brief AUTOSAR CRC8H2F routine.
///
/// See AUTOSAR Specification of CRC Routines >=3.1.4, sec. 7.2.1.2.
///
/// | Bit Width | Polynomial | Initial Value | Final XOR | Reflect In | Reflect Remainder |
/// |----------:|-----------:|--------------:|----------:|-----------:|------------------:|
/// |         8 |       0x2F |          0xFF |      0xFF |      false |             false |
using crc8h2f_t = boost::crc_optimal<8, 0x2F, 0xFF, 0xFF, false, false>;

/// @brief CRC-32/MPEG-2 algorithm
///
/// | Bit Width | Polynomial | Initial Value |  Final XOR | Reflect In | Reflect Remainder |
/// |----------:|-----------:|--------------:|-----------:|-----------:|------------------:|
/// |        32 | 0x04C11DB7 |   0xFFFFFFFF  | 0x00000000 |      false |             false |
using crc32_mpeg2_t = boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0x0, false, false>;

/**
 * @brief Given a memory range, calculates the CRC over its bytes.
 *
 * Both `begin` and `end` can be pointers to arbitrary types. They will be cast to `void *`.
 *
 * A common example looks like:
 *
 * ```c++
 * struct MyStruct {
 *   int a;
 *   float b;
 *   uint32_t crc;
 *
 *   bool is_crc_valid() const {
 *     return crc<crc32_mpeg_t>(&a, &crc) == crc;
 *   }
 * };
 * ```
 *
 * @tparam crc_type The type of CRC algorithm to use. Has to be a `boost::crc_optimal` type.
 * @param begin The beginning of the memory range
 * @param end One (byte) past the end of the memory range
 * @return crc_type::value_type The computed CRC
 */
template <typename crc_type, typename T, typename U>
typename crc_type::value_type crc(const T * begin, const U * end)
{
  const auto * begin_ptr = static_cast<const void *>(begin);
  const auto * end_ptr = static_cast<const void *>(end);

  crc_type crc_calculator;
  crc_calculator.process_block(begin_ptr, end_ptr);
  return crc_calculator.checksum();
}

}  // namespace nebula::drivers
