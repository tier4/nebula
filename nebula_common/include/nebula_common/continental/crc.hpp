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

#include <cstdint>

template <typename Iterator>
int crc16_packets(Iterator begin, Iterator end, int payload_offset)
{
  uint16_t crc_word = 0xffff;

  for (Iterator it = begin; it != end; ++it) {
    for (auto it2 = it->data.begin() + payload_offset; it2 != it->data.end(); ++it2) {
      auto byte = *it2;
      crc_word ^= byte << 8;
      for (int i = 0; i < 8; i++)
        crc_word = crc_word & 0x8000 ? (crc_word << 1) ^ 0x1021 : crc_word << 1;
    }
  }

  return crc_word;
}

template <typename Iterator>
int crc16_packet(Iterator begin, Iterator end)
{
  uint16_t crc_word = 0xffff;
  for (Iterator it = begin; it != end; ++it) {
    crc_word ^= (*it) << 8;
    for (int i = 0; i < 8; i++)
      crc_word = crc_word & 0x8000 ? (crc_word << 1) ^ 0x1021 : crc_word << 1;
  }

  return crc_word;
}

template <typename Iterator>
uint8_t crc8h2f(Iterator begin, Iterator end)
{
  uint8_t crc_word = 0xFF;
  uint8_t bit = 0;

  for (Iterator it = begin; it != end; ++it) {
    crc_word ^= *it;
    for (bit = 0; bit < 8; bit++) {
      if ((crc_word & 0x80) != 0) {
        crc_word <<= 1;
        crc_word ^= 0x2F;
      } else {
        crc_word <<= 1;
      }
    }
  }

  return crc_word ^ 0xFF;
}
