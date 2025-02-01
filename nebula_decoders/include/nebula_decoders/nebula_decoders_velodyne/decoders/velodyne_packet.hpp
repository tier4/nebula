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

#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula::drivers::velodyne_packet
{

/**
 * Raw Velodyne packet constants and structures.
 */
static const int g_raw_scan_size = 3;     // TODO(ike-kazu): remove
static const int g_scans_per_block = 32;  // TODO(ike-kazu): remove
static const int g_channels_per_block = 32;

static const double g_rotation_resolution = 0.01;         // [deg]
static const uint16_t g_rotation_max_units = 360 * 100u;  // [deg/100]

static const size_t g_return_mode_index = 1204;

/** @todo make this work for both big and little-endian machines */
static const uint16_t g_upper_bank = 0xeeff;
static const uint16_t g_lower_bank = 0xddff;

/** Return Modes **/
static const uint16_t g_return_mode_strongest = 55;
static const uint16_t g_return_mode_last = 56;
static const uint16_t g_return_mode_dual = 57;

const int g_blocks_per_packet = 12;
const int g_packet_status_size = 4;
const int g_scans_per_packet = (g_scans_per_block * g_blocks_per_packet);
const int g_points_per_packet = (g_scans_per_packet * g_raw_scan_size);

#pragma pack(push, 1)
/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
struct raw_units
{
  uint16_t distance;
  uint8_t reflectivity;
};

struct raw_block_t
{
  uint16_t flag;      ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  raw_units units[g_scans_per_block];
};

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
struct raw_packet_t
{
  raw_block_t blocks[g_blocks_per_packet];
  uint32_t timestamp;
  uint16_t factory;
};
#pragma pack(pop)

/** \brief Velodyne echo types */
enum RETURN_TYPE {
  INVALID = 0,
  SINGLE_STRONGEST = 1,
  SINGLE_LAST = 2,
  DUAL_STRONGEST_FIRST = 3,
  DUAL_STRONGEST_LAST = 4,
  DUAL_WEAK_FIRST = 5,
  DUAL_WEAK_LAST = 6,
  DUAL_ONLY = 7
};

}  // namespace nebula::drivers::velodyne_packet
