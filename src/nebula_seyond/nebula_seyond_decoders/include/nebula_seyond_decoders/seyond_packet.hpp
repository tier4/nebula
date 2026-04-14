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

#ifndef NEBULA_SEYOND_PACKET_HPP
#define NEBULA_SEYOND_PACKET_HPP

#include <cstdint>

namespace nebula::drivers
{

#pragma pack(push, 1)

struct SeyondPacketCommon
{
  uint16_t magic_number;
  uint8_t major_version;
  uint8_t minor_version;
  uint16_t fw_sequence;
  uint32_t checksum;
  uint32_t size;
  uint8_t source_id : 4;
  uint8_t timestamp_sync_type : 4;
  uint8_t lidar_type;
  double ts_start_us;
  uint8_t lidar_mode;
  uint8_t lidar_status;
};

struct SeyondDataPacket
{
  SeyondPacketCommon common;
  uint64_t idx;
  uint16_t sub_idx;
  uint16_t sub_seq;
  uint32_t type : 8;
  uint32_t item_number : 24;
  uint16_t item_size;
  uint32_t topic;
  uint16_t scanner_direction : 1;
  uint16_t use_reflectance : 1;
  uint16_t multi_return_mode : 3;
  uint16_t confidence_level : 2;
  uint16_t is_last_sub_frame : 1;
  uint16_t is_last_sequence : 1;
  uint16_t has_tail : 1;
  uint16_t frame_sync_locked : 1;
  uint16_t is_first_sub_frame : 1;
  uint16_t last_four_channel : 1;
  uint16_t long_distance_mode : 1;
  uint16_t reserved_flag : 2;
  int16_t roi_h_angle;
  int16_t roi_v_angle;
  uint32_t extend_reserved[4];
};

struct SeyondFalconDataPacket
{
  SeyondPacketCommon common;
  uint64_t idx;
  uint16_t sub_idx;
  uint16_t sub_seq;
  uint32_t type : 8;
  uint32_t item_number : 24;
  uint16_t item_size;
  uint32_t topic;
  uint16_t scanner_direction : 1;
  uint16_t use_reflectance : 1;
  uint16_t multi_return_mode : 3;
  uint16_t confidence_level : 2;
  uint16_t is_last_sub_frame : 1;
  uint16_t is_last_sequence : 1;
  uint16_t has_tail : 1;
  uint16_t frame_sync_locked : 1;
  uint16_t is_first_sub_frame : 1;
  uint16_t last_four_channel : 1;
  uint16_t long_distance_mode : 1;
  uint16_t reserved_flag : 2;
  int16_t roi_h_angle;
  int16_t roi_v_angle;
};

/// @brief Standard block header
struct SeyondBlockHeader
{
  int16_t h_angle;
  int16_t v_angle;
  uint16_t ts_10us;
  uint16_t scan_idx;
  uint16_t scan_id : 9;
  int64_t h_angle_diff_1 : 9;
  int64_t h_angle_diff_2 : 10;
  int64_t h_angle_diff_3 : 11;
  int64_t v_angle_diff_1 : 8;
  int64_t v_angle_diff_2 : 9;
  int64_t v_angle_diff_3 : 9;
  uint64_t in_roi : 2;
  uint64_t facet : 3;
  uint64_t reserved_flags : 2;
};

/// @brief FalconK standard channel point matching the vendor SDK layout, 4 bytes packed
struct SeyondChannelPoint
{
  /// distance unit: 1/200m (normal) or 1/100m (long_distance_mode)
  uint32_t radius : 17;
  uint32_t refl : 8;  ///< reflectance or intensity
  uint32_t is_2nd_return : 1;
  uint32_t type : 2;  ///< 0=normal, 1=ground, 2=fog
  uint32_t elongation : 4;
};

struct SeyondBlock
{
  SeyondBlockHeader header;
  SeyondChannelPoint points[4];
};

struct SeyondBlockDual
{
  SeyondBlockHeader header;
  SeyondChannelPoint points[8];
};

/// @brief Enhanced block for Robin W / E1X
struct SeyondEnBlockHeader
{
  int16_t h_angle;
  int16_t v_angle;
  uint16_t ts_10us;
  int64_t h_angle_diff_1 : 11;
  int64_t h_angle_diff_2 : 11;
  int64_t h_angle_diff_3 : 12;
  int64_t v_angle_diff_1 : 10;
  int64_t v_angle_diff_2 : 10;
  int64_t v_angle_diff_3 : 10;
  uint16_t scan_idx;
  uint16_t scan_id : 9;
  uint16_t in_roi : 2;
  uint16_t facet : 3;
  uint16_t reserved_flags : 2;
};

struct SeyondEnChannelPoint
{
  uint16_t reflectance;
  uint16_t intensity;
  uint32_t elongation : 7;
  uint32_t is_2nd_return : 1;
  uint32_t radius : 19;
  uint32_t type : 2;
  uint32_t firing : 1;
  uint32_t reserved_flags : 2;
};

struct SeyondEnBlock
{
  SeyondEnBlockHeader header;
  SeyondEnChannelPoint points[4];
};

/// @brief Compact block for Hummingbird D1
struct SeyondCoBlockHeader
{
  int16_t p_angle;
  int16_t g_angle;
  uint16_t ts_10us;
  uint16_t scan_idx;
  uint16_t scan_id : 9;
  uint16_t in_roi : 2;
  uint16_t facet : 3;
  uint16_t reserved_flags : 2;
};

struct SeyondCoChannelPoint
{
  uint32_t refl : 12;
  uint32_t radius : 18;
  uint32_t is_2nd_return : 1;
  uint32_t firing : 1;
};

struct SeyondCoBlock
{
  SeyondCoBlockHeader header;
  SeyondCoChannelPoint points[8];
};

#pragma pack(pop)

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_PACKET_HPP
