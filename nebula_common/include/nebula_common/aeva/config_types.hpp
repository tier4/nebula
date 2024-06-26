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

#include "nebula_common/nebula_common.hpp"

#include <cstdint>
#include <ostream>
#include <string>

namespace nebula::drivers::aeva
{

struct Aeries2Config : public SensorConfigurationBase
{
  std::string sensor_ip;
  float dithering_enable_ego_speed;
  std::string dithering_pattern_option;
  float ele_offset_rad;
  bool elevation_auto_adjustment;
  bool enable_frame_dithering;
  bool enable_frame_sync;
  bool flip_pattern_vertically;
  uint16_t frame_sync_offset_in_ms;
  std::string frame_sync_type;
  bool frame_synchronization_on_rising_edge;
  float hfov_adjustment_deg;
  float hfov_rotation_deg;
  bool highlight_ROI;
  std::string horizontal_fov_degrees;
  float roi_az_offset_rad;
  std::string vertical_pattern;
};

inline std::ostream & operator<<(std::ostream & os, const Aeries2Config & arg)
{
  os << "Aeries2Config:\n";
  os << "  sensor_model = " << arg.sensor_model << '\n';
  os << "  frame_id = " << arg.frame_id << '\n';
  os << "  sensor_ip = " << arg.sensor_ip << '\n';
  os << "  dithering_enable_ego_speed = " << arg.dithering_enable_ego_speed << '\n';
  os << "  dithering_pattern_option = " << arg.dithering_pattern_option << '\n';
  os << "  ele_offset_rad = " << arg.ele_offset_rad << '\n';
  os << "  elevation_auto_adjustment = " << arg.elevation_auto_adjustment << '\n';
  os << "  enable_frame_dithering = " << arg.enable_frame_dithering << '\n';
  os << "  enable_frame_sync = " << arg.enable_frame_sync << '\n';
  os << "  flip_pattern_vertically = " << arg.flip_pattern_vertically << '\n';
  os << "  frame_sync_offset_in_ms = " << static_cast<int32_t>(arg.frame_sync_offset_in_ms) << '\n';
  os << "  frame_sync_type = " << arg.frame_sync_type << '\n';
  os << "  frame_synchronization_on_rising_edge = " << arg.frame_synchronization_on_rising_edge
     << '\n';
  os << "  hfov_adjustment_deg = " << arg.hfov_adjustment_deg << '\n';
  os << "  hfov_rotation_deg = " << arg.hfov_rotation_deg << '\n';
  os << "  highlight_ROI = " << arg.highlight_ROI << '\n';
  os << "  horizontal_fov_degrees = " << arg.horizontal_fov_degrees << '\n';
  os << "  roi_az_offset_rad = " << arg.roi_az_offset_rad << '\n';
  os << "  vertical_pattern = " << arg.vertical_pattern;
  return os;
}

}  // namespace nebula::drivers::aeva
