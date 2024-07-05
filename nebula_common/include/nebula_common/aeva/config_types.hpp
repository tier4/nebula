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
  os << "Aeva Aeries2 Sensor Configuration:\n";
  os << "Sensor Model: " << arg.sensor_model << '\n';
  os << "Frame ID: " << arg.frame_id << '\n';
  os << "Sensor IP: " << arg.sensor_ip << '\n';
  os << "Dithering Enable Ego Speed: " << arg.dithering_enable_ego_speed << '\n';
  os << "Dithering Pattern Option: " << arg.dithering_pattern_option << '\n';
  os << "Elevation Offset (rad): " << arg.ele_offset_rad << '\n';
  os << "Elevation Auto Adjustment: " << arg.elevation_auto_adjustment << '\n';
  os << "Enable Frame Dithering: " << arg.enable_frame_dithering << '\n';
  os << "Enable Frame Sync: " << arg.enable_frame_sync << '\n';
  os << "Flip Pattern Vertically: " << arg.flip_pattern_vertically << '\n';
  os << "Frame Sync Offset (ms): " << static_cast<int32_t>(arg.frame_sync_offset_in_ms) << '\n';
  os << "Frame Sync Type: " << arg.frame_sync_type << '\n';
  os << "Frame Sync on Rising Edge: " << arg.frame_synchronization_on_rising_edge << '\n';
  os << "hFoV Adjustment (deg): " << arg.hfov_adjustment_deg << '\n';
  os << "hFoV Rotation (deg): " << arg.hfov_rotation_deg << '\n';
  os << "Highlight ROI: " << arg.highlight_ROI << '\n';
  os << "Horizontal FoV (deg): " << arg.horizontal_fov_degrees << '\n';
  os << "ROI Azimuth Offset (rad): " << arg.roi_az_offset_rad << '\n';
  os << "Vertical Pattern: " << arg.vertical_pattern;
  return os;
}

}  // namespace nebula::drivers::aeva
