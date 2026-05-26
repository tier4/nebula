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

#ifndef NEBULA_RADAR_TYPES_HPP
#define NEBULA_RADAR_TYPES_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace nebula::drivers
{

struct RadarDetection
{
  float x;
  float y;
  float z;
  float range;
  float azimuth;
  float elevation;
  float range_rate;
  float rcs;
  uint16_t id;
  uint8_t classification;
};

struct RadarDetectionList
{
  uint64_t timestamp_ns;
  std::string frame_id;
  std::vector<RadarDetection> detections;
};

using RadarDetectionListPtr = std::shared_ptr<RadarDetectionList>;

struct RadarObject
{
  float x;
  float y;
  float z;
  float velocity_x;
  float velocity_y;
  float acceleration_x;
  float acceleration_y;
  float size_x;
  float size_y;
  float orientation;
  uint32_t id;
  uint8_t classification;
  float existence_probability;
};

struct RadarObjectList
{
  uint64_t timestamp_ns;
  std::string frame_id;
  std::vector<RadarObject> objects;
};

using RadarObjectListPtr = std::shared_ptr<RadarObjectList>;

}  // namespace nebula::drivers

#endif  // NEBULA_RADAR_TYPES_HPP
