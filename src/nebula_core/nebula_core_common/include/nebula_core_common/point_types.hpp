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

#include <nebula_core_common/point_cloud.hpp>

#include <array>
#include <cstdint>
#include <memory>

namespace nebula::drivers
{

struct alignas(16) PointXYZ
{
  float x;
  float y;
  float z;

  static std::array<PointField, 3> fields()
  {
    return {
      {PointField{"x", offsetof(PointXYZ, x), PointField::DataType::Float32, 1},
       PointField{"y", offsetof(PointXYZ, y), PointField::DataType::Float32, 1},
       PointField{"z", offsetof(PointXYZ, z), PointField::DataType::Float32, 1}},
    };
  };
};

struct alignas(16) PointXYZIR
{
  float x;
  float y;
  float z;
  union {
    float padding_;
  };

  float intensity;
  uint16_t ring;

  static std::array<PointField, 5> fields()
  {
    return {
      {PointField{"x", offsetof(PointXYZIR, x), PointField::DataType::Float32, 1},
       PointField{"y", offsetof(PointXYZIR, y), PointField::DataType::Float32, 1},
       PointField{"z", offsetof(PointXYZIR, z), PointField::DataType::Float32, 1},
       PointField{"intensity", offsetof(PointXYZIR, intensity), PointField::DataType::Float32, 1},
       PointField{"ring", offsetof(PointXYZIR, ring), PointField::DataType::UInt16, 1}},
    };
  };
};

/**
 * This point type is not using PCL_ADD_POINT4D to avoid the addition of a 32-bit dummy word.
 * The fields are ordered to meet the SSE alignment.
 */
struct alignas(16) PointXYZIRCAEDT
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  std::uint32_t time_stamp;

  static std::array<PointField, 10> fields()
  {
    return {
      {PointField{"x", offsetof(PointXYZIRCAEDT, x), PointField::DataType::Float32, 1},
       PointField{"y", offsetof(PointXYZIRCAEDT, y), PointField::DataType::Float32, 1},
       PointField{"z", offsetof(PointXYZIRCAEDT, z), PointField::DataType::Float32, 1},
       PointField{
         "intensity", offsetof(PointXYZIRCAEDT, intensity), PointField::DataType::UInt8, 1},
       PointField{
         "return_type", offsetof(PointXYZIRCAEDT, return_type), PointField::DataType::UInt8, 1},
       PointField{"channel", offsetof(PointXYZIRCAEDT, channel), PointField::DataType::UInt16, 1},
       PointField{"azimuth", offsetof(PointXYZIRCAEDT, azimuth), PointField::DataType::Float32, 1},
       PointField{
         "elevation", offsetof(PointXYZIRCAEDT, elevation), PointField::DataType::Float32, 1},
       PointField{
         "distance", offsetof(PointXYZIRCAEDT, distance), PointField::DataType::Float32, 1},
       PointField{
         "time_stamp", offsetof(PointXYZIRCAEDT, time_stamp), PointField::DataType::UInt32, 1}},
    };
  };
};

struct alignas(16) PointXYZIRADT
{
  float x;
  float y;
  float z;
  union {
    float padding_;
  };

  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;

  static std::array<PointField, 9> fields()
  {
    return {
      {PointField{"x", offsetof(PointXYZIRADT, x), PointField::DataType::Float32, 1},
       PointField{"y", offsetof(PointXYZIRADT, y), PointField::DataType::Float32, 1},
       PointField{"z", offsetof(PointXYZIRADT, z), PointField::DataType::Float32, 1},
       PointField{
         "intensity", offsetof(PointXYZIRADT, intensity), PointField::DataType::Float32, 1},
       PointField{"ring", offsetof(PointXYZIRADT, ring), PointField::DataType::UInt16, 1},
       PointField{"azimuth", offsetof(PointXYZIRADT, azimuth), PointField::DataType::Float32, 1},
       PointField{"distance", offsetof(PointXYZIRADT, distance), PointField::DataType::Float32, 1},
       PointField{
         "return_type", offsetof(PointXYZIRADT, return_type), PointField::DataType::UInt8, 1},
       PointField{
         "time_stamp", offsetof(PointXYZIRADT, time_stamp), PointField::DataType::Float64, 1}},
    };
  };
};

using NebulaPoint = PointXYZIRCAEDT;
using NebulaPointCloud = PointCloud<NebulaPoint>;
using NebulaPointCloudPtr = std::shared_ptr<NebulaPointCloud>;

}  // namespace nebula::drivers
