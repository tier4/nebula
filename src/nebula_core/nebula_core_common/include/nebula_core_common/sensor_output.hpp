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

#ifndef NEBULA_SENSOR_OUTPUT_HPP
#define NEBULA_SENSOR_OUTPUT_HPP

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/radar_types.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace nebula::drivers
{
enum class SensorOutputKind {
  PointCloud,
  RadarDetections,
  RadarObjects,
  Status,
  Diagnostics,
  Telemetry,
};

struct SensorStatus
{
  std::string status;
  std::map<std::string, std::string> fields;
};

struct SensorDiagnostics
{
  std::vector<std::string> messages;
  std::map<std::string, double> metrics;
};

struct SensorTelemetry
{
  std::map<std::string, double> numeric_values;
  std::map<std::string, std::string> text_values;
};

using SensorOutputPayload = std::variant<
  std::monostate, NebulaPointCloudPtr, RadarDetectionListPtr, RadarObjectListPtr, SensorStatus,
  SensorDiagnostics, SensorTelemetry>;

struct SensorDecodedOutput
{
  SensorOutputKind kind{SensorOutputKind::Status};
  uint64_t timestamp_ns{0};
  std::string sensor_id;
  SensorOutputPayload payload;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_OUTPUT_HPP
