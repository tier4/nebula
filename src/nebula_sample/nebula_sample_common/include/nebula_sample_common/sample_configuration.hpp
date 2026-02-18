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

#include <nebula_core_common/util/angles.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nlohmann/json.hpp>

#include <string>

namespace nebula::drivers
{

/// @brief Network endpoint settings used by the sample driver.
/// @details These values define where the UDP packets are received from and on which local port.
struct ConnectionConfiguration
{
  /// IP address of the host interface that receives LiDAR packets.
  std::string host_ip;
  /// IP address assigned to the sensor.
  std::string sensor_ip;
  /// UDP destination port on the host for sensor data.
  uint16_t data_port;
};

// Allow automatic JSON serialization/deserialization
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ConnectionConfiguration, host_ip, sensor_ip, data_port);

/// @brief Sensor-specific configuration for the Sample LiDAR
/// @details Minimal tutorial configuration that combines connection settings and angular filtering.
struct SampleSensorConfiguration
{
  ConnectionConfiguration connection;
  FieldOfView<float, Degrees> fov{};
};

/// @brief Serialize SampleSensorConfiguration to JSON.
/// @details The JSON schema contains:
/// - `connection` (ConnectionConfiguration)
/// - `fov` (FieldOfView)
inline void to_json(nlohmann::json & j, const SampleSensorConfiguration & config)
{
  j = nlohmann::json{};
  j["connection"] = config.connection;

  nlohmann::json azimuth_fov;
  azimuth_fov["min_deg"] = config.fov.azimuth.start;
  azimuth_fov["max_deg"] = config.fov.azimuth.end;

  nlohmann::json elevation_fov;
  elevation_fov["min_deg"] = config.fov.elevation.start;
  elevation_fov["max_deg"] = config.fov.elevation.end;

  j["fov"] = {{"azimuth", azimuth_fov}, {"elevation", elevation_fov}};
}

/// @brief Parse SampleSensorConfiguration from JSON.
/// @throws nlohmann::json::exception if required fields are missing or have incompatible types.
inline void from_json(const nlohmann::json & j, SampleSensorConfiguration & config)
{
  j.at("connection").get_to(config.connection);

  const auto & fov_json = j.at("fov");
  const auto & azimuth_fov = fov_json.at("azimuth");
  azimuth_fov.at("min_deg").get_to(config.fov.azimuth.start);
  azimuth_fov.at("max_deg").get_to(config.fov.azimuth.end);

  const auto & elevation_fov = fov_json.at("elevation");
  elevation_fov.at("min_deg").get_to(config.fov.elevation.start);
  elevation_fov.at("max_deg").get_to(config.fov.elevation.end);
}

}  // namespace nebula::drivers
