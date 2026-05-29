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

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <nlohmann/json.hpp>

#include <string>

namespace nebula::drivers
{

/// @brief Network endpoint settings used by the ouster driver.
/// @details These values define where the UDP packets are received from and on which local port.
struct ConnectionConfiguration
{
  /// IP address of the host interface that receives LiDAR packets.
  std::string host_ip;
  /// IP address assigned to the sensor.
  std::string sensor_ip;
  /// UDP destination port on the host for sensor data.
  uint16_t data_port;
  /// UDP destination port on the host for IMU data. Set to 0 to disable the IMU socket (IMU
  /// packets may still arrive on @c data_port if the sensor is configured that way).
  uint16_t imu_port{0};
  /// Maximum UDP payload we allocate for recv (Ouster frames are often 12k–64kB; default was 1500).
  uint32_t receiver_mtu_bytes{65527};
  /// If true, require LiDAR packets from @c sensor_ip (Nebula UDP filter checks IP only).
  bool filter_sender_ip{true};
};

// JSON: extra keys optional so older config files keep working.
inline void to_json(nlohmann::json & j, const ConnectionConfiguration & c)
{
  j = nlohmann::json{
    {"host_ip", c.host_ip},
    {"sensor_ip", c.sensor_ip},
    {"data_port", c.data_port},
    {"receiver_mtu_bytes", c.receiver_mtu_bytes},
    {"filter_sender_ip", c.filter_sender_ip},
  };
}

inline void from_json(const nlohmann::json & j, ConnectionConfiguration & c)
{
  j.at("host_ip").get_to(c.host_ip);
  j.at("sensor_ip").get_to(c.sensor_ip);
  j.at("data_port").get_to(c.data_port);
  if (j.contains("imu_port")) {
    j.at("imu_port").get_to(c.imu_port);
  }
  if (j.contains("receiver_mtu_bytes")) {
    j.at("receiver_mtu_bytes").get_to(c.receiver_mtu_bytes);
  }
  if (j.contains("filter_sender_ip")) {
    j.at("filter_sender_ip").get_to(c.filter_sender_ip);
  }
}

/// @brief Sensor-specific configuration for the Ouster LiDAR
/// @details Minimal tutorial configuration that combines connection settings and angular filtering.
struct OusterSensorConfiguration
{
  ConnectionConfiguration connection;
  FieldOfView<float, Degrees> fov{};
};

/// @brief Serialize OusterSensorConfiguration to JSON.
/// @details The JSON schema contains:
/// - `connection` (ConnectionConfiguration)
/// - `fov` (FieldOfView)
inline void to_json(nlohmann::json & j, const OusterSensorConfiguration & config)
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

/// @brief Parse OusterSensorConfiguration from JSON.
/// @throws nlohmann::json::exception if required fields are missing or have incompatible types.
inline void from_json(const nlohmann::json & j, OusterSensorConfiguration & config)
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
