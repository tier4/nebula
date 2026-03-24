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

#ifndef NEBULA_SEYOND_CONFIGURATION_HPP
#define NEBULA_SEYOND_CONFIGURATION_HPP

#include <nebula_core_decoders/angles.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nlohmann/json.hpp>

#include <string>

namespace nebula::drivers
{

/// @brief Network endpoint settings used by the Seyond driver.
struct SeyondConnectionConfiguration
{
  std::string host_ip;
  std::string sensor_ip;
  uint16_t udp_port;
};

/// @brief Serialize SeyondConnectionConfiguration to JSON.
inline void to_json(nlohmann::json & j, const SeyondConnectionConfiguration & config)
{
  j["host_ip"] = config.host_ip;
  j["sensor_ip"] = config.sensor_ip;
  j["udp_port"] = config.udp_port;
}

/// @brief Deserialize SeyondConnectionConfiguration from JSON.
inline void from_json(const nlohmann::json & j, SeyondConnectionConfiguration & config)
{
  j.at("host_ip").get_to(config.host_ip);
  j.at("sensor_ip").get_to(config.sensor_ip);
  j.at("udp_port").get_to(config.udp_port);
}

/// @brief Sensor-specific configuration for Seyond LiDARs
struct SeyondSensorConfiguration
{
  SeyondSensorModel sensor_model;
  SeyondConnectionConfiguration connection;
  FieldOfView<float, Degrees> fov{};
  bool use_sensor_time{false};
  std::string frame_id{"seyond"};
  bool setup_sensor{true};
  ReturnMode return_mode{ReturnMode::STRONGEST};
  SeyondReflectanceMode reflectance_mode{SeyondReflectanceMode::REFLECTIVITY};
  SeyondSyncMode sync_mode{SeyondSyncMode::HOST};
};

/// @brief Serialize SeyondSensorConfiguration to JSON.
inline void to_json(nlohmann::json & j, const SeyondSensorConfiguration & config)
{
  j["sensor_model"] = static_cast<uint8_t>(config.sensor_model);
  j["connection"] = config.connection;
  j["use_sensor_time"] = config.use_sensor_time;
  j["frame_id"] = config.frame_id;
  j["setup_sensor"] = config.setup_sensor;
  j["return_mode"] = static_cast<uint8_t>(config.return_mode);
  j["reflectance_mode"] = static_cast<uint8_t>(config.reflectance_mode);
  j["sync_mode"] = static_cast<uint8_t>(config.sync_mode);

  nlohmann::json azimuth_fov;
  azimuth_fov["min_deg"] = config.fov.azimuth.start;
  azimuth_fov["max_deg"] = config.fov.azimuth.end;

  nlohmann::json elevation_fov;
  elevation_fov["min_deg"] = config.fov.elevation.start;
  elevation_fov["max_deg"] = config.fov.elevation.end;

  j["fov"] = {{"azimuth", azimuth_fov}, {"elevation", elevation_fov}};
}

/// @brief Parse SeyondSensorConfiguration from JSON.
inline void from_json(const nlohmann::json & j, SeyondSensorConfiguration & config)
{
  config.sensor_model = static_cast<SeyondSensorModel>(j.at("sensor_model").get<uint8_t>());
  j.at("connection").get_to(config.connection);
  j.at("use_sensor_time").get_to(config.use_sensor_time);
  j.at("frame_id").get_to(config.frame_id);
  j.at("setup_sensor").get_to(config.setup_sensor);
  if (j.contains("return_mode"))
    config.return_mode = static_cast<ReturnMode>(j.at("return_mode").get<uint8_t>());
  if (j.contains("reflectance_mode"))
    config.reflectance_mode =
      static_cast<SeyondReflectanceMode>(j.at("reflectance_mode").get<uint8_t>());
  if (j.contains("sync_mode"))
    config.sync_mode = static_cast<SeyondSyncMode>(j.at("sync_mode").get<uint8_t>());

  const auto & fov_json = j.at("fov");
  const auto & azimuth_fov = fov_json.at("azimuth");
  azimuth_fov.at("min_deg").get_to(config.fov.azimuth.start);
  azimuth_fov.at("max_deg").get_to(config.fov.azimuth.end);

  const auto & elevation_fov = fov_json.at("elevation");
  elevation_fov.at("min_deg").get_to(config.fov.elevation.start);
  elevation_fov.at("max_deg").get_to(config.fov.elevation.end);
}

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_CONFIGURATION_HPP
