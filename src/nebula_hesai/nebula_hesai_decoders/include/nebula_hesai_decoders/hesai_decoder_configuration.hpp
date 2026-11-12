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

#pragma once

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace nebula::drivers
{

/// @brief The settings the Hesai decoder needs to turn packets into point clouds.
/// @details This is the decoder-relevant subset of the sensor's parameters. It contains no
/// connection, synchronization or ROS settings.
struct HesaiDecoderConfiguration
{
  SensorModel sensor_model{SensorModel::UNKNOWN};
  /// Points closer than this are dropped [m]
  double min_range{};
  /// Points farther than this are dropped [m]
  double max_range{};
  /// Returns of the same firing closer together than this are treated as one return [m]
  double dual_return_distance_threshold{};
  /// The motor speed the sensor is configured to [RPM]
  uint16_t rotation_speed{};
  /// Start of the azimuth field of view [deg]
  uint16_t cloud_min_angle{};
  /// End of the azimuth field of view [deg]
  uint16_t cloud_max_angle{};
  /// The azimuth at which one scan ends and the next one starts [deg]
  double cut_angle{};
  /// Path to a downsample mask image. No downsampling is done if unset.
  std::optional<std::string> downsample_mask_path;
};

/// @brief Serialize a HesaiDecoderConfiguration to JSON.
/// @details The layout is the one of the Hesai parameter files, without the
/// `/**: ros__parameters:` root:
/// - `sensor_model` (string)
/// - `min_range`, `max_range`, `dual_return_distance_threshold` (number)
/// - `rotation_speed`, `cloud_min_angle`, `cloud_max_angle` (integer)
/// - `cut_angle` (number)
/// - `point_filters.downsample_mask.path` (string, optional, empty means unset)
inline void to_json(nlohmann::json & j, const HesaiDecoderConfiguration & config)
{
  j = nlohmann::json{
    {"sensor_model", sensor_model_to_string(config.sensor_model)},
    {"min_range", config.min_range},
    {"max_range", config.max_range},
    {"dual_return_distance_threshold", config.dual_return_distance_threshold},
    {"rotation_speed", config.rotation_speed},
    {"cloud_min_angle", config.cloud_min_angle},
    {"cloud_max_angle", config.cloud_max_angle},
    {"cut_angle", config.cut_angle},
  };

  if (config.downsample_mask_path) {
    j["point_filters"]["downsample_mask"]["path"] = *config.downsample_mask_path;
  }
}

/// @brief Parse a HesaiDecoderConfiguration from JSON.
/// @details See `to_json` for the layout. Keys that are not decoder settings are ignored, so the
/// full parameter tree of a sensor can be passed.
/// @throws nlohmann::json::exception if required fields are missing or have incompatible types.
inline void from_json(const nlohmann::json & j, HesaiDecoderConfiguration & config)
{
  config.sensor_model = sensor_model_from_string(j.at("sensor_model").get<std::string>());
  j.at("min_range").get_to(config.min_range);
  j.at("max_range").get_to(config.max_range);
  j.at("dual_return_distance_threshold").get_to(config.dual_return_distance_threshold);
  j.at("rotation_speed").get_to(config.rotation_speed);
  j.at("cloud_min_angle").get_to(config.cloud_min_angle);
  j.at("cloud_max_angle").get_to(config.cloud_max_angle);
  j.at("cut_angle").get_to(config.cut_angle);

  config.downsample_mask_path = std::nullopt;
  const auto mask_path_ptr = nlohmann::json::json_pointer("/point_filters/downsample_mask/path");
  if (j.contains(mask_path_ptr)) {
    auto mask_path = j.at(mask_path_ptr).get<std::string>();
    if (!mask_path.empty()) {
      config.downsample_mask_path = std::move(mask_path);
    }
  }
}

/// @brief Check that a HesaiDecoderConfiguration can be used to construct a decoder.
/// @return Nothing if the configuration is valid, otherwise a description of the first problem.
inline util::expected<std::monostate, std::string> validate(
  const HesaiDecoderConfiguration & config)
{
  if (config.sensor_model == SensorModel::UNKNOWN) {
    return std::string("Unknown sensor model.");
  }

  if (config.cloud_min_angle > 360 || config.cloud_max_angle > 360) {
    return std::string("The FoV has to be within 0 and 360 deg.");
  }

  if (!angle_is_between<double>(config.cloud_min_angle, config.cloud_max_angle, config.cut_angle)) {
    return std::string("Cannot cut scan outside of the FoV.");
  }

  const bool fov_is_360 = config.cloud_min_angle == 0 && config.cloud_max_angle == 360;
  if (!fov_is_360 && config.cut_angle == config.cloud_min_angle) {
    return std::string("Cannot cut scan right at the start of the FoV. Cut at the end instead.");
  }

  // Handling cutting at 360deg (as opposed to 0deg) for a full 360deg FoV requires a special case
  // in the cutting logic. Thus, require the user to cut at 0deg.
  if (fov_is_360 && config.cut_angle == 360) {
    return std::string("Cannot cut a 360deg FoV at 360deg. Cut at 0deg instead.");
  }

  if (config.downsample_mask_path && !std::filesystem::exists(*config.downsample_mask_path)) {
    return "Downsample mask not found: " + *config.downsample_mask_path;
  }

  return std::monostate{};
}

/// @brief Extract the decoder settings from a full sensor configuration.
inline HesaiDecoderConfiguration make_decoder_configuration(const HesaiSensorConfiguration & config)
{
  HesaiDecoderConfiguration decoder_config;
  decoder_config.sensor_model = config.sensor_model;
  decoder_config.min_range = config.min_range;
  decoder_config.max_range = config.max_range;
  decoder_config.dual_return_distance_threshold = config.dual_return_distance_threshold;
  decoder_config.rotation_speed = config.rotation_speed;
  decoder_config.cloud_min_angle = config.cloud_min_angle;
  decoder_config.cloud_max_angle = config.cloud_max_angle;
  decoder_config.cut_angle = config.cut_angle;
  decoder_config.downsample_mask_path = config.downsample_mask_path;
  return decoder_config;
}

}  // namespace nebula::drivers
