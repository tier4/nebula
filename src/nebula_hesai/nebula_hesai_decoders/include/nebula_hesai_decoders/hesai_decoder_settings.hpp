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

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <filesystem>
#include <string>
#include <variant>

namespace nebula::drivers
{

/// @brief Check the settings of a sensor configuration that the decoder uses.
/// @return Nothing if they are valid, otherwise a description of the first problem.
inline util::expected<std::monostate, std::string> validate_decoder_settings(
  const HesaiSensorConfiguration & config)
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

}  // namespace nebula::drivers
