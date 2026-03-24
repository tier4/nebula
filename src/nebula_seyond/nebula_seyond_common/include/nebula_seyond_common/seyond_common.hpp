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

#ifndef NEBULA_SEYOND_COMMON_HPP
#define NEBULA_SEYOND_COMMON_HPP

#include <nebula_core_common/nebula_common.hpp>

#include <cstdint>
#include <cstdio>
#include <ostream>
#include <string>
#include <vector>

namespace nebula::drivers
{

/// @brief Sensor model types for Seyond LiDARs
enum class SeyondSensorModel : uint8_t {
  UNKNOWN = 0,
  FALCON_K,
  ROBIN_W,
  ROBIN_E1X,
  HUMMINGBIRD_D1
};

/// @brief Convert SeyondSensorModel enum to string
inline std::ostream & operator<<(std::ostream & os, const SeyondSensorModel & model)
{
  switch (model) {
    case SeyondSensorModel::FALCON_K:
      os << "FalconK";
      break;
    case SeyondSensorModel::ROBIN_W:
      os << "RobinW";
      break;
    case SeyondSensorModel::ROBIN_E1X:
      os << "RobinE1X";
      break;
    case SeyondSensorModel::HUMMINGBIRD_D1:
      os << "HummingbirdD1";
      break;
    default:
      os << "Unknown";
      break;
  }
  return os;
}

/// @brief Reflectance mode types for Seyond LiDARs
enum class SeyondReflectanceMode : uint8_t { NONE = 0, INTENSITY = 1, REFLECTIVITY = 2 };

/// @brief Sync mode types for Seyond LiDARs
enum class SeyondSyncMode : uint8_t { HOST = 0, PTP = 1, GPS = 2, EXTERNAL_FILE = 3, NTP = 4 };

/// @brief Convert sensor name to SeyondSensorModel enum
inline SeyondSensorModel seyond_sensor_model_from_string(const std::string & model_str)
{
  if (model_str == "FalconK" || model_str == "falcon_k") return SeyondSensorModel::FALCON_K;
  if (model_str == "RobinW" || model_str == "robin_w") return SeyondSensorModel::ROBIN_W;
  if (model_str == "RobinE1X" || model_str == "robin_e1x") return SeyondSensorModel::ROBIN_E1X;
  if (model_str == "HummingbirdD1" || model_str == "hummingbird_d1")
    return SeyondSensorModel::HUMMINGBIRD_D1;
  return SeyondSensorModel::UNKNOWN;
}

/// @brief Convert return mode name to ReturnMode enum
inline ReturnMode return_mode_from_string_seyond(const std::string & return_mode)
{
  if (return_mode == "Single" || return_mode == "1") return ReturnMode::STRONGEST;
  if (return_mode == "Dual" || return_mode == "2") return ReturnMode::DUAL;
  if (return_mode == "Triple" || return_mode == "3") return ReturnMode::TRIPLE;
  if (return_mode == "StrongestFurthest") return ReturnMode::DUAL_LAST_STRONGEST;
  return ReturnMode::UNKNOWN;
}

/// @brief Convert reflectance mode name to SeyondReflectanceMode enum
inline SeyondReflectanceMode reflectance_mode_from_string_seyond(
  const std::string & reflectance_mode)
{
  if (reflectance_mode == "Intensity") return SeyondReflectanceMode::INTENSITY;
  if (reflectance_mode == "Reflectivity") return SeyondReflectanceMode::REFLECTIVITY;
  return SeyondReflectanceMode::NONE;
}

/// @brief Convert sync mode name to SeyondSyncMode enum
inline SeyondSyncMode sync_mode_from_string_seyond(const std::string & sync_mode)
{
  if (sync_mode == "Host") return SeyondSyncMode::HOST;
  if (sync_mode == "PTP") return SeyondSyncMode::PTP;
  if (sync_mode == "GPS") return SeyondSyncMode::GPS;
  if (sync_mode == "File") return SeyondSyncMode::EXTERNAL_FILE;
  if (sync_mode == "NTP") return SeyondSyncMode::NTP;
  return SeyondSyncMode::HOST;
}

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_COMMON_HPP
