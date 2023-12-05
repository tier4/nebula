#ifndef NEBULA_INNOVUSION_COMMON_H
#define NEBULA_INNOVUSION_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <fstream>
#include <sstream>
namespace nebula
{
namespace drivers
{
/// @brief struct for Innovusion sensor configuration
struct InnovusionSensorConfiguration : SensorConfigurationBase
{
  double cloud_min_range;
  double cloud_max_range;
};
/// @brief Convert InnovusionSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, InnovusionSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", cloud_min_range: " << arg.cloud_min_range
     << ", cloud_max_range: " << arg.cloud_max_range << "\n";
  return os;
}

/// @brief struct for Innovusion calibration configuration
struct InnovusionCalibrationConfiguration : CalibrationConfigurationBase
{
  // InnovusionCalibration innovusion_calibration;
  inline nebula::Status LoadFromFile(const std::string &)
  {
    return Status::OK;
  }
  inline nebula::Status SaveFile(const std::string &)
  {
    return Status::OK;
  }
};

/// @brief Convert return mode name to ReturnMode enum (Innovusion-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringInnovusion(const std::string & return_mode)
{
  if (return_mode == "Strongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "Last") return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual") return ReturnMode::DUAL_ONLY;

  return ReturnMode::UNKNOWN;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_INNOVUSION_COMMON_H
