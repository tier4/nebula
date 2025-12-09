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

#ifndef NEBULA_SAMPLE_COMMON_H
#define NEBULA_SAMPLE_COMMON_H

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

/// @brief Sensor-specific configuration for the Sample LiDAR
/// @details This struct extends LidarConfigurationBase with any sensor-specific settings.
/// When implementing for a real sensor, add fields for:
/// - Return mode (single, dual, strongest, last, etc.)
/// - Rotation speed / scan frequency
/// - IP addresses (sensor, host)
/// - Port numbers
/// - FOV settings
/// - Any vendor-specific parameters
struct SampleSensorConfiguration : public LidarConfigurationBase
{
  // Example field - replace with actual sensor parameters
  std::string sample_field;
};

inline std::ostream & operator<<(std::ostream & os, SampleSensorConfiguration const & arg)
{
  os << "Sample Sensor Configuration:" << '\n';
  os << static_cast<const LidarConfigurationBase &>(arg) << '\n';
  os << "Sample Field: " << arg.sample_field << '\n';
  return os;
}

/// @brief Calibration data for the Sample LiDAR (optional)
/// @details This struct is only needed if your sensor requires calibration data.
/// Common calibration data includes:
/// - Vertical/horizontal angle corrections per laser
/// - Distance corrections
/// - Intensity corrections
/// - Timing offsets
/// If your sensor doesn't need calibration, you can remove this struct entirely.
struct SampleCalibrationConfiguration : public CalibrationConfigurationBase
{
  /// @brief Load calibration data from a file
  /// @param calibration_file Path to the calibration file
  /// @return Status::OK on success, error status otherwise
  /// @details Implement parsing logic for your sensor's calibration file format (CSV, XML, binary,
  /// etc.)
  nebula::Status load_from_file(const std::string & calibration_file)
  {
    // Implementation Items: Implement calibration file parsing
    // Example: Parse CSV/XML/binary file containing angle corrections
    (void)calibration_file;
    return Status::OK;
  }

  /// @brief Save calibration data to a file
  /// @param calibration_file Path to save the calibration file
  /// @return Status::OK on success, error status otherwise
  /// @details Implement serialization logic for your sensor's calibration format
  nebula::Status save_to_file(const std::string & calibration_file)
  {
    // Implementation Items: Implement calibration file writing
    (void)calibration_file;
    return Status::OK;
  }
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_SAMPLE_COMMON_H
