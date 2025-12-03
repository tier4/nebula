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

struct SampleSensorConfiguration : public LidarConfigurationBase
{
  // Add any dummy specific configuration here if needed
  std::string sample_field;
};

inline std::ostream & operator<<(std::ostream & os, SampleSensorConfiguration const & arg)
{
  os << "Sample Sensor Configuration:" << '\n';
  os << static_cast<const LidarConfigurationBase &>(arg) << '\n';
  os << "Sample Field: " << arg.sample_field << '\n';
  return os;
}

struct SampleCalibrationConfiguration : public CalibrationConfigurationBase
{
  nebula::Status load_from_file(const std::string & calibration_file)
  {
    // Sample implementation
    return Status::OK;
  }

  nebula::Status save_to_file(const std::string & calibration_file)
  {
    // Sample implementation
    return Status::OK;
  }
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_SAMPLE_COMMON_H
