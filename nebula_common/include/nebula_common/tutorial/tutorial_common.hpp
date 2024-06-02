#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include "nebula_common/hesai/hesai_common.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
namespace nebula
{
namespace drivers
{

using TutorialSensorConfiguration = typename nebula::drivers::HesaiSensorConfiguration;

struct ExampleSensorConfiguration : public LidarConfigurationBase
{
  /// @brief UDP Port for GNSS data packets
  uint16_t gnss_port;
  /// @brief At which angle the sensor syncs to the 1.00000...s mark
  double scan_phase;
  /// @brief The distance in m below which two points from the same ray are fused into one
  double dual_return_distance_threshold;
  /// @brief The motor RPM. This directly influences the output frequency of pointclouds
  uint16_t rotation_speed;
  /// @brief The angle at which to start outputting points
  uint16_t cloud_min_angle;
  /// @brief The angle at which to stop outputting points
  uint16_t cloud_max_angle;
};

}  // namespace drivers
}  // namespace nebula