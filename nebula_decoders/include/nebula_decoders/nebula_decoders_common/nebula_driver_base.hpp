#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
/// @brief Base class for each sensor driver
class NebulaDriverBase
{
public:
  NebulaDriverBase(NebulaDriverBase && c) = delete;
  NebulaDriverBase & operator=(NebulaDriverBase && c) = delete;
  NebulaDriverBase(const NebulaDriverBase & c) = delete;
  NebulaDriverBase & operator=(const NebulaDriverBase & c) = delete;

  NebulaDriverBase() = default;

  /// @brief Virtual function for setting calibration configuration
  /// @param calibration_configuration CalibrationConfiguration including file path
  /// @return Resulting status
  virtual Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) = 0;
};

}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_DRIVER_BASE_H
