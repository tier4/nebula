#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

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

  // Lidar specific conversion of Packet to Pointcloud2 Msg
  // Header information should be filled in by the DriverBaseWrapper
  //  virtual std::shared_ptr<sensor_msgs::msg::PointCloud2> ConvertScanToPointcloud(
  //    PACKET_MSG) = 0;
};

}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_DRIVER_BASE_H
