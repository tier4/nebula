#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "configuration_base.hpp"
#include "nebula_status.hpp"

namespace nebula
{
namespace drivers
{
class NebulaDriverBase
{
public:
  NebulaDriverBase(NebulaDriverBase && c) = delete;
  NebulaDriverBase & operator=(NebulaDriverBase && c) = delete;
  NebulaDriverBase(const NebulaDriverBase & c) = delete;
  NebulaDriverBase & operator=(const NebulaDriverBase & c) = delete;

  NebulaDriverBase() = default;
  NebulaDriverBase(
    const CloudConfigurationBase & cloud_configuration,
    const CalibrationConfigurationBase & calibration_configuration);

  virtual Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) = 0;

  virtual Status SetCloudConfiguration(const CloudConfigurationBase & cloud_configuration) = 0;

  virtual Status GetCloudConfiguration(CloudConfigurationBase & cloud_configuration) = 0;

  // Lidar specific conversion of Packet to Pointcloud2 Msg
  // Header information should be filled in by the DriverBaseWrapper
  virtual std::shared_ptr<sensor_msgs::msg::PointCloud2> ParsePacketToPointcloud(
    std::vector<uint8_t> & packet) = 0;

private:
  CloudConfigurationBase cloud_configuration_;
  CalibrationConfigurationBase calibration_configuration_;
};

NebulaDriverBase::NebulaDriverBase(
  const CloudConfigurationBase & cloud_configuration,
  const CalibrationConfigurationBase & calibration_configuration)
: cloud_configuration_(cloud_configuration), calibration_configuration_(calibration_configuration)
{
}

}  // namespace drivers
}  // namespace nebula
#endif  //NEBULA_DRIVER_BASE_H
