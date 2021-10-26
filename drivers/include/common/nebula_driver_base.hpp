#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include "configuration_base.hpp"
#include "status.hpp"

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

  NebulaDriverBase(
    CloudConfigurationBase & cloud_configuration,
    CalibrationConfigurationBase & calibration_configuration);

  virtual STATUS SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration);
  virtual STATUS SetCloudConfiguration(const CloudConfigurationBase & cloud_configuration);
  virtual STATUS GetCloudConfiguration(CloudConfigurationBase & cloud_configuration);
  // Lidar specific conversion of Packet to Pointcloud2 Msg
  // Header information should be filled in by the DriverBaseWrapper
  virtual sensor_msgs::msg::PointCloud2 ParsePacketToPointcloud(std::vector<uint8_t> & packet);

private:
  CloudConfigurationBase cloud_configuration_;
  CalibrationConfigurationBase calibration_configuration_;
};

}  // namespace drivers
}  // namespace nebula
#endif  //NEBULA_DRIVER_BASE_H
