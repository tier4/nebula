#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

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

  virtual Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) = 0;

  virtual Status SetCloudConfiguration(const CloudConfigurationBase & cloud_configuration) = 0;

  // Lidar specific conversion of Packet to Pointcloud2 Msg
  // Header information should be filled in by the DriverBaseWrapper
  //  virtual std::shared_ptr<sensor_msgs::msg::PointCloud2> ConvertScanToPointcloud(
  //    PACKET_MSG) = 0;
};

}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_DRIVER_BASE_H
