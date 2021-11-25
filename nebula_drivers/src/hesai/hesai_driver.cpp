#include "hesai/hesai_driver.hpp"

namespace nebula
{
namespace drivers
{
HesaiDriver::HesaiDriver() = default;

HesaiDriver::HesaiDriver(
  const std::shared_ptr<drivers::HesaiCloudConfiguration> & cloud_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration
)
{
  // initialize proper parser from cloud config's model and echo mode

}

Status HesaiDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error("SetCalibrationConfiguration. Not yet implemented");
}

Status HesaiDriver::SetCloudConfiguration(
  const nebula::drivers::CloudConfigurationBase & cloud_configuration)
{
  throw std::runtime_error("SetCalibrationConfiguration. Not yet implemented");
}\

sensor_msgs::msg::PointCloud2 HesaiDriver::ParsePacketToPointcloud(
  std::vector<pandar_msgs::msg::PandarPacket> & packets)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  return pointcloud;
}

}  // namespace drivers
}  // namespace nebula