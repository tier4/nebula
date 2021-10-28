#include "hesai/nebula_hesai_driver.hpp"

namespace nebula
{
namespace drivers
{

Status NebulaHesaiDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error("SetCalibrationConfiguration. Not yet implemented");
}

Status NebulaHesaiDriver::SetCloudConfiguration(
  const nebula::drivers::CloudConfigurationBase & cloud_configuration)
{
  throw std::runtime_error("SetCalibrationConfiguration. Not yet implemented");
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> NebulaHesaiDriver::ParsePacketToPointcloud(
  std::vector<uint8_t> & packet)
{
  throw std::runtime_error("ParsePacketToPointcloud. Not yet implemented");
}

NebulaHesaiDriver::NebulaHesaiDriver() {}

NebulaHesaiDriver::NebulaHesaiDriver(
  const CalibrationConfigurationBase & calibration_configuration,
  const CloudConfigurationBase & cloud_configuration)
{
  std::cout << calibration_configuration.calibration_file;
  std::cout << cloud_configuration.cloud_min_range;
}

}  // namespace drivers
}  // namespace nebula