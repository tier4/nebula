#include "hesai/hesai_driver.hpp"

namespace nebula
{
namespace drivers
{
HesaiDriver::HesaiDriver() = default;

HesaiDriver::HesaiDriver(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCloudConfiguration> & cloud_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode

  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      break;
    case SensorModel::HESAI_PANDAR64:
      break;
    case SensorModel::HESAI_PANDAR40P:
      break;
    case SensorModel::HESAI_PANDAR40M:
      break;
    case SensorModel::HESAI_PANDARQT64:
      break;
    case SensorModel::HESAI_PANDARQT128:
      break;
    case SensorModel::HESAI_PANDARXT32:
      break;
    case SensorModel::HESAI_PANDAR128_V13:
      break;
    case SensorModel::HESAI_PANDAR128_V14:
      break;
  }
  driver_status_ = nebula::Status::OK;
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
}

sensor_msgs::msg::PointCloud2 HesaiDriver::ParsePacketToPointcloud(
  std::vector<pandar_msgs::msg::PandarPacket> & packets)
{
  for(auto packet: packets)
  {

  }
  sensor_msgs::msg::PointCloud2 pointcloud;
  return pointcloud;
}
Status HesaiDriver::GetStatus() { return driver_status_; }

}  // namespace drivers
}  // namespace nebula