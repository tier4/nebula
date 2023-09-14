#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"

#include "nebula_decoders/nebula_decoders_robosense/decoders/helios.hpp"

namespace nebula
{
namespace drivers
{

RobosenseDriver::RobosenseDriver(
  const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<RobosenseCalibrationConfiguration> & calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
      //    case SensorModel::ROBOSENSE_BPEARL:
      //      scan_decoder_.reset(new RobosenseDecoder<Bpearl>(
      //        sensor_configuration, calibration_configuration, correction_configuration));
      //      break;
    case SensorModel::ROBOSENSE_HELIOS_5515:
      scan_decoder_.reset(
        new RobosenseDecoder<Helios>(sensor_configuration, calibration_configuration));
      break;

    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}

Status RobosenseDriver::GetStatus()
{
  return driver_status_;
}

Status RobosenseDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

std::tuple<drivers::NebulaPointCloudPtr, double> RobosenseDriver::ConvertScanToPointcloud(
  const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("RobosenseDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  int cnt = 0, last_azimuth = 0;
  for (auto & packet : pandar_scan->packets) {
    last_azimuth = scan_decoder_->unpack(packet);
    if (scan_decoder_->hasScanned()) {
      pointcloud = scan_decoder_->getPointcloud();
      cnt++;
    }
  }

  if (cnt == 0) {
    RCLCPP_ERROR_STREAM(
      logger, "Scanned " << pandar_scan->packets.size() << " packets, but no "
                         << "pointclouds were generated. Last azimuth: " << last_azimuth);
  }

  return pointcloud;
}

}  // namespace drivers
}  // namespace nebula
