#include "nebula_decoders/nebula_decoders_innovusion/innovusion_driver.hpp"

namespace nebula
{
namespace drivers
{

InnovusionDriver::InnovusionDriver(
  const std::shared_ptr<InnovusionSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<InnovusionCalibrationConfiguration> & calibration_configuration)
{
  scan_decoder_.reset(
    new innovusion_packet::InnovusionDecoder(sensor_configuration, calibration_configuration));
}

std::tuple<drivers::NebulaPointCloudPtr, double> InnovusionDriver::ConvertScanToPointcloud(
  const std::shared_ptr<innovusion_msgs::msg::InnovusionScan> & innovusion_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("InnovusionDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  int cnt = 0;
  for (auto & packet : innovusion_scan->packets) {
    if (scan_decoder_->unpack(packet) == 0) {
      cnt++;
    } else {
      RCLCPP_ERROR_STREAM(
        logger, "Failed to unpack packet with timestamp ");
    }
  }

  pointcloud = scan_decoder_->getPointcloud();

  if (cnt == 0) {
    RCLCPP_ERROR_STREAM(
      logger, "Scanned " << innovusion_scan->packets.size() << " packets, but no "
                         << "pointclouds were generated.");
  }

  return pointcloud;
}

Status InnovusionDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  // throw std::runtime_error(
  //   "SetCalibrationConfiguration. Not yet implemented (" +
  //   calibration_configuration.calibration_file + ")");
}

Status InnovusionDriver::GetStatus()
{
  return driver_status_;
}

}  // namespace drivers
}  // namespace nebula
