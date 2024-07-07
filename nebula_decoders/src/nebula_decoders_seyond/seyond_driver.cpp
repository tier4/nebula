#include "nebula_decoders/nebula_decoders_seyond/seyond_driver.hpp"

namespace nebula
{
namespace drivers
{

SeyondDriver::SeyondDriver(
  const std::shared_ptr<const SeyondSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const SeyondCalibrationConfiguration> & calibration_configuration)
{
  // scan_decoder_.reset(
  //   new seyond_packet::SeyondDecoder(sensor_configuration, calibration_configuration));
  scan_decoder_ = InitializeDecoder(sensor_configuration, calibration_configuration);
}

  std::shared_ptr<SeyondScanDecoder> SeyondDriver::InitializeDecoder(
    const std::shared_ptr<const drivers::SeyondSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::SeyondCalibrationConfiguration> &
      calibration_configuration)
{
  return std::make_shared<seyond_packet::SeyondDecoder>(
    sensor_configuration, calibration_configuration);
}

std::tuple<drivers::NebulaPointCloudPtr, double> SeyondDriver::ParseCloudPacket(
  const std::vector<uint8_t> & packet)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("SeyondDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  scan_decoder_->unpack(packet);
  if (scan_decoder_->hasScanned()) {
    pointcloud = scan_decoder_->getPointcloud();
  }

  // int cnt = 0;
  // for (auto & packet : seyond_scan->packets) {
  //   if (scan_decoder_->unpack(packet) == 0) {
  //     cnt++;
  //   } else {
  //     RCLCPP_ERROR_STREAM(
  //       logger, "Failed to unpack packet with timestamp ");
  //   }
  // }

  // pointcloud = scan_decoder_->getPointcloud();

  // if (cnt == 0) {
  //   RCLCPP_ERROR_STREAM(
  //     logger, "Scanned " << seyond_scan->packets.size() << " packets, but no "
  //                        << "pointclouds were generated.");
  // }

  return pointcloud;
}

Status SeyondDriver::SetCalibrationConfiguration(
  const SeyondCalibrationConfiguration & calibration_configuration)
{

}

Status SeyondDriver::GetStatus()
{
  return driver_status_;
}

}  // namespace drivers
}  // namespace nebula