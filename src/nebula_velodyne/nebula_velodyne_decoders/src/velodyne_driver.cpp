// Copyright 2024 TIER IV, Inc.

#include "nebula_velodyne_decoders/velodyne_driver.hpp"

#include "nebula_velodyne_decoders/decoders/vlp16_decoder.hpp"
#include "nebula_velodyne_decoders/decoders/vlp32_decoder.hpp"
#include "nebula_velodyne_decoders/decoders/vls128_decoder.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
VelodyneDriver::VelodyneDriver(
  const std::shared_ptr<const drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> &
    calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::VELODYNE_VLS128:
      scan_decoder_.reset(
        new drivers::vls128::Vls128Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP32:
    case SensorModel::VELODYNE_HDL64:
    case SensorModel::VELODYNE_HDL32:
      scan_decoder_.reset(
        new drivers::vlp32::Vlp32Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP16:
      scan_decoder_.reset(
        new drivers::vlp16::Vlp16Decoder(sensor_configuration, calibration_configuration));
      break;
    default:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
  }
}

Status VelodyneDriver::set_calibration_configuration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "set_calibration_configuration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

std::tuple<drivers::NebulaPointCloudPtr, double> VelodyneDriver::parse_cloud_packet(
  const std::vector<uint8_t> & packet, double packet_seconds)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;

  if (driver_status_ != nebula::Status::OK) {
    auto logger = rclcpp::get_logger("VelodyneDriver");
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  switch (scan_decoder_->unpack(packet, packet_seconds)) {
    case PacketDecodeStatus::OK:
      break;
    case PacketDecodeStatus::INVALID_PACKET_SIZE:
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("VelodyneDriver"), clock_, 5000,
        "Packet size mismatch: got %zu bytes, expected %d bytes. Dropping packet.", packet.size(),
        g_packet_size);
      return pointcloud;
    case PacketDecodeStatus::INVALID_BLOCK_HEADER:
      // The decoder skipped the rest of the packet; the scan state is still updated.
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("VelodyneDriver"), clock_, 5000,
        "Unrecognized block header, skipping the rest of the packet.");
      break;
  }

  if (scan_decoder_->has_scanned()) {
    pointcloud = scan_decoder_->get_pointcloud();
  }

  return pointcloud;
}
Status VelodyneDriver::get_status()
{
  return driver_status_;
}

}  // namespace nebula::drivers
