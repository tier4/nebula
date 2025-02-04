// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"

#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v3.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v4.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/helios.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_decoder.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{

RobosenseDriver::RobosenseDriver(
  const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const RobosenseCalibrationConfiguration> & calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::ROBOSENSE_BPEARL_V3:
      scan_decoder_.reset(
        new RobosenseDecoder<BpearlV3>(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::ROBOSENSE_BPEARL_V4:
      scan_decoder_.reset(
        new RobosenseDecoder<BpearlV4>(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::ROBOSENSE_HELIOS:
      scan_decoder_.reset(
        new RobosenseDecoder<Helios>(sensor_configuration, calibration_configuration));
      break;
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
  }
}

Status RobosenseDriver::get_status()
{
  return driver_status_;
}

Status RobosenseDriver::set_calibration_configuration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "set_calibration_configuration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

std::tuple<drivers::NebulaPointCloudPtr, double> RobosenseDriver::parse_cloud_packet(
  const std::vector<uint8_t> & packet)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("RobosenseDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  scan_decoder_->unpack(packet);
  if (scan_decoder_->has_scanned()) {
    pointcloud = scan_decoder_->get_pointcloud();
  }

  return pointcloud;
}

}  // namespace nebula::drivers
