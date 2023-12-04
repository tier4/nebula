#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"

#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v3.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v4.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/helios.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/m1.hpp"

namespace nebula
{
namespace drivers
{

RobosenseDriver::RobosenseDriver(
  const std::shared_ptr<RobosenseSensorConfiguration> sensor_configuration,
  const std::shared_ptr<RobosenseCalibrationConfiguration> calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      std::cerr << "Invalid sensor model: " << sensor_configuration->sensor_model << std::endl;
      break;
    case SensorModel::ROBOSENSE_BPEARL_V3: {
      std::shared_ptr<BpearlV3> sensor = std::make_shared<BpearlV3>(sensor_configuration, calibration_configuration);
      scan_decoder_.reset(new RobosenseDecoder<BpearlV3>(sensor_configuration, sensor));
    } break;
    case SensorModel::ROBOSENSE_BPEARL_V4: {
      std::shared_ptr<BpearlV4> sensor = std::make_shared<BpearlV4>(sensor_configuration, calibration_configuration);
      scan_decoder_.reset(new RobosenseDecoder<BpearlV4>(sensor_configuration, sensor));
    } break;
    case SensorModel::ROBOSENSE_HELIOS: {
      std::shared_ptr<Helios> sensor = std::make_shared<Helios>(sensor_configuration, calibration_configuration);
      scan_decoder_.reset(new RobosenseDecoder<Helios>(sensor_configuration, sensor));
    } break;
    case SensorModel::ROBOSENSE_M1: {
      std::shared_ptr<M1> sensor = std::make_shared<M1>();
      scan_decoder_.reset(new RobosenseDecoder<M1>(sensor_configuration, sensor));
    } break;
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
  }
}

Status RobosenseDriver::GetStatus()
{
  return driver_status_;
}

bool RobosenseDriver::HasScanned()
{
  return scan_decoder_->hasScanned();
}

Status RobosenseDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

std::tuple<drivers::NebulaPointCloudPtr, double> RobosenseDriver::ConvertScanToPointcloud(
  const std::shared_ptr<robosense_msgs::msg::RobosenseScan> & robosense_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("RobosenseDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  int cnt = 0, last_azimuth = 0;
  for (auto & packet : robosense_scan->packets) {
    last_azimuth = scan_decoder_->unpack(packet);
    if (scan_decoder_->hasScanned()) {
      pointcloud = scan_decoder_->getPointcloud();
      cnt++;
    }
  }

  // if (cnt == 0) {
  //   RCLCPP_ERROR_STREAM(
  //     logger, "Scanned " << robosense_scan->packets.size() << " packets, but no "
  //                        << "pointclouds were generated. Last azimuth: " << last_azimuth);
  // }

  return pointcloud;
}

}  // namespace drivers
}  // namespace nebula
