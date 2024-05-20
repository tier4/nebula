#include "nebula_decoders/nebula_decoders_seyond/seyond_driver.hpp"

#include "nebula_decoders/nebula_decoders_seyond/decoders/falcon.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/robin_w.hpp"

// #define WITH_DEBUG_STD_COUT_SEYOND_CLIENT // Use std::cout messages for debugging

namespace nebula
{
namespace drivers
{
SeyondDriver::SeyondDriver(
  const std::shared_ptr<const SeyondSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const SeyondCalibrationConfigurationBase> & calibration_data)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;

  switch (sensor_configuration->sensor_model) {
    case SensorModel::SEYOND_FALCON_KINETIC:
      scan_decoder_ = InitializeDecoder<Falcon>(sensor_configuration, calibration_data);
      break;
    case SensorModel::SEYOND_ROBIN_W:
      scan_decoder_ = InitializeDecoder<RobinW>(sensor_configuration, calibration_data);
      break;
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      throw std::runtime_error("Invalid sensor model.");
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
  }
}

template <typename SensorT>
std::shared_ptr<SeyondScanDecoder> SeyondDriver::InitializeDecoder(
  const std::shared_ptr<const drivers::SeyondSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const drivers::SeyondCalibrationConfigurationBase> &
    calibration_configuration)
{
  // using CalibT = typename SensorT::angle_corrector_t::correction_data_t;
  // return std::make_shared<SeyondDecoder<SensorT>>(
  //   sensor_configuration, std::dynamic_pointer_cast<const CalibT>(calibration_configuration));
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

  return pointcloud;
}

Status SeyondDriver::SetCalibrationConfiguration(
  const SeyondCalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

Status SeyondDriver::GetStatus()
{
  return driver_status_;
}

}  // namespace drivers
}  // namespace nebula
