#include "nebula_decoders/nebula_decoders_robosense/robosense_info_driver.hpp"

namespace nebula
{
namespace drivers
{

RobosenseInfoDriver::RobosenseInfoDriver(
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
      info_decoder_.reset(
        new RobosenseInfoDecoder<Helios>(sensor_configuration, calibration_configuration));
      break;

    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}

Status RobosenseInfoDriver::GetStatus()
{
  return driver_status_;
}

Status RobosenseInfoDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

Status RobosenseInfoDriver::DecodeInfoPacket(const std::vector<uint8_t> & packet)
{
  const auto parsed = info_decoder_->parsePacket(packet);
  std::cout << "parsed: " << parsed << std::endl;
  if (parsed) return nebula::Status::OK;
  return nebula::Status::ERROR_1;
}
std::map<std::string, std::string> RobosenseInfoDriver::GetSensorInfo()
{
  return info_decoder_->getSensorInfo();
}

}  // namespace drivers
}  // namespace nebula
