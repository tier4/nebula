#include "nebula_decoders/nebula_decoders_robosense/robosense_info_driver.hpp"

namespace nebula
{
namespace drivers
{

RobosenseInfoDriver::RobosenseInfoDriver(
  const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::ROBOSENSE_BPEARL_V3:
      info_decoder_.reset(new RobosenseInfoDecoder<BpearlV3>());
      break;
    case SensorModel::ROBOSENSE_BPEARL_V4:
      info_decoder_.reset(new RobosenseInfoDecoder<BpearlV4>());
      break;
    case SensorModel::ROBOSENSE_HELIOS:
      info_decoder_.reset(new RobosenseInfoDecoder<Helios>());
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

Status RobosenseInfoDriver::DecodeInfoPacket(const std::vector<uint8_t> & packet)
{
  const auto parsed = info_decoder_->parsePacket(packet);
  if (parsed) return nebula::Status::OK;
  return nebula::Status::ERROR_1;
}

std::map<std::string, std::string> RobosenseInfoDriver::GetSensorInfo()
{
  return info_decoder_->getSensorInfo();
}

ReturnMode RobosenseInfoDriver::GetReturnMode()
{
  return info_decoder_->getReturnMode();
}

RobosenseCalibrationConfiguration RobosenseInfoDriver::GetSensorCalibration()
{
  return info_decoder_->getSensorCalibration();
}

bool RobosenseInfoDriver::GetSyncStatus()
{
  return info_decoder_->getSyncStatus();
}

}  // namespace drivers
}  // namespace nebula
