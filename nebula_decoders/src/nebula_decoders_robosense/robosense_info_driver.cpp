// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_robosense/robosense_info_driver.hpp"

#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v3.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v4.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/helios.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
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

Status RobosenseInfoDriver::get_status()
{
  return driver_status_;
}

Status RobosenseInfoDriver::decode_info_packet(const std::vector<uint8_t> & packet)
{
  const auto parsed = info_decoder_->parse_packet(packet);
  if (parsed) return nebula::Status::OK;
  return nebula::Status::ERROR_1;
}

std::map<std::string, std::string> RobosenseInfoDriver::get_sensor_info()
{
  return info_decoder_->get_sensor_info();
}

ReturnMode RobosenseInfoDriver::get_return_mode()
{
  return info_decoder_->get_return_mode();
}

RobosenseCalibrationConfiguration RobosenseInfoDriver::get_sensor_calibration()
{
  return info_decoder_->get_sensor_calibration();
}

bool RobosenseInfoDriver::get_sync_status()
{
  return info_decoder_->get_sync_status();
}

}  // namespace nebula::drivers
