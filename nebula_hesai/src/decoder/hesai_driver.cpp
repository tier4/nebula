// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e3x.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e4x.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_40.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_64.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_at128.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt128.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt64.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt16.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32m.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
HesaiDriver::HesaiDriver(
  const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const HesaiCalibrationConfigurationBase> & calibration_data,
  const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;

  switch (sensor_configuration->sensor_model) {
    case SensorModel::HESAI_PANDAR64:
      scan_decoder_ = initialize_decoder<Pandar64>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
      scan_decoder_ = initialize_decoder<Pandar40>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARQT64:
      scan_decoder_ = initialize_decoder<PandarQT64>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARQT128:
      scan_decoder_ = initialize_decoder<PandarQT128>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARXT16:
      scan_decoder_ = initialize_decoder<PandarXT16>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARXT32:
      scan_decoder_ = initialize_decoder<PandarXT32>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARXT32M:
      scan_decoder_ = initialize_decoder<PandarXT32M>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDARAT128:
      scan_decoder_ = initialize_decoder<PandarAT128>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDAR128_E3X:
      scan_decoder_ = initialize_decoder<Pandar128E3X>(sensor_configuration, calibration_data);
      break;
    case SensorModel::HESAI_PANDAR128_E4X:
      scan_decoder_ = initialize_decoder<Pandar128E4X>(sensor_configuration, calibration_data);
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
std::shared_ptr<HesaiScanDecoder> HesaiDriver::initialize_decoder(
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
    calibration_configuration)
{
  using CalibT = typename SensorT::angle_corrector_t::correction_data_t;
  return std::make_shared<HesaiDecoder<SensorT>>(
    sensor_configuration, std::dynamic_pointer_cast<const CalibT>(calibration_configuration),
    logger_->child("Decoder"));
}

std::tuple<drivers::NebulaPointCloudPtr, double> HesaiDriver::parse_cloud_packet(
  const std::vector<uint8_t> & packet)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;

  if (driver_status_ != nebula::Status::OK) {
    logger_->error("Driver not OK.");
    return pointcloud;
  }

  scan_decoder_->unpack(packet);
  if (scan_decoder_->has_scanned()) {
    pointcloud = scan_decoder_->get_pointcloud();
  }

  return pointcloud;
}

Status HesaiDriver::set_calibration_configuration(
  const HesaiCalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "set_calibration_configuration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

Status HesaiDriver::get_status()
{
  return driver_status_;
}

}  // namespace nebula::drivers
