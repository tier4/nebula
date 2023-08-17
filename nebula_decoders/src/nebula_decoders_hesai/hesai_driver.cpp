#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e3x.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e4x.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_40.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_64.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_at128.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt128.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt64.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32m.hpp"

// #define WITH_DEBUG_STD_COUT_HESAI_CLIENT // Use std::cout messages for debugging

namespace nebula
{
namespace drivers
{
HesaiDriver::HesaiDriver(
  const std::shared_ptr<HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<HesaiCalibrationConfiguration> & calibration_configuration,
  const std::shared_ptr<HesaiCorrection> & correction_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::HESAI_PANDAR64:
      scan_decoder_.reset(new HesaiDecoder<Pandar64>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
      scan_decoder_.reset(new HesaiDecoder<Pandar40>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARQT64:
      scan_decoder_.reset(new HesaiDecoder<PandarQT64>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARQT128:
      scan_decoder_.reset(new HesaiDecoder<PandarQT128>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32:
      scan_decoder_.reset(new HesaiDecoder<PandarXT32>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32M:
      scan_decoder_.reset(new HesaiDecoder<PandarXT32M>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARAT128:
      scan_decoder_.reset(new HesaiDecoder<PandarAT128>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDAR128_E3X:
      scan_decoder_.reset(new HesaiDecoder<Pandar128E3X>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDAR128_E4X:
      scan_decoder_.reset(new HesaiDecoder<Pandar128E4X>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}

std::tuple<drivers::NebulaPointCloudPtr, double> HesaiDriver::ConvertScanToPointcloud(
  const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  auto logger = rclcpp::get_logger("HesaiDriver");

  if (driver_status_ != nebula::Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return pointcloud;
  }

  int cnt = 0, last_azimuth = 0;
  for (auto & packet : pandar_scan->packets) {
    last_azimuth = scan_decoder_->unpack(packet);
    if (scan_decoder_->hasScanned()) {
      pointcloud = scan_decoder_->getPointcloud();
      cnt++;
    }
  }

  if (cnt == 0) {
    RCLCPP_ERROR_STREAM(
      logger, "Scanned " << pandar_scan->packets.size() << " packets, but no "
                         << "pointclouds were generated. Last azimuth: " << last_azimuth);
  }

  return pointcloud;
}

Status HesaiDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

Status HesaiDriver::GetStatus()
{
  return driver_status_;
}

}  // namespace drivers
}  // namespace nebula
