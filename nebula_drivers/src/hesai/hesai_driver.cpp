#include "hesai/hesai_driver.hpp"

#include "hesai/decoders/pandar_40_decoder.hpp"
#include "hesai/decoders/pandar_64_decoder.hpp"
#include "hesai/decoders/pandar_at_decoder.hpp"
#include "hesai/decoders/pandar_qt_decoder.hpp"
#include "hesai/decoders/pandar_xt_decoder.hpp"
#include "hesai/decoders/pandar_xtm_decoder.hpp"

namespace nebula
{
namespace drivers
{
HesaiDriver::HesaiDriver(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
  const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration)

{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::HESAI_PANDAR64:
      scan_decoder_.reset(
        new drivers::pandar_64::Pandar64Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
      scan_decoder_.reset(
        new drivers::pandar_40::Pandar40Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::HESAI_PANDARQT64:
      scan_decoder_.reset(
        new drivers::pandar_qt::PandarQTDecoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32:
      scan_decoder_.reset(
        new drivers::pandar_xt::PandarXTDecoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32M:
      scan_decoder_.reset(
        new drivers::pandar_xtm::PandarXTMDecoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::HESAI_PANDARAT128:
      scan_decoder_.reset(new drivers::pandar_at::PandarATDecoder(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARQT128:
    case SensorModel::HESAI_PANDAR128_V13:
    case SensorModel::HESAI_PANDAR128_V14:
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}

PointCloudXYZIRADTPtr HesaiDriver::ConvertScanToPointcloud(
  const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan)
{
  PointCloudXYZIRADTPtr pointcloud;
  if (driver_status_ == nebula::Status::OK) {
    for (auto & packet : pandar_scan->packets) {
      scan_decoder_->unpack(packet);
      if (scan_decoder_->hasScanned()) {
        pointcloud = scan_decoder_->get_pointcloud();
      }
    }
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

Status HesaiDriver::GetStatus() { return driver_status_; }

}  // namespace drivers
}  // namespace nebula
