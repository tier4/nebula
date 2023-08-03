#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"

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
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::Packet64>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::Packet40P>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARQT64:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::PacketQT64>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARQT128:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::PacketQT128C2X>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::PacketXT32>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARXT32M:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::PacketXT32M2X>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDARAT128:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::PacketAT128E2X>(
        sensor_configuration, calibration_configuration, correction_configuration));
      break;
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
      scan_decoder_.reset(new HesaiDecoder<hesai_packet::Packet128E3X>(
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

  if (driver_status_ == nebula::Status::OK) {
    int cnt = 0, last_azimuth;
    for (auto & packet : pandar_scan->packets) {
      last_azimuth = scan_decoder_->unpack(packet);
      if (scan_decoder_->hasScanned()) {
        pointcloud = scan_decoder_->getPointcloud();
        cnt++;
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

Status HesaiDriver::GetStatus()
{
  return driver_status_;
}

}  // namespace drivers
}  // namespace nebula
