#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_bpearl_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_e1_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_helios16P_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_helios_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_m1plus_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_mech_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_rubyplus_decoder.hpp"

namespace nebula
{
namespace drivers
{

RobosenseDriver::RobosenseDriver(
  const std::shared_ptr<drivers::RobosenseSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::RobosenseCalibrationConfiguration> & calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  RSDecoderParam param;
  // param.wait_for_difop = false;
  param.start_angle = sensor_configuration->cloud_min_angle;
  param.end_angle = sensor_configuration->cloud_max_angle;
  if (sensor_configuration->difop_port > 0) {
    param.wait_for_difop = true;
  } else {
    param.config_from_file = true;
    param.angle_path = calibration_configuration->calibration_file;
  }
  param.min_distance = 0.2;
  param.max_distance = 200;
  param.use_lidar_clock = true;
  param.dense_points = true;
  param.print();

  driver_status_ = nebula::Status::OK;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::ROBOSENSE_HELIOS:
      scan_decoder_.reset(new drivers::robosense_helios::RobosenseHeliosDecoder(param));
      break;
    case SensorModel::ROBOSENSE_HELIOS16P:
      scan_decoder_.reset(new drivers::robosense_helios16P::RobosenseHelios16PDecoder(param));
      break;
    case SensorModel::ROBOSENSE_BPEARL:
      scan_decoder_.reset(new drivers::robosense_bpearl::RobosenseBpearlDecoder(param));
      break;
    case SensorModel::ROBOSENSE_M1PLUS:
      scan_decoder_.reset(new drivers::robosense_m1plus::RobosenseM1PlusDecoder(param));
      break;
    case SensorModel::ROBOSENSE_E1:
      scan_decoder_.reset(new drivers::robosense_e1::RobosenseE1Decoder(param));
      break;
    case SensorModel::ROBOSENSE_RUBYPLUS:
      scan_decoder_.reset(new drivers::robosense_rubyplus::RobosenseRubyPlusDecoder(param));
      break;
    default:
      driver_status_ = nebula::Status::NOT_INITIALIZED;
      throw std::runtime_error("Driver not Implemented for selected sensor.");
      break;
  }
}
std::tuple<drivers::NebulaPointCloudPtr, double> RobosenseDriver::ConvertScanToPointcloud(
  const std::shared_ptr<robosense_msgs::msg::RobosenseScan> & robosense_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  static const uint8_t msop_id[] = {0x55, 0xAA};
  static const uint8_t difop_id[] = {0xA5, 0xFF};
  if (driver_status_ == nebula::Status::OK) {
    for (auto & packet : robosense_scan->packets) {
      uint8_t * id = &packet.data[0];
      if (memcmp(id, msop_id, sizeof(msop_id)) == 0) {
        scan_decoder_->processMsopPkt(&packet.data[0], packet.size);
      } else if (memcmp(id, difop_id, sizeof(difop_id)) == 0) {
        scan_decoder_->processDifopPkt(&packet.data[0], packet.size);
        if (difop_callback_) {
          AllDeviceInfo device_info;
          if (scan_decoder_->getDeviceInfo(device_info)) {
            difop_callback_(device_info);
          }
        }
      }
      if (scan_decoder_->isSpliteFrame()) {
        pointcloud = scan_decoder_->get_pointcloud();
      }
    }
  }
  nebula::drivers::NebulaPointCloudPtr pointcloud1 = std::get<0>(pointcloud);
  // if (pointcloud1->points.size() != 78750) {
  //   RS_DEBUG << "pointcloud->points.size():" << pointcloud1->points.size() << RS_REND;
  // }
  return pointcloud;
}

Status RobosenseDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

Status RobosenseDriver::GetStatus()
{
  return driver_status_;
}

void RobosenseDriver::registDifopCallback(
  const std::function<void(const AllDeviceInfo &)> & callback)
{
  difop_callback_ = callback;
}

}  // namespace drivers
}  // namespace nebula
