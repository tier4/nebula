#include "velodyne/velodyne_driver.hpp"
#include "velodyne/decoders/vls128_decoder.hpp"
#include "velodyne/decoders/vlp32_decoder.hpp"
#include "velodyne/decoders/vlp16_decoder.hpp"

namespace nebula
{
namespace drivers
{

VelodyneDriver::VelodyneDriver(
const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration)
{
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  std::cout << "sensor_configuration->sensor_model=" << sensor_configuration->sensor_model << std::endl;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::VELODYNE_VLS128:
      scan_decoder_.reset(new drivers::vls128::Vls128Decoder(sensor_configuration,
                                                         calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP32:
    case SensorModel::VELODYNE_HDL64:
    case SensorModel::VELODYNE_HDL32:
      scan_decoder_.reset(new drivers::vlp32::Vlp32Decoder(sensor_configuration,
                                                             calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP16:
      scan_decoder_.reset(new drivers::vlp16::Vlp16Decoder(sensor_configuration,
                                                             calibration_configuration));
      break;
    default:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
  }
}

Status VelodyneDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error("SetCalibrationConfiguration. Not yet implemented (" + calibration_configuration.calibration_file + ")");
}

PointCloudXYZIRADTPtr VelodyneDriver::ConvertScanToPointcloud(
  const std::shared_ptr<velodyne_msgs::msg::VelodyneScan>& velodyne_scan)
{
  PointCloudXYZIRADTPtr pointcloud;
  if(driver_status_ == nebula::Status::OK)
  {
//    std::cout << "velodyne_scan->packets.size() = " << velodyne_scan->packets.size() << std::endl;
    scan_decoder_->reset_pointcloud(velodyne_scan->packets.size());
//    int cnt = 0;
    for (auto& packet : velodyne_scan->packets) {
      scan_decoder_->unpack(packet);
//      cnt++;
    }
//    std::cout << "cnt = " << cnt << std::endl;
    pointcloud = scan_decoder_->get_pointcloud();
//    std::cout << "pointcloud->size() = " << pointcloud->size() << std::endl;
  }else{
    std::cout << "not ok driver_status_ = " << driver_status_ << std::endl;
  }
  return pointcloud;

}
Status VelodyneDriver::GetStatus() { return driver_status_; }

}  // namespace drivers
}  // namespace nebula
