#ifndef NEBULA_VELODYNE_DRIVER_H
#define NEBULA_VELODYNE_DRIVER_H

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include "common/nebula_common.hpp"
#include "common/nebula_driver_base.hpp"
#include "common/nebula_status.hpp"
#include "common/point_types.hpp"
#include "velodyne/decoders/velodyne_scan_decoder.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace nebula
{
namespace drivers
{
class VelodyneDriver : NebulaDriverBase
{
private:
  Status driver_status_;
  std::shared_ptr<drivers::VelodyneScanDecoder> scan_decoder_;

public:
  VelodyneDriver() = delete;
  VelodyneDriver(
    const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration);

  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  Status GetStatus();

  PointCloudXYZIRADTPtr ConvertScanToPointcloud(
    const std::shared_ptr<velodyne_msgs::msg::VelodyneScan> & velodyne_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_DRIVER_H
