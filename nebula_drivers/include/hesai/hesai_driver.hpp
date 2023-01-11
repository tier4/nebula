#ifndef NEBULA_HESAI_DRIVER_H
#define NEBULA_HESAI_DRIVER_H

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include "common/nebula_common.hpp"
#include "common/nebula_driver_base.hpp"
#include "common/nebula_status.hpp"
#include "common/point_types.hpp"
#include "hesai/decoders/hesai_scan_decoder.hpp"
#include "hesai/hesai_common.hpp"
#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
class HesaiDriver : NebulaDriverBase
{
private:
  Status driver_status_;
  std::shared_ptr<drivers::HesaiScanDecoder> scan_decoder_;

public:
  HesaiDriver() = delete;
  explicit HesaiDriver(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
    const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration = nullptr);

  Status GetStatus();

  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  PointCloudXYZIRADTPtr ConvertScanToPointcloud(
    const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_DRIVER_H
