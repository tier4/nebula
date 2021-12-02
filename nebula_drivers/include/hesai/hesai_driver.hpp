#ifndef NEBULA_HESAI_DRIVER_H
#define NEBULA_HESAI_DRIVER_H

#include "common/nebula_common.hpp"
#include "common/nebula_driver_base.hpp"
#include "common/nebula_status.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/point_types.hpp"
#include "hesai/scan_decoder.hpp"

#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

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
  HesaiDriver();
  HesaiDriver(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCloudConfiguration> & cloud_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;
  Status SetCloudConfiguration(const CloudConfigurationBase & cloud_configuration) override;

  Status GetStatus();

  sensor_msgs::msg::PointCloud2 ParsePacketToPointcloud(
    std::vector<pandar_msgs::msg::PandarPacket> & packets);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_DRIVER_H
