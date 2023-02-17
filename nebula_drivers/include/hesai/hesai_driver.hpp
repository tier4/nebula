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
/// @brief Hesai driver
class HesaiDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<drivers::HesaiScanDecoder> scan_decoder_;

public:
  HesaiDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @param correction_configuration CorrectionConfiguration for this driver (for AT)
  explicit HesaiDriver(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
    const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration = nullptr);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Convert PandarScan message to point cloud
  /// @param pandar_scan Message
  /// @return Point cloud
  PointCloudXYZIRADTPtr ConvertScanToPointcloud(
    const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_DRIVER_H
