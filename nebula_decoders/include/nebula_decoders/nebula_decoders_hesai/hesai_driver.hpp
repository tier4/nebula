#ifndef NEBULA_HESAI_DRIVER_H
#define NEBULA_HESAI_DRIVER_H

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

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
  std::shared_ptr<HesaiScanDecoder> scan_decoder_;

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
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ConvertScanToPointcloud(
    const std::shared_ptr<pandar_msgs::msg::PandarScan> & pandar_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_DRIVER_H
