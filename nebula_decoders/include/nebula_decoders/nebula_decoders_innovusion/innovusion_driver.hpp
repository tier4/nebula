#ifndef NEBULA_Innovusion_DRIVER_H
#define NEBULA_Innovusion_DRIVER_H

#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_innovusion/decoders/innovusion_decoder.hpp"

#include "innovusion_msgs/msg/innovusion_packet.hpp"
#include "innovusion_msgs/msg/innovusion_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace nebula
{
namespace drivers
{
/// @brief Innovusion driver
class InnovusionDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<InnovusionScanDecoder> scan_decoder_;

public:
  InnovusionDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  explicit InnovusionDriver(
    const std::shared_ptr<drivers::InnovusionSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::InnovusionCalibrationConfiguration> & calibration_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Convert InnovusionScan message to point cloud
  /// @param innovusion_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ConvertScanToPointcloud(
    const std::shared_ptr<innovusion_msgs::msg::InnovusionScan> & innovusion_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_Innovusion_DRIVER_H
