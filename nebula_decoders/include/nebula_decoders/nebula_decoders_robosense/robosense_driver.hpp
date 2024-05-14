#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_decoder.hpp"

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace nebula
{
namespace drivers
{
/// @brief Robosense driver
class RobosenseDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;

  /// @brief Decoder according to the model
  std::shared_ptr<RobosenseScanDecoder> scan_decoder_;

public:
  RobosenseDriver() = delete;

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  explicit RobosenseDriver(
    const std::shared_ptr<const drivers::RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::RobosenseCalibrationConfiguration> & calibration_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Convert RobosenseScan message to point cloud
  /// @param robosense_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ParseCloudPacket(const std::vector<uint8_t> & packet);
};

}  // namespace drivers
}  // namespace nebula
