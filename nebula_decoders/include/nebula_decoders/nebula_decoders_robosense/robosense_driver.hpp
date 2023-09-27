#ifndef NEBULA_ROBOSENSE_DRIVER_H
#define NEBULA_ROBOSENSE_DRIVER_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/decoder.hpp"

#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"
#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <functional>
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
  std::shared_ptr<drivers::Decoder> scan_decoder_;

public:
  RobosenseDriver() = delete;

  // /// @brief Constructor
  // /// @param type SensorType  for this driver
  // /// @param param DecoderParam for this driver
  // RobosenseDriver(
  //   const SensorModel type, const RSDecoderParam& param);

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver

  explicit RobosenseDriver(
    const std::shared_ptr<drivers::RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::RobosenseCalibrationConfiguration> & calibration_configuration);

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting statuso
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Convert RobosenseScan message to point cloud
  /// @param robosense_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ConvertScanToPointcloud(
    const std::shared_ptr<robosense_msgs::msg::RobosenseScan> & robosense_scan);

  std::function<void(const AllDeviceInfo &)> difop_callback_;
  void registDifopCallback(const std::function<void(const AllDeviceInfo &)> & callback);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_ROBOSENSE_DRIVER_H
