#ifndef NEBULA_VELODYNE_DRIVER_H
#define NEBULA_VELODYNE_DRIVER_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace nebula
{
namespace drivers
{
/// @brief Velodyne driver
class VelodyneDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<drivers::VelodyneScanDecoder> scan_decoder_;

public:
  VelodyneDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  VelodyneDriver(
    const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration);

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Convert VelodyneScan message to point cloud
  /// @param velodyne_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ConvertScanToPointcloud(
    const std::shared_ptr<velodyne_msgs::msg::VelodyneScan> & velodyne_scan);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_DRIVER_H
