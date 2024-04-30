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
#include <memory>

namespace nebula
{
namespace drivers
{
/// @brief Hesai driver
class HesaiDriver
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<HesaiScanDecoder> scan_decoder_;

  template <typename SensorT>
  std::shared_ptr<HesaiScanDecoder> InitializeDecoder(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration);

public:
  HesaiDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver (either
  /// HesaiCalibrationConfiguration for sensors other than AT128 or HesaiCorrection for AT128)
  explicit HesaiDriver(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const HesaiCalibrationConfigurationBase & calibration_configuration);

  /// @brief Convert PandarScan message to point cloud
  /// @param pandar_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ParseCloudPacket(
    const std::vector<uint8_t> & packet);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_DRIVER_H
