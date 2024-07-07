#ifndef NEBULA_SEYOND_DRIVER_H
#define NEBULA_SEYOND_DRIVER_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/seyond/seyond_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_decoder.hpp"

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace nebula
{
namespace drivers
{
/// @brief Seyond driver
class SeyondDriver
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<SeyondScanDecoder> scan_decoder_;

  // template <typename SensorT>
  std::shared_ptr<SeyondScanDecoder> InitializeDecoder(
    const std::shared_ptr<const drivers::SeyondSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::SeyondCalibrationConfiguration> &
      calibration_configuration);

public:
  SeyondDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  explicit SeyondDriver(
    const std::shared_ptr<const drivers::SeyondSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::SeyondCalibrationConfiguration> &
      calibration_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status SetCalibrationConfiguration(
    const SeyondCalibrationConfiguration & calibration_configuration);

  /// @brief Convert NebulaPackets message to point cloud
  /// @param nebula_packets Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> ParseCloudPacket(
    const std::vector<uint8_t> & packet);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_SEYOND_DRIVER_H
