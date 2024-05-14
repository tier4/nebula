#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v3.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/bpearl_v4.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/helios.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"

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
/// @brief Robosense driver
class RobosenseInfoDriver
{
private:
  /// @brief Current driver status
  Status driver_status_;

  /// @brief Decoder according to the model
  std::shared_ptr<RobosenseInfoDecoderBase> info_decoder_;

public:
  RobosenseInfoDriver() = delete;

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  explicit RobosenseInfoDriver(
    const std::shared_ptr<const drivers::RobosenseSensorConfiguration> & sensor_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  Status DecodeInfoPacket(const std::vector<uint8_t> & packet);

  std::map<std::string, std::string> GetSensorInfo();

  ReturnMode GetReturnMode();

  RobosenseCalibrationConfiguration GetSensorCalibration();

  /// @brief Get the status of time synchronization
  /// @return True if the sensor's clock is synchronized
  bool GetSyncStatus();
};

}  // namespace drivers
}  // namespace nebula
