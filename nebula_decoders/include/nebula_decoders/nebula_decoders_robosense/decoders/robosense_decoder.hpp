#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_scan_decoder.hpp"
#include "nebula_common/robosense/robosense_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
template <typename SensorT>
class RobosenseDecoder : public RobosenseScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::RobosenseSensorConfiguration> sensor_configuration_;
};

}  // namespace drivers
}  // namespace nebula