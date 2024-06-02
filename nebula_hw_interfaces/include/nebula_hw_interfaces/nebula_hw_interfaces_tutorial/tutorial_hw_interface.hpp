#ifndef NEBULA_TUTORIAL_HW_INTERFACE_H
#define NEBULA_TUTORIAL_HW_INTERFACE_H

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/ptc.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/udp_receiver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/tutorial/tutorial_common.hpp>
#include <nebula_common/util/expected.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

using TutorialConfig = typename nebula::HesaiConfig;
constexpr auto returnModeFromInt = nebula::drivers::ReturnModeFromIntHesai;
constexpr auto intFromReturnMode = nebula::drivers::IntFromReturnModeHesai;

const int TCP_PORT = 9347;
const uint8_t PTC_COMMAND_GET_CONFIG_INFO = 0x08;
const uint8_t PTC_COMMAND_SET_SPIN_RATE = 0x17;
const uint8_t PTC_COMMAND_SET_SYNC_ANGLE = 0x18;
const uint8_t PTC_COMMAND_SET_RETURN_MODE = 0x1e;

class TutorialHwInterface
{
private:
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;
  connections::PtcConnection ptc_connection_;
  connections::UdpReceiver udp_receiver_;
  connections::UdpReceiver::callback_t cloud_packet_callback_ = nullptr;

public:
  TutorialHwInterface(
    std::shared_ptr<loggers::Logger> logger,
    const std::shared_ptr<const TutorialSensorConfiguration> & sensor_configuration);

  /// @brief Retrieve the configuration from the sensor synchronously
  /// @return The retrieved config, or throw if an error occurs
  TutorialConfig getConfig();

  /// @brief Get current config from sensor, compare with given config, and send config updates to
  /// sensor where needed
  /// @param updated The updated configuration that should be sent to the sensor
  /// @return Resulting status
  Status compareAndSendConfig(const TutorialSensorConfiguration & updated);

  /// @brief Set the RPM of the sensor
  /// @param rpm Spin rate
  /// @return Resulting status
  Status setSpinRate(uint16_t rpm);

  /// @brief Set the angle in degrees where the sensor syncs to 1.000...s
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @return Resulting status
  Status setSyncAngle(int sync_angle, int angle);

  /// @brief Set the return mode (last, strongest, lastStrongest etc.) of the sensor
  /// @param return_mode Return mode
  /// @return Resulting status
  Status setReturnMode(int return_mode);

  /// @brief Register a callback that gets called for each received UDP packet
  Status registerOnSensorPacketCallback(connections::UdpReceiver::callback_t scan_callback)
  {
    cloud_packet_callback_ = std::move(scan_callback);
    return Status::OK;
  }

private:
  void onSensorPacket(std::vector<uint8_t> & buffer)
  {
    if (!cloud_packet_callback_) {
      return;
    }

    cloud_packet_callback_(buffer);
  }
};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_TUTORIAL_HW_INTERFACE_H
