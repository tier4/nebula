#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebula
{
namespace drivers
{

template <typename SensorT>
class RobosenseInfoDecoder : public RobosenseInfoDecoderBase
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::RobosenseSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  typename SensorT::info_t packet_{};

  rclcpp::Logger logger_;

public:
  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param pandar_packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const std::vector<uint8_t> & raw_packet) override
  {
    const auto packet_size = raw_packet.size();
    if (packet_size < sizeof(typename SensorT::info_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << packet_size << " | Expected at least:"
                                         << sizeof(typename SensorT::info_t));
      return false;
    }
    try {
      if (std::memcpy(&packet_, raw_packet.data(), sizeof(typename SensorT::info_t)) == &packet_) {
        return true;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Packet memcopy failed: " << e.what());
    }

    return false;
  }

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  explicit RobosenseInfoDecoder(
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration)
  : sensor_configuration_(sensor_configuration), logger_(rclcpp::get_logger("RobosenseInfoDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, sensor_configuration_);
  }

  std::map<std::string, std::string> getSensorInfo() override
  {
    return sensor_.getSensorInfo(packet_);
  }
};

}  // namespace drivers
}  // namespace nebula