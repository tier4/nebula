#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

namespace nebula
{
namespace drivers
{

template <typename SensorInfoT>
class RobosenseInfoDecoder : public RobosenseInfoDecoderBase
{
  static_assert(std::is_base_of_v<SensorInfoBase<typename SensorInfoT::packet_t>, SensorInfoT>);

protected:
  SensorInfoT sensor_{};

  /// @brief The last decoded packet
  typename SensorInfoT::packet_t packet_{};

  rclcpp::Logger logger_;

public:
  /// @brief Validates and parses DIFOP packet. Currently only checks size, not checksums etc.
  /// @param raw_packet The incoming DIFOP packet
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const std::vector<uint8_t> & raw_packet) override
  {
    const auto packet_size = raw_packet.size();
    if (packet_size < sizeof(typename SensorInfoT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << packet_size << " | Expected at least:"
                                         << sizeof(typename SensorInfoT::packet_t));
      return false;
    }
    try {
      if (std::memcpy(&packet_, raw_packet.data(), sizeof(typename SensorInfoT::packet_t)) == &packet_) {
        return true;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Packet memcopy failed: " << e.what());
    }

    return false;
  }

  RobosenseInfoDecoder() : logger_(rclcpp::get_logger("RobosenseInfoDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

  /// @brief Get the sensor telemetry
  /// @return The sensor telemetry
  std::map<std::string, std::string> getSensorInfo() override
  {
    return sensor_.getSensorInfo(packet_);
  }

  /// @brief Get the laser return mode
  /// @return The laser return mode
  ReturnMode getReturnMode() override { return sensor_.getReturnMode(packet_); }

  /// @brief Get sensor calibration
  /// @return The sensor calibration
  std::optional<RobosenseCalibrationConfiguration> getSensorCalibration() override
  {
    return sensor_.getSensorCalibration(packet_);
  }

  /// @brief Get the status of time synchronization
  /// @return True if the sensor's clock is synchronized
  bool getSyncStatus() override { return sensor_.getSyncStatus(packet_); }
};

}  // namespace drivers
}  // namespace nebula