// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class RobosenseInfoDecoder : public RobosenseInfoDecoderBase
{
protected:
  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief The last decoded packet
  typename SensorT::info_t packet_{};

  rclcpp::Logger logger_;

public:
  /// @brief Validates and parses DIFOP packet. Currently only checks size, not checksums etc.
  /// @param raw_packet The incoming DIFOP packet
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & raw_packet) override
  {
    const auto packet_size = raw_packet.size();
    if (packet_size < sizeof(typename SensorT::info_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch: " << packet_size << " | Expected at least: "
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
  RobosenseInfoDecoder() : logger_(rclcpp::get_logger("RobosenseInfoDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
  }

  /// @brief Get the sensor telemetry
  /// @return The sensor telemetry
  std::map<std::string, std::string> get_sensor_info() override
  {
    return sensor_.get_sensor_info(packet_);
  }

  /// @brief Get the laser return mode
  /// @return The laser return mode
  ReturnMode get_return_mode() override { return sensor_.get_return_mode(packet_); }

  /// @brief Get sensor calibration
  /// @return The sensor calibration
  RobosenseCalibrationConfiguration get_sensor_calibration() override
  {
    return sensor_.get_sensor_calibration(packet_);
  }

  /// @brief Get the status of time synchronization
  /// @return True if the sensor's clock is synchronized
  bool get_sync_status() override { return sensor_.get_sync_status(packet_); }
};

}  // namespace nebula::drivers
