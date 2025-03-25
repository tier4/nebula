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

#include <nebula_common/robosense/robosense_common.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>
namespace nebula::drivers
{

class RobosenseInfoDecoderBase
{
public:
  RobosenseInfoDecoderBase(RobosenseInfoDecoderBase && c) = delete;
  RobosenseInfoDecoderBase & operator=(RobosenseInfoDecoderBase && c) = delete;
  RobosenseInfoDecoderBase(const RobosenseInfoDecoderBase & c) = delete;
  RobosenseInfoDecoderBase & operator=(const RobosenseInfoDecoderBase & c) = delete;

  virtual ~RobosenseInfoDecoderBase() = default;
  RobosenseInfoDecoderBase() = default;

  /// @brief Parses DIFOP and add its telemetry
  /// @param raw_packet The incoming DIFOP packet
  /// @return Whether the packet was parsed successfully
  virtual bool parse_packet(const std::vector<uint8_t> & raw_packet) = 0;

  /// @brief Get the sensor telemetry
  /// @return The sensor telemetry
  virtual std::map<std::string, std::string> get_sensor_info() = 0;

  /// @brief Get the laser return mode
  /// @return The laser return mode
  virtual ReturnMode get_return_mode() = 0;

  /// @brief Get sensor calibration
  /// @return The sensor calibration
  virtual RobosenseCalibrationConfiguration get_sensor_calibration() = 0;

  /// @brief Get the status of time synchronization
  /// @return True if the sensor's clock is synchronized
  virtual bool get_sync_status() = 0;
};

}  // namespace nebula::drivers
