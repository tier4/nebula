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

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers
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
  Status get_status();

  Status decode_info_packet(const std::vector<uint8_t> & packet);

  std::map<std::string, std::string> get_sensor_info();

  ReturnMode get_return_mode();

  RobosenseCalibrationConfiguration get_sensor_calibration();

  /// @brief Get the status of time synchronization
  /// @return True if the sensor's clock is synchronized
  bool get_sync_status();
};

}  // namespace nebula::drivers
