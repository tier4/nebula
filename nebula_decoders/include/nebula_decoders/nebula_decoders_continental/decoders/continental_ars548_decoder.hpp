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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_packets_decoder.hpp"

#include <nebula_common/continental/continental_ars548.hpp>

#include <continental_msgs/msg/continental_ars548_detection_list.hpp>
#include <continental_msgs/msg/continental_ars548_object_list.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <array>
#include <memory>
#include <vector>

namespace nebula::drivers::continental_ars548
{
/// @brief Continental Radar decoder (ARS548)
class ContinentalARS548Decoder : public ContinentalPacketsDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  explicit ContinentalARS548Decoder(
    const std::shared_ptr<const ContinentalARS548SensorConfiguration> & sensor_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status() override;

  /// @brief Function for parsing NebulaPackets
  /// @param nebula_packets
  /// @return Resulting flag
  bool process_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg) override;

  /// @brief Register function to call when a new detection list is processed
  /// @param detection_list_callback
  /// @return Resulting status
  Status register_detection_list_callback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList>)>
      detection_list_callback);

  /// @brief Register function to call when a new object list is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status register_object_list_callback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList>)>
      object_list_callback);

  /// @brief Register function to call when a new sensor status message is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status register_sensor_status_callback(
    std::function<void(const ContinentalARS548Status & status)> sensor_status_callback);

  /// @brief Register function to call when a new sensor status message is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status register_packets_callback(
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> packets_callback);

private:
  /// @brief Function for parsing detection lists
  /// @param data
  /// @return Resulting flag
  bool parse_detections_list_packet(const nebula_msgs::msg::NebulaPacket & packet_msg);

  /// @brief Function for parsing object lists
  /// @param data
  /// @return Resulting flag
  bool parse_objects_list_packet(const nebula_msgs::msg::NebulaPacket & packet_msg);

  /// @brief Function for parsing sensor status messages
  /// @param data
  /// @return Resulting flag
  bool parse_sensor_status_packet(const nebula_msgs::msg::NebulaPacket & packet_msg);

  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)>
    detection_list_callback_{};
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)>
    object_list_callback_{};
  std::function<void(const ContinentalARS548Status & status)> sensor_status_callback_{};
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg)>
    nebula_packets_callback_{};

  ContinentalARS548Status radar_status_{};

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<const continental_ars548::ContinentalARS548SensorConfiguration> config_ptr_{};
};

}  // namespace nebula::drivers::continental_ars548
