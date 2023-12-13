// Copyright 2023 Tier IV, Inc.
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

#include "nebula_common/continental/continental_common.hpp"
#include "nebula_decoders/nebula_decoders_continental/decoders/continental_packets_decoder.hpp"

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"
#include <continental_msgs/msg/continental_ars548_detection_list.hpp>
#include <continental_msgs/msg/continental_ars548_object_list.hpp>

#include <array>
#include <memory>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
/// @brief Continental Radar decoder (ARS548)
class ContinentalARS548Decoder : public ContinentalPacketsDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  explicit ContinentalARS548Decoder(
    const std::shared_ptr<drivers::ContinentalARS548SensorConfiguration> & sensor_configuration);

  /// @brief Function for parsing NebulaPackets
  /// @param nebula_packets
  /// @return Resulting flag
  bool ProcessPackets(const nebula_msgs::msg::NebulaPackets & nebula_packets) override;

  bool ParseDetectionsListPacket(const std::vector<uint8_t> & data);
  bool ParseObjectsListPacket(const std::vector<uint8_t> & data);

  /// @brief Register function to call whenever a new detection list is processed
  /// @param detection_list_callback
  /// @return Resulting status
  Status RegisterDetectionListCallback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList>)>
      detection_list_callback);

  /// @brief Register function to call whenever a new object list is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status RegisterObjectListCallback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList>)>
      object_list_callback);

private:
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)>
    detection_list_callback_;
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)>
    object_list_callback_;
};

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
