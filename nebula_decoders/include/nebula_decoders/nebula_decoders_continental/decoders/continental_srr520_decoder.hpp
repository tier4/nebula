// Copyright 2024 Tier IV, Inc.
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

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_decoders/nebula_decoders_continental/decoders/continental_packets_decoder.hpp>

#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_object_list.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <std_msgs/msg/header.hpp>

#include <array>
#include <memory>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_srr520
{
/// @brief Continental Radar decoder (SRR520)
class ContinentalSRR520Decoder : public ContinentalPacketsDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  explicit ContinentalSRR520Decoder(
    const std::shared_ptr<ContinentalSRR520SensorConfiguration> & sensor_configuration);

  /// @brief Function for psrring NebulaPackets
  /// @param nebula_packets
  /// @return Resulting flag
  bool ProcessPackets(const nebula_msgs::msg::NebulaPackets & nebula_packets) override;

  bool ParseDetectionsListPacket(const nebula_msgs::msg::NebulaPackets & nebula_packets, bool near);
  bool ParseObjectsListPacket(const nebula_msgs::msg::NebulaPackets & nebula_packets);
  bool ParseStatusPacket(const nebula_msgs::msg::NebulaPackets & nebula_packets);

  /// @brief Register function to call whenever a new rdi near detection list is processed
  /// @param detection_list_callback
  /// @return Resulting status
  Status RegisterNearDetectionListCallback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
      detection_list_callback);

  /// @brief Register function to call whenever a new rdi hrr detection list is processed
  /// @param detection_list_callback
  /// @return Resulting status
  Status RegisterHRRDetectionListCallback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
      detection_list_callback);

  /// @brief Register function to call whenever a new object list is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status RegisterObjectListCallback(
    std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList>)>
      object_list_callback);

  /// @brief Register function to call whenever a new object list is processed
  /// @param object_list_callback
  /// @return Resulting status
  Status RegisterStatusCallback(
    std::function<void(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray>)> status_callback);

private:
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)>
    near_detection_list_callback_;
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)>
    hrr_detection_list_callback_;
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg)>
    object_list_callback_;
  std::function<void(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> msg)> status_callback_;

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<continental_srr520::ContinentalSRR520SensorConfiguration> sensor_configuration_;
};

}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula
