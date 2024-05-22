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
#include <rclcpp/rclcpp.hpp>

#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_object_list.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_srr520
{
/// @brief Continental Radar decoder (SRR520)
class ContinentalSrr520Decoder : public ContinentalPacketsDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  explicit ContinentalSrr520Decoder(
    const std::shared_ptr<const ContinentalSrr520SensorConfiguration> & sensor_configuration);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus() override;

  /// @brief Function for parsing NebulaPackets
  /// @param nebula_packets
  /// @return Resulting flag
  bool ProcessPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg) override;

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

  /// @brief Register function to call whenever a sync fup packet is processed
  /// @param sync_fup_callback
  /// @return Resulting status
  Status RegisterSyncFupCallback(
    std::function<void(builtin_interfaces::msg::Time)> sync_fup_callback);

  /// @brief Register function to call whenever enough packets have been processed
  /// @param object_list_callback
  /// @return Resulting status
  Status RegisterPacketsCallback(
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> nebula_packets_callback);

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);

private:
  /// @brief Process a new near detection header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearHeaderPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new near element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearElementPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new hrr header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRHeaderPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new hrr element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRElementPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new object header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectHeaderPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new object element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectElementPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessCRCListPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new Near detections crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearCRCListPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new HRR crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRCRCListPacket(
    std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);  // cspell:ignore HRRCRC

  /// @brief Process a new objects crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectCRCListPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new sensor status packet
  /// @param buffer The buffer containing the status packet
  /// @param stamp The stamp in nanoseconds
  void ProcessSensorStatusPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Process a new sensor status packet
  /// @param buffer The buffer containing the status packet
  /// @param stamp The stamp in nanoseconds
  void ProcessSyncFupPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void PrintInfo(std::string info);

  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void PrintError(std::string error);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void PrintDebug(std::string debug);

  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)>
    near_detection_list_callback_{};
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)>
    hrr_detection_list_callback_{};
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg)>
    object_list_callback_{};
  std::function<void(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> msg)>
    status_callback_{};
  std::function<void(builtin_interfaces::msg::Time)> sync_fup_callback_{};
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg)>
    nebula_packets_callback_{};

  std::unique_ptr<nebula_msgs::msg::NebulaPackets> rdi_near_packets_ptr_{};
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> rdi_hrr_packets_ptr_{};
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> object_packets_ptr_{};

  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> near_detection_list_ptr_{};
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> hrr_detection_list_ptr_{};
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> object_list_ptr_{};

  bool first_rdi_near_packet_{true};
  bool first_rdi_hrr_packet_{true};
  bool first_object_packet_{true};

  ScanHeaderPacket rdi_near_header_packet_{};
  ScanHeaderPacket rdi_hrr_header_packet_{};
  ObjectHeaderPacket object_header_packet_{};

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<const continental_srr520::ContinentalSrr520SensorConfiguration>
    sensor_configuration_{};

  std::shared_ptr<rclcpp::Logger> parent_node_logger_ptr_{};
};

}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula
