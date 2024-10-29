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

#ifndef NEBULA_ContinentalRosDecoderTestsrr520_H
#define NEBULA_ContinentalRosDecoderTestsrr520_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_decoders/nebula_decoders_continental/decoders/continental_srr520_decoder.hpp>
#include <rclcpp/rclcpp.hpp>

#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_object_list.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace nebula::ros
{
class ContinentalRosDecoderTest final : public rclcpp::Node  //, testing::Test
{
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520Decoder> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    sensor_cfg_ptr_;

  Status initialize_driver(
    std::shared_ptr<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
      sensor_configuration);

  Status get_parameters(
    drivers::continental_srr520::ContinentalSRR520SensorConfiguration & sensor_configuration);

  void check_result(const std::string msg_as_string, const std::string & gt_path);

  void hrr_detection_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  void near_detection_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  void status_callback([[maybe_unused]] std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> msg)
  {
  }

  void object_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg);

  void compare_nodes(const YAML::Node & node1, const YAML::Node & node2);

  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit ContinentalRosDecoderTest(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  void receive_scan_msg_callback(const nebula_msgs::msg::NebulaPackets::SharedPtr scan_msg);
  Status get_status();
  void read_bag();

private:
  std::string bag_path_{};
  std::string storage_id_{};
  std::string format_{};
  std::string target_topic_{};
  std::size_t near_detection_list_count_{};
  std::size_t hrr_detection_list_count_{};
  std::size_t object_list_count_{};
};

}  // namespace nebula::ros

#endif  // NEBULA_ContinentalRosDecoderTestsrr520_H
