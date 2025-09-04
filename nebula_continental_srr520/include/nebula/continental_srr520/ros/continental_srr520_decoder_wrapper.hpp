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

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/util/expected.hpp>
#include <nebula_decoders/nebula_decoders_continental/decoders/continental_srr520_decoder.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <continental_msgs/msg/continental_srr520_detection.hpp>
#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_object.hpp>
#include <continental_msgs/msg/continental_srr520_object_list.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace nebula::ros
{
class ContinentalSRR520DecoderWrapper
{
public:
  ContinentalSRR520DecoderWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
      config,
    std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface_ptr);

  void process_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<
      const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
      new_config_ptr);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status status();

  /// @brief Callback to process a new Near ContinentalSrr520DetectionList from the driver
  /// @param msg The new ContinentalSrr520DetectionList from the driver
  void near_detection_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  /// @brief Callback to process a new HRR ContinentalSrr520DetectionList from the driver
  /// @param msg The new ContinentalSrr520DetectionList from the driver
  void hrr_detection_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  /// @brief Callback to process a new ContinentalSrr520ObjectList from the driver
  /// @param msg The new ContinentalSrr520ObjectList from the driver
  void object_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg);

  /// @brief Callback to process a new DiagnosticArray from the driver
  /// @param msg The new DiagnosticArray from the driver
  void status_callback(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> msg);

  /// @brief Callback to process a new SyncFollowUp message from the driver
  void sync_follow_up_callback(builtin_interfaces::msg::Time stamp);

  /// @brief Callback to process a new NebulaPackets message from the driver
  /// @param msg The new NebulaPackets from the driver
  void packets_callback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg);

private:
  nebula::Status initialize_driver(
    const std::shared_ptr<
      const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> & config);

  /// @brief Convert SRR520 detections to a pointcloud
  /// @param msg The SRR520 detection list msg
  /// @return Resulting detection pointcloud
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Detection>::Ptr
  convert_to_pointcloud(const continental_msgs::msg::ContinentalSrr520DetectionList & msg);

  /// @brief Convert SRR520 objects to a pointcloud
  /// @param msg The SRR520 object list msg
  /// @return Resulting object pointcloud
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Object>::Ptr
  convert_to_pointcloud(const continental_msgs::msg::ContinentalSrr520ObjectList & msg);

  /// @brief Convert SRR520 detections to a standard RadarScan msg
  /// @param msg The SRR520 detection list msg
  /// @return Resulting RadarScan msg
  radar_msgs::msg::RadarScan convert_to_radar_scan(
    const continental_msgs::msg::ContinentalSrr520DetectionList & msg);

  /// @brief Convert SRR520 objects to a standard RadarTracks msg
  /// @param msg The SRR520 object list msg
  /// @return Resulting RadarTracks msg
  radar_msgs::msg::RadarTracks convert_to_radar_tracks(
    const continental_msgs::msg::ContinentalSrr520ObjectList & msg);

  /// @brief Convert SRR520 objects to a standard MarkerArray msg
  /// @param msg The SRR520 object list msg
  /// @return Resulting MarkerArray msg
  visualization_msgs::msg::MarkerArray convert_to_markers(
    const continental_msgs::msg::ContinentalSrr520ObjectList & msg);

  const rclcpp::Node * const parent_node_;
  nebula::Status status_;
  rclcpp::Logger logger_;

  std::shared_ptr<const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    sensor_cfg_{};

  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520Decoder> driver_ptr_{};
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface_ptr_{};
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_{};

  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520DetectionList>::SharedPtr
    near_detection_list_pub_{};
  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520DetectionList>::SharedPtr
    hrr_detection_list_pub_{};
  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520ObjectList>::SharedPtr
    object_list_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_detection_pointcloud_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hrr_detection_pointcloud_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_pub_{};
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr near_scan_raw_pub_{};
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr hrr_scan_raw_pub_{};
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr objects_raw_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_markers_pub_{};

  std::unordered_set<int> previous_ids_{};

  std::shared_ptr<WatchdogTimer> watchdog_;
};
}  // namespace nebula::ros
