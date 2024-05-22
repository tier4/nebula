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

#ifndef NEBULA_ContinentalSrr520DriverRosWrapper_H
#define NEBULA_ContinentalSrr520DriverRosWrapper_H

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_decoders/nebula_decoders_continental/decoders/continental_srr520_decoder.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp>
#include <nebula_ros/common/nebula_driver_ros_wrapper_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <continental_msgs/msg/continental_srr520_detection.hpp>
#include <continental_msgs/msg/continental_srr520_detection_list.hpp>
#include <continental_msgs/msg/continental_srr520_object.hpp>
#include <continental_msgs/msg/continental_srr520_object_list.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <unordered_set>

namespace nebula
{
namespace ros
{
/// @brief Ros wrapper of continental radar ethernet driver
class ContinentalSrr520DriverRosWrapper final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::continental_srr520::ContinentalSrr520Decoder> decoder_ptr_;
  Status wrapper_status_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;

  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520DetectionList>::SharedPtr
    near_detection_list_pub_;
  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520DetectionList>::SharedPtr
    hrr_detection_list_pub_;
  rclcpp::Publisher<continental_msgs::msg::ContinentalSrr520ObjectList>::SharedPtr object_list_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_detection_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hrr_detection_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_pub_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr near_scan_raw_pub_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr hrr_scan_raw_pub_;
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr objects_raw_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_markers_pub_;

  std::unordered_set<int> previous_ids_;

  std::shared_ptr<drivers::continental_srr520::ContinentalSrr520SensorConfiguration>
    sensor_cfg_ptr_;

  drivers::continental_srr520::ContinentalSrr520HwInterface hw_interface_;

  /// @brief Initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  Status InitializeDriver(std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration);

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @param correction_configuration Output of CorrectionConfiguration (for AT)
  /// @return Resulting status
  Status GetParameters(
    drivers::continental_srr520::ContinentalSrr520SensorConfiguration & sensor_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /// @brief Callback to process new Near ContinentalSrr520DetectionList from the driver
  /// @param msg The new ContinentalSrr520DetectionList from the driver
  void NearDetectionListCallback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  /// @brief Callback to process new HRR ContinentalSrr520DetectionList from the driver
  /// @param msg The new ContinentalSrr520DetectionList from the driver
  void HRRDetectionListCallback(
    std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg);

  /// @brief Callback to process new ContinentalSrr520ObjectList from the driver
  /// @param msg The new ContinentalSrr520ObjectList from the driver
  void ObjectListCallback(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg);

  /// @brief Callback to process new DiagnosticArray from the driver
  /// @param msg The new DiagnosticArray from the driver
  void StatusCallback(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> msg);

public:
  explicit ContinentalSrr520DriverRosWrapper(const rclcpp::NodeOptions & options);

  /// @brief Callback for NebulaPackets subscriber
  /// @param scan_msg Received NebulaPackets
  void ReceivePacketsMsgCallback(const nebula_msgs::msg::NebulaPackets::SharedPtr scan_msg);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Convert SRR520 detections to a pointcloud
  /// @param msg The SRR520 detection list msg
  /// @return Resulting detection pointcloud
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSrr520Detection>::Ptr
  ConvertToPointcloud(const continental_msgs::msg::ContinentalSrr520DetectionList & msg);

  /// @brief Convert SRR520 objects to a pointcloud
  /// @param msg The SRR520 object list msg
  /// @return Resulting object pointcloud
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSrr520Object>::Ptr ConvertToPointcloud(
    const continental_msgs::msg::ContinentalSrr520ObjectList & msg);

  /// @brief Convert SRR520 detections to a standard RadarScan msg
  /// @param msg The SRR520 detection list msg
  /// @return Resulting RadarScan msg
  radar_msgs::msg::RadarScan ConvertToRadarScan(
    const continental_msgs::msg::ContinentalSrr520DetectionList & msg);

  /// @brief Convert SRR520 objects to a standard RadarTracks msg
  /// @param msg The SRR520 object list msg
  /// @return Resulting RadarTracks msg
  radar_msgs::msg::RadarTracks ConvertToRadarTracks(
    const continental_msgs::msg::ContinentalSrr520ObjectList & msg);

  /// @brief Convert SRR520 objects to a standard MarkerArray msg
  /// @param msg The SRR520 object list msg
  /// @return Resulting MarkerArray msg
  visualization_msgs::msg::MarkerArray ConvertToMarkers(
    const continental_msgs::msg::ContinentalSrr520ObjectList & msg);
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_ContinentalSrr520DriverRosWrapper_H
