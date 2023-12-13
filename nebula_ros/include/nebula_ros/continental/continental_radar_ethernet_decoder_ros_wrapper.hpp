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

#ifndef NEBULA_ContinentalRadarEthernetDriverRosWrapper_H
#define NEBULA_ContinentalRadarEthernetDriverRosWrapper_H

#include "nebula_common/continental/continental_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_decoders/nebula_decoders_continental/decoders/continental_ars548_decoder.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_radar_ethernet_hw_interface.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "continental_msgs/msg/continental_ars548_detection.hpp"
#include "continental_msgs/msg/continental_ars548_detection_list.hpp"
#include "continental_msgs/msg/continental_ars548_object.hpp"
#include "continental_msgs/msg/continental_ars548_object_list.hpp"
#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

#include <chrono>
#include <memory>

namespace nebula
{
namespace ros
{
/// @brief Ros wrapper of continental radar ethernet driver
class ContinentalRadarEthernetDriverRosWrapper final : public rclcpp::Node,
                                                       NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::continental_ars548::ContinentalARS548Decoder> decoder_ptr_;
  Status wrapper_status_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;

  rclcpp::Publisher<continental_msgs::msg::ContinentalArs548DetectionList>::SharedPtr
    detection_list_pub_;
  rclcpp::Publisher<continental_msgs::msg::ContinentalArs548ObjectList>::SharedPtr object_list_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_pointcloud_pub_;

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  drivers::ContinentalRadarEthernetHwInterface hw_interface_;

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
  Status GetParameters(drivers::ContinentalRadarEthernetSensorConfiguration & sensor_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /***
   * Publishes a sensor_msgs::msg::PointCloud2 to the specified publisher
   * @param pointcloud unique pointer containing the point cloud to publish
   * @param publisher
   */
  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  void detectionListCallback(
    std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg);
  void objectListCallback(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg);

public:
  explicit ContinentalRadarEthernetDriverRosWrapper(const rclcpp::NodeOptions & options);

  /// @brief Callback for NebulaPackets subscriber
  /// @param scan_msg Received NebulaPackets
  void ReceivePacketsMsgCallback(const nebula_msgs::msg::NebulaPackets::SharedPtr scan_msg);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_ContinentalRadarEthernetDriverRosWrapper_H
