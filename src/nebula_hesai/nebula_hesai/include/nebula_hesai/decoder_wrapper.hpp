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

#include "nebula_core_ros/agnocast_wrapper/diagnostic_updater.hpp"
#include "nebula_core_ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp"
#include "nebula_core_ros/agnocast_wrapper/node.hpp"
#include "nebula_core_ros/diagnostics/render.hpp"
#include "nebula_core_ros/rclcpp_clock.hpp"
#include "nebula_core_ros/single_consumer_processor.hpp"
#include "nebula_hesai/diagnostics/render.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/hesai_decode_pipeline.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

namespace nebula::ros
{
class HesaiDecoderWrapper
{
public:
  HesaiDecoderWrapper(
    nebula::agnocast_wrapper::Node * parent_node,
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
    nebula::agnocast_wrapper::diagnostic_updater::Updater & diagnostic_updater,
    bool publish_packets);

  /// @brief Process a cloud packet and return metadata
  /// @param packet_msg The packet to process
  /// @param receive_metadata Performance metadata from packet reception
  /// @return Expected containing metadata on success, or decode error on failure
  drivers::PacketDecodeResult process_cloud_packet(
    std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg, uint64_t receive_time_ns);

  void on_pointcloud_decoded(const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config);

  void on_calibration_change(
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> &
      new_calibration);

  /// @brief Reset the decoder to its initial state, discarding all state related to the current
  /// scan.
  void reset_decoder();

  void publish_and_reset_performance_counters(const rclcpp::Time & scan_timestamp);

  nebula::Status status();

private:
  void publish_cloud(
    NEBULA_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) && pointcloud,
    const NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) & publisher);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /// @brief Read the settings of the decoder's diagnostics from the node's parameters
  static drivers::HesaiDecoderDiagnosticsConfiguration declare_diagnostics_configuration(
    drivers::SensorModel sensor_model, nebula::agnocast_wrapper::Node & node);

  void on_blockage_mask(
    const drivers::point_filters::BlockageMask & blockage_mask, double timestamp_s);

  nebula::Status status_;
  rclcpp::Logger logger_;
  nebula::agnocast_wrapper::Node & parent_node_;

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_;

  pandar_msgs::msg::PandarScan::UniquePtr current_scan_msg_;
  NEBULA_PUBLISHER_PTR(pandar_msgs::msg::PandarScan) packets_pub_;
  std::optional<SingleConsumerProcessor<pandar_msgs::msg::PandarScan::UniquePtr>>
    packets_pub_thread_;

  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) nebula_points_pub_;
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) aw_points_ex_pub_;
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) aw_points_base_pub_;

  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::Image) blockage_mask_pub_;

  autoware_utils_debug::BasicDebugPublisher<nebula::agnocast_wrapper::Node> debug_publisher_;

  struct PerformanceCounters
  {
    uint64_t decode_time_current_scan_ns{0};
    uint64_t receive_time_current_scan_ns{0};
    uint64_t publish_time_current_scan_ns{0};
  } current_scan_perf_counters_;

  std::mutex mtx_pipeline_;
  /// Declared last, since its callbacks use the members above
  std::optional<drivers::HesaiDecodePipeline> pipeline_;
};
}  // namespace nebula::ros
