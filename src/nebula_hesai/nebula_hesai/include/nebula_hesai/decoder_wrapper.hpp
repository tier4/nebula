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

#include "nebula_core_ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp"
#include "nebula_core_ros/diagnostics/rate_bound_status.hpp"
#include "nebula_core_ros/single_consumer_processor.hpp"
#include "nebula_hesai/diagnostics/functional_safety_diagnostic_task.hpp"
#include "nebula_hesai/diagnostics/packet_loss_diagnostic.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/hesai_driver.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_hesai_common/hesai/hesai_common.hpp>
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
    rclcpp::Node * parent_node,
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
    diagnostic_updater::Updater & diagnostic_updater, bool publish_packets);

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

  static custom_diagnostic_tasks::RateBoundStatus make_rate_bound_status(
    uint16_t rpm, rclcpp::Node & node)
  {
    double nominal_rate_hz = drivers::rpm2hz(rpm);

    double min_ok_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_ok.min_hz");
    double max_ok_hz = node.declare_parameter<double>(
      "diagnostics.pointcloud_publish_rate.frequency_ok.max_hz",
      std::numeric_limits<double>::infinity());
    double min_warn_hz =
      node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_warn.min_hz");
    double max_warn_hz = node.declare_parameter<double>(
      "diagnostics.pointcloud_publish_rate.frequency_warn.max_hz",
      std::numeric_limits<double>::infinity());

    size_t num_frame_transition =
      node.declare_parameter<int>("diagnostics.pointcloud_publish_rate.num_frame_transition", 1);

    // Warn if misconfigured. Since this is not a critical error, continue operation.
    if (nominal_rate_hz < min_ok_hz || nominal_rate_hz > max_ok_hz) {
      RCLCPP_WARN(
        node.get_logger(),
        "The configured sensor framerate (%d RPM = %.2f Hz) is outside the configured framerate "
        "bounds: %.2f - %.2f. Please check the sensor configuration.",
        rpm, nominal_rate_hz, min_ok_hz, max_ok_hz);
    }

    custom_diagnostic_tasks::RateBoundStatusParam ok_params(min_ok_hz, max_ok_hz);
    custom_diagnostic_tasks::RateBoundStatusParam warn_params(min_warn_hz, max_warn_hz);
    return {&node, ok_params, warn_params, num_frame_transition};
  }

  void initialize_functional_safety(
    diagnostic_updater::Updater & diagnostic_updater,
    const std::optional<drivers::AdvancedFunctionalSafetyConfiguration> & fs_config);

  void initialize_packet_loss_diagnostic(diagnostic_updater::Updater & diagnostic_updater);

  std::pair<
    std::shared_ptr<drivers::point_filters::BlockageMaskPlugin>,
    NEBULA_PUBLISHER_PTR(sensor_msgs::msg::Image)>
  initialize_blockage_mask_plugin();

  std::shared_ptr<drivers::HesaiDriver> initialize_driver(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & config,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> & calibration);

  nebula::Status status_;
  rclcpp::Logger logger_;
  rclcpp::Node & parent_node_;

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> calibration_cfg_ptr_;

  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;
  std::mutex mtx_driver_ptr_;

  pandar_msgs::msg::PandarScan::UniquePtr current_scan_msg_;
  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr packets_pub_;
  std::optional<SingleConsumerProcessor<pandar_msgs::msg::PandarScan::UniquePtr>>
    packets_pub_thread_;

  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) nebula_points_pub_;
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) aw_points_ex_pub_;
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) aw_points_base_pub_;

  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::Image) blockage_mask_pub_;

  custom_diagnostic_tasks::RateBoundStatus publish_diagnostic_;
  std::optional<FunctionalSafetyDiagnosticTask> functional_safety_diagnostic_;
  std::optional<PacketLossDiagnosticTask> packet_loss_diagnostic_;

  autoware_utils_debug::DebugPublisher debug_publisher_;

  struct PerformanceCounters
  {
    uint64_t decode_time_current_scan_ns{0};
    uint64_t receive_time_current_scan_ns{0};
    uint64_t publish_time_current_scan_ns{0};
  } current_scan_perf_counters_;
};
}  // namespace nebula::ros
