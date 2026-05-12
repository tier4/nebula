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

#include "nebula_core_ros/diagnostics/hysteresis_state_machine.hpp"
#include "nebula_core_ros/diagnostics/liveness_monitor.hpp"
#include "nebula_core_ros/diagnostics/rate_bound_status.hpp"
#include "nebula_core_ros/parameter_descriptors.hpp"
#include "nebula_core_ros/sync_tooling/sync_tooling_worker.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_continental_common/continental_ars548.hpp>
#include <nebula_continental_decoders/decoders/continental_ars548_decoder.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/thread_safety_annotations.hpp>

#include <autoware_sensing_msgs/msg/radar_classification.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>
#include <continental_msgs/msg/continental_ars548_detection.hpp>
#include <continental_msgs/msg/continental_ars548_detection_list.hpp>
#include <continental_msgs/msg/continental_ars548_object.hpp>
#include <continental_msgs/msg/continental_ars548_object_list.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace nebula::ros
{
struct SyncToolingPlugin
{
  std::shared_ptr<SyncToolingWorker> worker;
  util::RateLimiter rate_limiter;
};

class ContinentalARS548DecoderWrapper
{
public:
  ContinentalARS548DecoderWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<
      const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> & config,
    bool launch_hw);

  void process_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<
      const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
      new_config);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status status();

  /// @brief Callback to process new ContinentalArs548DetectionList from the driver
  /// @param msg The new ContinentalArs548DetectionList from the driver
  void detection_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg);

  /// @brief Callback to process new ContinentalArs548ObjectList from the driver
  /// @param msg The new ContinentalArs548ObjectList from the driver
  void object_list_callback(
    std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg);

  /// @brief Callback to process new ContinentalARS548Status from the driver
  /// @param msg The new ContinentalArs548ObjectList from the driver
  void sensor_status_callback(
    const drivers::continental_ars548::ContinentalARS548Status & sensor_status);

  /// @brief Callback to process new ContinentalARS548Status from the driver
  /// @param msg The new ContinentalArs548ObjectList from the driver
  void packets_callback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg);

private:
  nebula::Status initialize_driver(
    const std::shared_ptr<
      const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> & config)
    RCPPUTILS_TSA_REQUIRES(mtx_config_ptr_);

  // @brief Create a RadarInfo message for the ARS548 radar
  // @return RadarInfo message
  void create_radar_info();

  // @brief Convert an ARS548 reference point to the center of an object
  // @return Thee center of the object
  geometry_msgs::msg::Point reference_point_to_center(
    const geometry_msgs::msg::Point & reference_point, double yaw, double length, double width,
    int reference_index);

  /// @brief Convert ARS548 detections to a autoware's radar PointCloud2 msg
  /// @param msg The ARS548 objects list msg
  /// @return Resulting PointCloud2 msg
  sensor_msgs::msg::PointCloud2 convert_to_autoware_radar_detections(
    const continental_msgs::msg::ContinentalArs548DetectionList & msg);

  /// @brief Convert ARS548 objects to a autoware's PointCloud2 msg
  /// @param msg The ARS548 objects list msg
  /// @return Resulting RadarObjects msg
  autoware_sensing_msgs::msg::RadarObjects convert_to_autoware_radar_objects(
    const continental_msgs::msg::ContinentalArs548ObjectList & msg);

  void initialize_sync_diagnostics(rclcpp::Node * parent_node);

  /// @brief Convert ARS548 detections to a pointcloud
  /// @param msg The ARS548 detection list msg
  /// @return Resulting detection pointcloud
  pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Detection>::Ptr
  convert_to_pointcloud(const continental_msgs::msg::ContinentalArs548DetectionList & msg);

  /// @brief Convert ARS548 objects to a pointcloud
  /// @param msg The ARS548 object list msg
  /// @return Resulting object pointcloud
  pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Object>::Ptr
  convert_to_pointcloud(const continental_msgs::msg::ContinentalArs548ObjectList & msg);

  /// @brief Convert ARS548 detections to a standard RadarScan msg
  /// @param msg The ARS548 detection list msg
  /// @return Resulting RadarScan msg
  radar_msgs::msg::RadarScan convert_to_radar_scan(
    const continental_msgs::msg::ContinentalArs548DetectionList & msg);

  /// @brief Convert ARS548 objects to a standard RadarTracks msg
  /// @param msg The ARS548 object list msg
  /// @return Resulting RadarTracks msg
  radar_msgs::msg::RadarTracks convert_to_radar_tracks(
    const continental_msgs::msg::ContinentalArs548ObjectList & msg);

  /// @brief Convert ARS548 objects to a standard MarkerArray msg
  /// @param msg The ARS548 object list msg
  /// @return Resulting MarkerArray msg
  visualization_msgs::msg::MarkerArray convert_to_markers(
    const continental_msgs::msg::ContinentalArs548ObjectList & msg);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /// @brief Make a RateBoundStatus object from the node parameters
  /// @param node The node to read the parameters from
  /// @param name The name of the rate bound status
  /// @return RateBoundStatus for OK and WARN diagnostics
  static custom_diagnostic_tasks::RateBoundStatus make_rate_bound_status(
    rclcpp::Node * const node, const std::string & name)
  {
    static constexpr bool immediate_error_report = false;
    static constexpr bool immediate_relax_state = true;

    auto read_int_param = [&node](const std::string & param_name) {
      if (node->has_parameter(param_name)) {
        return node->get_parameter(param_name).as_int();
      }
      return node->declare_parameter<int64_t>(param_name, param_read_only());
    };

    auto read_fp_param = [&node](const std::string & param_name) {
      if (node->has_parameter(param_name)) {
        return node->get_parameter(param_name).as_double();
      }
      return node->declare_parameter<double>(param_name, param_read_only());
    };

    double min_ok_hz = read_fp_param("diagnostics.rate_bound_status.frequency_ok.min_hz");
    double min_warn_hz = read_fp_param("diagnostics.rate_bound_status.frequency_warn.min_hz");
    auto num_frame_transition =
      static_cast<size_t>(read_int_param("diagnostics.rate_bound_status.num_frame_transition"));

    custom_diagnostic_tasks::RateBoundStatusParam ok_params(min_ok_hz);
    custom_diagnostic_tasks::RateBoundStatusParam warn_params(min_warn_hz);

    return {node,
            ok_params,
            warn_params,
            num_frame_transition,
            immediate_error_report,
            immediate_relax_state,
            name};
  }

  /// @brief Make a HysteresisStateMachine object from the node parameters
  /// @param node The node to read the parameters from
  /// @param diagnostics_ns The diagnostics namespace for parameter
  /// @return HysteresisStateMachine for internal diagnostics
  static custom_diagnostic_tasks::HysteresisStateMachine make_hysteresis_status(
    rclcpp::Node * const node, const std::string & diagnostics_ns)
  {
    static constexpr bool immediate_error_report = false;
    static constexpr bool immediate_relax_state = true;

    auto read_int_param = [&node](const std::string & param_name) {
      if (node->has_parameter(param_name)) {
        return node->get_parameter(param_name).as_int();
      }
      return node->declare_parameter<int64_t>(param_name, param_read_only());
    };

    auto num_frame_transition = static_cast<size_t>(
      read_int_param("diagnostics." + diagnostics_ns + ".num_frame_transition"));

    return custom_diagnostic_tasks::HysteresisStateMachine(
      num_frame_transition, immediate_error_report, immediate_relax_state);
  }

  custom_diagnostic_tasks::RateBoundStatus objects_rate_bound_status_;
  custom_diagnostic_tasks::RateBoundStatus detections_rate_bound_status_;
  custom_diagnostic_tasks::HysteresisStateMachine blockage_status_;
  custom_diagnostic_tasks::HysteresisStateMachine internal_status_;
  nebula::ros::LivenessMonitor liveness_monitor_;
  diagnostic_updater::Updater objects_diagnostics_updater_;
  diagnostic_updater::Updater detections_diagnostics_updater_;
  diagnostic_updater::Updater liveness_diagnostics_updater_;

  nebula::Status status_;
  rclcpp::Logger logger_;
  rclcpp::Node * const parent_node_;

  std::shared_ptr<const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>
    config_ptr_ RCPPUTILS_TSA_GUARDED_BY(mtx_config_ptr_);

  std::shared_ptr<drivers::continental_ars548::ContinentalARS548Decoder> driver_ptr_
    RCPPUTILS_TSA_GUARDED_BY(mtx_driver_ptr_);

  std::mutex mtx_driver_ptr_;
  std::shared_mutex mtx_config_ptr_;

  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_{};

  rclcpp::Publisher<continental_msgs::msg::ContinentalArs548DetectionList>::SharedPtr
    detection_list_pub_{};
  rclcpp::Publisher<continental_msgs::msg::ContinentalArs548ObjectList>::SharedPtr
    object_list_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_pointcloud_pub_{};
  rclcpp::Publisher<autoware_sensing_msgs::msg::RadarObjects>::SharedPtr autoware_objects_pub_{};
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr scan_raw_pub_{};
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr objects_raw_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_markers_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{};
  rclcpp::Publisher<autoware_sensing_msgs::msg::RadarInfo>::SharedPtr radar_info_pub_{};

  autoware_sensing_msgs::msg::RadarInfo radar_info_msg_{};
  std::size_t detection_msgs_counter_{0};

  std::optional<SyncToolingPlugin> sync_tooling_plugin_;

  std::unordered_set<int> previous_ids_{};

  uint32_t latest_config_cycle_time_ms_{};

  constexpr static int reference_points_num = 9;
  constexpr static std::array<std::array<double, 2>, reference_points_num> reference_to_center_ = {
    {{{-1.0, -1.0}},
     {{-1.0, 0.0}},
     {{-1.0, 1.0}},
     {{0.0, 1.0}},
     {{1.0, 1.0}},
     {{1.0, 0.0}},
     {{1.0, -1.0}},
     {{0.0, -1.0}},
     {{0.0, 0.0}}}};
};
}  // namespace nebula::ros
