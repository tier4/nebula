// Copyright 2026 TIER IV, Inc.
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

#ifndef NEBULA_OUSTER_ROS_WRAPPER_HPP
#define NEBULA_OUSTER_ROS_WRAPPER_HPP

#include "nebula_ouster_decoders/ouster_decoder.hpp"
#include "nebula_ouster_hw_interfaces/ouster_hw_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_ros/diagnostics/liveness_monitor.hpp>
#include <nebula_core_ros/diagnostics/rate_bound_status.hpp>
#include <nebula_ouster_common/ouster_configuration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace nebula::ros
{

struct ConfigError
{
  enum class Code : uint8_t {
    PARAMETER_DECLARATION_FAILED,  ///< Parameter declaration/read failed.
    PARAMETER_VALIDATION_FAILED,   ///< Parameter value failed semantic validation.
  };

  Code code;
  std::string message;
};

/// @brief Read and validate Ouster driver configuration from ROS parameters.
/// @param node Node used to declare/read parameters.
/// @return Parsed OusterSensorConfiguration or ConfigError on validation failure.
util::expected<drivers::OusterSensorConfiguration, ConfigError> load_config_from_ros_parameters(
  rclcpp::Node & node);

/// @brief ROS 2 wrapper for the Ouster LiDAR driver
/// @details This node bridges the C++ driver with ROS 2.
/// Responsibilities:
/// - Turn ROS 2 parameters into sensor configuration
/// - Initialize decoder and hardware interface
/// - Forward packets from HW interface and pass to decoder
/// - Convert decoded point clouds to ROS messages
/// - Publish point clouds on ROS topics
/// - Optionally: provide services for runtime configuration
class OusterRosWrapper : public rclcpp::Node
{
public:
  struct Error
  {
    enum class Code : uint8_t {
      HW_INTERFACE_NOT_INITIALIZED,  ///< Stream start requested while HW interface is absent.
      HW_STREAM_START_FAILED,        ///< Underlying HW interface failed to start.
    };

    Code code;
    std::string message;
  };

  /// @brief Construct the ROS 2 node and initialize decoder + optional HW stream.
  /// @param options Standard ROS 2 component/node options.
  /// @throws std::runtime_error on invalid configuration or startup failures.
  explicit OusterRosWrapper(const rclcpp::NodeOptions & options);
  ~OusterRosWrapper() override;

private:
  /// @brief Resources used only when launch_hw is true.
  struct OnlineMode
  {
    explicit OnlineMode(drivers::ConnectionConfiguration connection_configuration)
    : hw_interface(std::move(connection_configuration))
    {
    }

    drivers::OusterHwInterface hw_interface;
    rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub;
    std::unique_ptr<nebula_msgs::msg::NebulaPackets> current_scan_packets_msg{
      std::make_unique<nebula_msgs::msg::NebulaPackets>()};
  };

  /// @brief Resources used only when launch_hw is false.
  struct OfflineMode
  {
    rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub;
  };

  struct Publishers
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr receive_duration_ms;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr decode_duration_ms;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publish_duration_ms;
  };

  struct Diagnostics
  {
    explicit Diagnostics(rclcpp::Node * node) : updater(node) {}

    diagnostic_updater::Updater updater;
    std::optional<custom_diagnostic_tasks::RateBoundStatus> publish_rate;
    std::optional<LivenessMonitor> packet_liveness;
  };

  /// @brief Publish a decoded pointcloud to ROS.
  /// @param pointcloud Decoded pointcloud from the decoder.
  /// @param timestamp_s Scan timestamp in seconds, epoch time.
  void publish_pointcloud_callback(
    const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s);

  /// @brief Process one received UDP packet through the decoder pipeline.
  /// @param packet Raw packet payload.
  /// @param metadata Transport metadata provided by the UDP receiver.
  void receive_cloud_packet_callback(
    std::vector<uint8_t> & packet, const drivers::connections::UdpSocket::RxMetadata & metadata);

  /// @brief Process one replayed NebulaPackets message.
  /// @param packets_msg Packed scan data used for software-only replay.
  void receive_packets_message_callback(
    std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg);

  /// @brief Configure common diagnostics used by most sensor integrations.
  void initialize_diagnostics();

  /// @brief Decode one packet and publish relevant outputs.
  /// @param packet Raw packet payload.
  /// @param receive_duration_ns Time spent in transport receive path for this packet.
  void process_packet(const std::vector<uint8_t> & packet, uint64_t receive_duration_ns);

  /// @brief Publish receive/decode/publish debug durations.
  /// @param receive_duration_ns Time spent in transport receive path.
  /// @param decode_duration_ns Time spent decoding one packet.
  /// @param publish_duration_ns Time spent publishing one completed scan callback.
  void publish_debug_durations(
    uint64_t receive_duration_ns, uint64_t decode_duration_ns, uint64_t publish_duration_ns) const;

  static const char * to_cstr(Error::Code code);

  drivers::OusterSensorConfiguration config_;
  std::string frame_id_;
  Publishers publishers_;
  Diagnostics diagnostics_;

  std::optional<drivers::OusterDecoder> decoder_;
  /// @brief Exactly one runtime mode is active: offline replay or online hardware mode.
  /// @details During construction, neither mode is active.
  std::variant<std::monostate, OfflineMode, OnlineMode> runtime_mode_;
};

}  // namespace nebula::ros

#endif  // NEBULA_OUSTER_ROS_WRAPPER_HPP
