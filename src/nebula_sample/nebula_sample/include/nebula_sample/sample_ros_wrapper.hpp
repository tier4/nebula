// Copyright 2025 TIER IV, Inc.
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

#ifndef NEBULA_SAMPLE_ROS_WRAPPER_HPP
#define NEBULA_SAMPLE_ROS_WRAPPER_HPP

#include "nebula_sample_decoders/sample_decoder.hpp"
#include "nebula_sample_hw_interfaces/sample_hw_interface.hpp"

#include <nebula_core_common/util/expected.hpp>
#include <nebula_sample_common/sample_configuration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace nebula::ros
{

struct ConfigError
{
  std::string message;
};

util::expected<drivers::SampleSensorConfiguration, ConfigError> load_config_from_ros_parameters(
  rclcpp::Node & node);

/// @brief ROS 2 wrapper for the Sample LiDAR driver
/// @details This node bridges the C++ driver with ROS 2.
/// Responsibilities:
/// - Declare and read ROS parameters for sensor configuration
/// - Initialize the driver and hardware interface
/// - Receive packets from HW interface and pass to driver
/// - Convert decoded point clouds to ROS messages
/// - Publish point clouds on ROS topics
/// - Optionally: provide services for runtime configuration
class SampleRosWrapper : public rclcpp::Node
{
public:
  enum class Error : uint8_t {
    HW_INTERFACE_NOT_INITIALIZED,
    HW_STREAM_START_FAILED,
  };

  /// @brief Constructor
  /// @param options ROS node options
  /// @details Initializes the driver, HW interface, and ROS publishers
  explicit SampleRosWrapper(const rclcpp::NodeOptions & options);

  /// @brief Destructor
  /// @details Stops the hardware interface cleanly
  ~SampleRosWrapper() override;

  /// @brief Start the sensor data stream
  /// @return Nothing on success, error otherwise
  util::expected<std::monostate, Error> stream_start();

private:
  /// @brief Callback for incoming UDP packets
  /// @param packet Raw packet data from the sensor
  /// @param metadata Packet metadata (timestamp, source IP, etc.)
  /// @details This is called by the HW interface when a packet arrives.
  /// Passes the packet to the driver for decoding.
  void receive_cloud_packet_callback(
    const std::vector<uint8_t> & packet,
    const drivers::connections::UdpSocket::RxMetadata & metadata);

  static const char * to_cstr(Error error);
  static const char * to_cstr(drivers::SampleHwInterface::Error error);

  drivers::SampleSensorConfiguration config_;
  std::string frame_id_;

  std::optional<drivers::SampleDecoder> decoder_;
  std::optional<drivers::SampleHwInterface> hw_interface_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

}  // namespace nebula::ros

#endif  // NEBULA_SAMPLE_ROS_WRAPPER_HPP
