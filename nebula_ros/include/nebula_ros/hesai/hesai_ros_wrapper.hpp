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

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_ros/hesai/decoder_wrapper.hpp"
#include "nebula_ros/hesai/hw_interface_wrapper.hpp"
#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace nebula::ros
{

/// @brief Ros wrapper of hesai driver
class HesaiRosWrapper final : public rclcpp::Node
{
  using get_calibration_result_t = nebula::util::expected<
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase>, nebula::Status>;

public:
  explicit HesaiRosWrapper(const rclcpp::NodeOptions & options);
  ~HesaiRosWrapper() noexcept override
  {
    if (!hw_interface_wrapper_) return;
    hw_interface_wrapper_->hw_interface()->sensor_interface_stop();
  };

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Start point cloud streaming (Call SensorInterfaceStart of HwInterface)
  /// @return Resulting status
  Status stream_start();

private:
  void receive_cloud_packet_callback(const std::vector<uint8_t> & packet);

  void receive_scan_message_callback(std::unique_ptr<pandar_msgs::msg::PandarScan> scan_msg);

  Status declare_and_get_sensor_config_params();

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  Status validate_and_set_config(
    std::shared_ptr<const drivers::HesaiSensorConfiguration> & new_config);

  /// @brief The ROS 2 parameter holding the calibration file path is called differently depending
  /// on the sensor model. This function returns the correct parameter name given a model.
  /// @param model The sensor model
  /// @return std::string The parameter name
  std::string get_calibration_parameter_name(drivers::SensorModel model) const;

  /// @brief Load calibration data from the best available source:
  /// 1. If sensor connected, download and save from sensor
  /// 2. If downloaded file available, load that file
  /// 3. Load the file given by `calibration_file_path`
  /// @param calibration_file_path The file to use if no better option is available
  /// @param ignore_others If true, skip straight so step 3 above, ignoring better calibration
  /// options
  /// @return The calibration data if successful, or an error code if not
  get_calibration_result_t get_calibration_data(
    const std::string & calibration_file_path, bool ignore_others = false);

  Status wrapper_status_;

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_ptr_{};

  rclcpp::Subscription<pandar_msgs::msg::PandarScan>::SharedPtr packets_sub_{};

  bool launch_hw_;

  std::optional<HesaiHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<HesaiHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<HesaiDecoderWrapper> decoder_wrapper_;

  std::mutex mtx_config_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace nebula::ros
