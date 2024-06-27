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
#include "nebula_ros/common/mt_queue.hpp"
#include "nebula_ros/hesai/decoder_wrapper.hpp"
#include "nebula_ros/hesai/hw_interface_wrapper.hpp"
#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

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

namespace nebula
{
namespace ros
{

/// @brief Ros wrapper of hesai driver
class HesaiRosWrapper final : public rclcpp::Node
{
  using get_calibration_result_t = nebula::util::expected<
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase>, nebula::Status>;

public:
  explicit HesaiRosWrapper(const rclcpp::NodeOptions & options);
  ~HesaiRosWrapper() noexcept {};

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Start point cloud streaming (Call SensorInterfaceStart of HwInterface)
  /// @return Resulting status
  Status StreamStart();

private:
  void ReceiveCloudPacketCallback(std::vector<uint8_t> & packet);

  void ReceiveScanMessageCallback(std::unique_ptr<pandar_msgs::msg::PandarScan> scan_msg);

  Status DeclareAndGetSensorConfigParams();

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult OnParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  Status ValidateAndSetConfig(
    std::shared_ptr<const drivers::HesaiSensorConfiguration> & new_config);

  /// @brief The ROS 2 parameter holding the calibration file path is called differently depending
  /// on the sensor model. This function returns the correct parameter name given a model.
  /// @param model A sensor model
  /// @return std::string The parameter name
  std::string getCalibrationParameterName(drivers::SensorModel model) const;

  /// @brief Load calibration data from the best available source:
  /// 1. If sensor connected, download and save from sensor
  /// 2. If downloaded file available, load that file
  /// 3. Load the file given by `calibration_file_path`
  /// @param calibration_file_path The file to use if no better option is available
  /// @param ignore_others If true, skip straight so step 3 above, ignoring better calibration
  /// options
  /// @return The calibration data if successful, or an error code if not
  get_calibration_result_t GetCalibrationData(
    const std::string & calibration_file_path, bool ignore_others = false);

  Status wrapper_status_;

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_ptr_{};

  /// @brief Stores received packets that have not been processed yet by the decoder thread
  mt_queue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;
  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  rclcpp::Subscription<pandar_msgs::msg::PandarScan>::SharedPtr packets_sub_{};

  bool launch_hw_;

  std::optional<HesaiHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<HesaiHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<HesaiDecoderWrapper> decoder_wrapper_;

  std::mutex mtx_config_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace ros
}  // namespace nebula
