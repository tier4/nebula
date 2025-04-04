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

#include "nebula_ros/continental/continental_srr520_ros_wrapper.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
ContinentalSRR520RosWrapper::ContinentalSRR520RosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "continental_srr520_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *config_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, config_ptr_);
    decoder_wrapper_.emplace(this, config_ptr_, hw_interface_wrapper_->hw_interface());
  } else {
    decoder_wrapper_.emplace(this, config_ptr_, nullptr);
  }

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  if (launch_hw_) {
    hw_interface_wrapper_->hw_interface()->register_packet_callback(
      std::bind(
        &ContinentalSRR520RosWrapper::receive_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS(),
      std::bind(
        &ContinentalSRR520RosWrapper::receive_packets_callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&ContinentalSRR520RosWrapper::on_parameter_change, this, std::placeholders::_1));
}

nebula::Status ContinentalSRR520RosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration config;

  config.sensor_model = nebula::drivers::sensor_model_from_string(
    declare_parameter<std::string>("sensor_model", param_read_only()));
  config.interface = declare_parameter<std::string>("interface", param_read_only());
  config.receiver_timeout_sec =
    static_cast<float>(declare_parameter<double>("receiver_timeout_sec", param_read_only()));
  config.sender_timeout_sec =
    static_cast<float>(declare_parameter<double>("sender_timeout_sec", param_read_only()));
  config.filters = declare_parameter<std::string>("filters", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_write());
  config.base_frame = declare_parameter<std::string>("base_frame", param_read_write());
  config.use_bus_time = declare_parameter<bool>("use_bus_time", param_read_only());
  config.configuration_vehicle_wheelbase = static_cast<float>(
    declare_parameter<double>("configuration_vehicle_wheelbase", param_read_only()));

  if (config.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  auto new_config_ptr = std::make_shared<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(config);
  return validate_and_set_config(new_config_ptr);
}

Status ContinentalSRR520RosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
    new_config)
{
  if (new_config->sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  if (new_config->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->on_config_change(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->on_config_change(new_config);
  }

  config_ptr_ = new_config;
  return Status::OK;
}

void ContinentalSRR520RosWrapper::receive_packets_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring NebulaPackets. Launch with launch_hw:=false to enable NebulaPackets "
      "replay.");
    return;
  }

  for (auto & packet : packets_msg->packets) {
    auto nebula_packet_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_packet_ptr->stamp = packet.stamp;
    nebula_packet_ptr->data = std::move(packet.data);

    decoder_wrapper_->process_packet(std::move(nebula_packet_ptr));
  }
}

void ContinentalSRR520RosWrapper::receive_packet_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> msg_ptr)
{
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  decoder_wrapper_->process_packet(std::move(msg_ptr));
}

Status ContinentalSRR520RosWrapper::get_status()
{
  return wrapper_status_;
}

Status ContinentalSRR520RosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  hw_interface_wrapper_->sensor_interface_start();

  return hw_interface_wrapper_->status();
}

rcl_interfaces::msg::SetParametersResult ContinentalSRR520RosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::continental_srr520::ContinentalSRR520SensorConfiguration new_config(*config_ptr_);

  bool got_any =
    get_param(p, "frame_id", new_config.frame_id) |
    get_param(p, "base_frame", new_config.base_frame) |
    get_param(p, "use_bus_time", new_config.use_bus_time) |
    get_param(p, "configuration_vehicle_wheelbase", new_config.configuration_vehicle_wheelbase);

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  auto new_config_ptr = std::make_shared<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(new_config);
  auto status = validate_and_set_config(new_config_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalSRR520RosWrapper)
}  // namespace nebula::ros
