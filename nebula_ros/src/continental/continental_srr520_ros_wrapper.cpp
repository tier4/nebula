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

#include <nebula_ros/continental/continental_srr520_ros_wrapper.hpp>

namespace nebula
{
namespace ros
{
ContinentalSRR520RosWrapper::ContinentalSRR520RosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "continental_srr520_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  packet_queue_(3000),
  hw_interface_wrapper_(),
  decoder_wrapper_()
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = DeclareAndGetSensorConfigParams();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << wrapper_status_).str());
  }

  RCLCPP_INFO_STREAM(get_logger(), "SensorConfig:" << *config_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, config_ptr_);
    decoder_wrapper_.emplace(this, config_ptr_, hw_interface_wrapper_->HwInterface());
  } else {
    decoder_wrapper_.emplace(this, config_ptr_, nullptr);
  }

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  decoder_thread_ = std::thread([this]() {
    while (true) {
      decoder_wrapper_->ProcessPacket(packet_queue_.pop());
    }
  });

  if (launch_hw_) {
    hw_interface_wrapper_->HwInterface()->RegisterPacketCallback(
      std::bind(&ContinentalSRR520RosWrapper::ReceivePacketCallback, this, std::placeholders::_1));
    StreamStart();
  } else {
    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS(),
      std::bind(&ContinentalSRR520RosWrapper::ReceivePacketsCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&ContinentalSRR520RosWrapper::OnParameterChange, this, std::placeholders::_1));
}

nebula::Status ContinentalSRR520RosWrapper::DeclareAndGetSensorConfigParams()
{
  nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration config;

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", descriptor);
    config.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("interface", descriptor);
    config.interface = this->get_parameter("interface").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("receiver_timeout_sec", descriptor);
    config.receiver_timeout_sec = this->get_parameter("receiver_timeout_sec").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("sender_timeout_sec", descriptor);
    config.sender_timeout_sec = this->get_parameter("sender_timeout_sec").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("filters", descriptor);
    config.filters = this->get_parameter("filters").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", descriptor);
    config.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("base_frame", descriptor);
    config.base_frame = this->get_parameter("base_frame").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("use_bus_time", descriptor);
    config.use_bus_time = this->get_parameter("use_bus_time").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("configuration_vehicle_wheelbase", descriptor);
    config.configuration_vehicle_wheelbase =
      static_cast<float>(this->get_parameter("configuration_vehicle_wheelbase").as_double());
  }

  if (config.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  auto new_config_ptr = std::make_shared<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(config);
  return ValidateAndSetConfig(new_config_ptr);
}

Status ContinentalSRR520RosWrapper::ValidateAndSetConfig(
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
    hw_interface_wrapper_->OnConfigChange(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->OnConfigChange(new_config);
  }

  config_ptr_ = new_config;
  return Status::OK;
}

void ContinentalSRR520RosWrapper::ReceivePacketsCallback(
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

    packet_queue_.push(std::move(nebula_packet_ptr));
  }
}

void ContinentalSRR520RosWrapper::ReceivePacketCallback(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> msg_ptr)
{
  if (!decoder_wrapper_ || decoder_wrapper_->Status() != Status::OK) {
    return;
  }

  if (!packet_queue_.try_push(std::move(msg_ptr))) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packet(s) dropped");
  }
}

Status ContinentalSRR520RosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status ContinentalSRR520RosWrapper::StreamStart()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->Status() != Status::OK) {
    return hw_interface_wrapper_->Status();
  }

  hw_interface_wrapper_->SensorInterfaceStart();

  return hw_interface_wrapper_->Status();
}

rcl_interfaces::msg::SetParametersResult ContinentalSRR520RosWrapper::OnParameterChange(
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
  auto status = ValidateAndSetConfig(new_config_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = (std::stringstream() << "Invalid configuration: " << status).str();
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalSRR520RosWrapper)
}  // namespace ros
}  // namespace nebula
