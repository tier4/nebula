// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hesai_ros_wrapper.hpp"

#include <utility>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula
{
namespace ros
{
HesaiRosWrapper::HesaiRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = DeclareAndGetSensorConfigParams();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << wrapper_status_).str());
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->HwInterface(), sensor_cfg_ptr_);
  }

  decoder_wrapper_.emplace(
    this, hw_interface_wrapper_ ? hw_interface_wrapper_->HwInterface() : nullptr, sensor_cfg_ptr_);

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  if (launch_hw_) {
    hw_interface_wrapper_->HwInterface()->RegisterScanCallback(
      std::bind(&HesaiRosWrapper::ReceiveCloudPacketCallback, this, std::placeholders::_1));
    StreamStart();
  } else {
    packets_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS(),
      std::bind(&HesaiRosWrapper::ReceiveScanMessageCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&HesaiRosWrapper::OnParameterChange, this, std::placeholders::_1));
}

nebula::Status HesaiRosWrapper::DeclareAndGetSensorConfigParams()
{
  nebula::drivers::HesaiSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::SensorModelFromString(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::ReturnModeFromStringHesai(_return_mode, config.sensor_model);

  config.host_ip = declare_parameter<std::string>("host_ip", param_read_only());
  config.sensor_ip = declare_parameter<std::string>("sensor_ip", param_read_only());
  config.data_port = declare_parameter<uint16_t>("data_port", param_read_only());
  config.gnss_port = declare_parameter<uint16_t>("gnss_port", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_write());

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.])";
    descriptor.floating_point_range = float_range(0, 360, 0.01);
    config.scan_phase = declare_parameter<double>("scan_phase", descriptor);
  }

  config.min_range = declare_parameter<double>("min_range", param_read_write());
  config.max_range = declare_parameter<double>("max_range", param_read_write());
  config.packet_mtu_size = declare_parameter<uint16_t>("packet_mtu_size", param_read_only());

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    RCLCPP_DEBUG_STREAM(get_logger(), config.sensor_model);
    if (config.sensor_model == nebula::drivers::SensorModel::HESAI_PANDARAT128) {
      descriptor.additional_constraints = "200, 400";
      descriptor.integer_range = int_range(200, 400, 200);
    } else {
      descriptor.additional_constraints = "300, 600, 1200";
      descriptor.integer_range = int_range(300, 1200, 300);
    }
    config.rotation_speed = declare_parameter<uint16_t>("rotation_speed", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.integer_range = int_range(0, 360, 1);
    config.cloud_min_angle = declare_parameter<uint16_t>("cloud_min_angle", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.integer_range = int_range(0, 360, 1);
    config.cloud_max_angle = declare_parameter<uint16_t>("cloud_max_angle", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    descriptor.floating_point_range = float_range(0.01, 0.5, 0.01);
    config.dual_return_distance_threshold =
      declare_parameter<double>("dual_return_distance_threshold", descriptor);
  }

  auto _ptp_profile = declare_parameter<std::string>("ptp_profile", param_read_only());
  config.ptp_profile = drivers::PtpProfileFromString(_ptp_profile);

  auto _ptp_transport = declare_parameter<std::string>("ptp_transport_type", param_read_only());
  config.ptp_transport_type = drivers::PtpTransportTypeFromString(_ptp_transport);

  if (
    config.ptp_transport_type != drivers::PtpTransportType::L2 &&
    config.ptp_profile != drivers::PtpProfile::IEEE_1588v2 &&
    config.ptp_profile != drivers::PtpProfile::UNKNOWN_PROFILE) {
    RCLCPP_WARN_STREAM(
      get_logger(), "PTP transport was set to '" << _ptp_transport << "' but PTP profile '"
                                                 << _ptp_profile
                                                 << "' only supports 'L2'. Setting it to 'L2'.");
    config.ptp_transport_type = drivers::PtpTransportType::L2;
    set_parameter(rclcpp::Parameter("ptp_transport_type", "L2"));
  }

  auto _ptp_switch = declare_parameter<std::string>("ptp_switch_type", param_read_only());
  config.ptp_switch_type = drivers::PtpSwitchTypeFromString(_ptp_switch);

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.integer_range = int_range(0, 127, 1);
    config.ptp_domain = declare_parameter<uint8_t>("ptp_domain", descriptor);
  }

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::HesaiSensorConfiguration>(config);
  return ValidateAndSetConfig(new_cfg_ptr);
}

Status HesaiRosWrapper::ValidateAndSetConfig(
  std::shared_ptr<const drivers::HesaiSensorConfiguration> & new_config)
{
  if (new_config->sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (new_config->return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (new_config->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_profile == nebula::drivers::PtpProfile::UNKNOWN_PROFILE) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalid PTP Profile Provided. Please use '1588v2', '802.1as' or 'automotive'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_transport_type == nebula::drivers::PtpTransportType::UNKNOWN_TRANSPORT) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Invalid PTP Transport Provided. Please use 'udp' or 'l2', 'udp' is only available when "
      "using the '1588v2' PTP Profile");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_switch_type == nebula::drivers::PtpSwitchType::UNKNOWN_SWITCH) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalid PTP Switch Type Provided. Please use 'tsn' or 'non_tsn'");
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->OnConfigChange(new_config);
  }
  if (hw_monitor_wrapper_) {
    hw_monitor_wrapper_->OnConfigChange(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->OnConfigChange(new_config);
  }

  sensor_cfg_ptr_ = new_config;
  return Status::OK;
}

void HesaiRosWrapper::ReceiveScanMessageCallback(
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_msg)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring received PandarScan. Launch with launch_hw:=false to enable PandarScan replay.");
    return;
  }

  for (auto & pkt : scan_msg->packets) {
    auto nebula_pkt_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_pkt_ptr->stamp = pkt.stamp;
    std::copy(pkt.data.begin(), pkt.data.end(), std::back_inserter(nebula_pkt_ptr->data));

    decoder_wrapper_->ProcessCloudPacket(std::move(nebula_pkt_ptr));
  }
}

Status HesaiRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status HesaiRosWrapper::StreamStart()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->Status() != Status::OK) {
    return hw_interface_wrapper_->Status();
  }

  return hw_interface_wrapper_->HwInterface()->SensorInterfaceStart();
}

rcl_interfaces::msg::SetParametersResult HesaiRosWrapper::OnParameterChange(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::HesaiSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string _return_mode = "";
  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) | get_param(p, "min_range", new_cfg.min_range) |
    get_param(p, "max_range", new_cfg.max_range) |
    get_param(p, "rotation_speed", new_cfg.rotation_speed) |
    get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold);

  // Currently, HW interface and monitor wrappers have only read-only parameters, so their update
  // logic is not implemented
  if (decoder_wrapper_) {
    auto result = decoder_wrapper_->OnParameterChange(p);
    if (!result.successful) {
      return result;
    }
  }

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::ReturnModeFromString(_return_mode);

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::HesaiSensorConfiguration>(new_cfg);
  auto status = ValidateAndSetConfig(new_cfg_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = (std::stringstream() << "Invalid configuration: " << status).str();
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void HesaiRosWrapper::ReceiveCloudPacketCallback(std::vector<uint8_t> & packet)
{
  if (!decoder_wrapper_ || decoder_wrapper_->Status() != Status::OK) {
    return;
  }

  const auto now = std::chrono::high_resolution_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);

  decoder_wrapper_->ProcessCloudPacket(std::move(msg_ptr));
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiRosWrapper)
}  // namespace ros
}  // namespace nebula
