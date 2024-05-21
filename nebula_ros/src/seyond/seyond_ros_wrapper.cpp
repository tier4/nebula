#include "nebula_ros/seyond/seyond_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
SeyondRosWrapper::SeyondRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("seyond_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr),
  packet_queue_(3000),
  hw_interface_wrapper_(),
  hw_monitor_wrapper_(),
  decoder_wrapper_()
{
  // ////////////////////////////////////////
  // Define, get and validate ROS parameters
  // ////////////////////////////////////////

  wrapper_status_ = DeclareAndGetSensorConfigParams();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << wrapper_status_).str());
  }

  RCLCPP_INFO_STREAM(get_logger(), "SensorConfig: " << *sensor_cfg_ptr_);

  // ////////////////////////////////////////
  // Launch hardware wrappers (if enabled)
  // ////////////////////////////////////////

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->HwInterface(), sensor_cfg_ptr_);
  }

  // ////////////////////////////////////////
  // Get calibration data and launch decoder
  // ////////////////////////////////////////

  decoder_wrapper_.emplace(
    this, hw_interface_wrapper_ ? hw_interface_wrapper_->HwInterface() : nullptr, sensor_cfg_ptr_);

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  // The decoder is running in its own thread to not block UDP reception
  decoder_thread_ = std::thread([this]() {
    while (true) {
      decoder_wrapper_->ProcessCloudPacket(std::move(packet_queue_.pop()));
    }
  });

  // ////////////////////////////////////////
  // Configure packet / scan message routing
  // ////////////////////////////////////////

  if (launch_hw_) {
    hw_interface_wrapper_->HwInterface()->RegisterScanCallback(
      std::bind(&SeyondRosWrapper::ReceiveCloudPacketCallback, this, std::placeholders::_1));
    StreamStart();
  } else {
    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS(),
      std::bind(&SeyondRosWrapper::ReceiveScanMessageCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  // ////////////////////////////////////////
  // Enable callbacks for config changes
  // ////////////////////////////////////////

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&SeyondRosWrapper::OnParameterChange, this, std::placeholders::_1));
}

nebula::Status SeyondRosWrapper::DeclareAndGetSensorConfigParams()
{
  SeyondSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", "", param_read_only());
  config.sensor_model = drivers::SensorModelFromString(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", "", param_read_write());
  config.return_mode = drivers::ReturnModeFromString(_return_mode);

  config.host_ip = declare_parameter<std::string>("host_ip", "255.255.255.255", param_read_only());
  config.sensor_ip =
    declare_parameter<std::string>("sensor_ip", "192.168.1.201", param_read_only());
  config.data_port = declare_parameter<uint16_t>("data_port", 2368, param_read_only());
  config.gnss_port = declare_parameter<uint16_t>("gnss_port", 2369, param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", "seyond", param_read_write());

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.])";
    descriptor.floating_point_range = float_range(0, 360, 0.01);
    config.scan_phase = declare_parameter<double>("scan_phase", 0., descriptor);
  }

  config.min_range = declare_parameter<double>("min_range", 0.3, param_read_write());
  config.max_range = declare_parameter<double>("max_range", 300., param_read_write());
  config.packet_mtu_size = declare_parameter<uint16_t>("packet_mtu_size", 1500, param_read_only());

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "300, 600, 1200";
    descriptor.integer_range = int_range(300, 1200, 300);
    config.rotation_speed = declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.integer_range = int_range(0, 360, 1);
    config.cloud_min_angle = declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.integer_range = int_range(0, 360, 1);
    config.cloud_max_angle = declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    descriptor.floating_point_range = float_range(0.01, 0.5, 0.01);
    config.dual_return_distance_threshold =
      declare_parameter<double>("dual_return_distance_threshold", 0.1, descriptor);
  }

  auto _ptp_profile = declare_parameter<std::string>("ptp_profile", "", param_read_only());
  config.ptp_profile = drivers::PtpProfileFromString(_ptp_profile);

  auto _ptp_transport = declare_parameter<std::string>("ptp_transport_type", "", param_read_only());
  config.ptp_transport_type = drivers::PtpTransportTypeFromString(_ptp_transport);

  auto _ptp_switch = declare_parameter<std::string>("ptp_switch_type", "", param_read_only());
  config.ptp_switch_type = drivers::PtpSwitchTypeFromString(_ptp_switch);

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.integer_range = int_range(0, 127, 1);
    config.ptp_domain = declare_parameter<uint8_t>("ptp_domain", 0, descriptor);
  }

  auto new_cfg_ptr = std::make_shared<const SeyondSensorConfiguration>(config);
  return ValidateAndSetConfig(new_cfg_ptr);
}

Status SeyondRosWrapper::ValidateAndSetConfig(
  std::shared_ptr<const SeyondSensorConfiguration> & new_config)
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

void SeyondRosWrapper::ReceiveScanMessageCallback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_msg)
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

    packet_queue_.push(std::move(nebula_pkt_ptr));
  }
}

Status SeyondRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status SeyondRosWrapper::StreamStart()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->Status() != Status::OK) {
    return hw_interface_wrapper_->Status();
  }

  return hw_interface_wrapper_->HwInterface()->SensorInterfaceStart();
}

rcl_interfaces::msg::SetParametersResult SeyondRosWrapper::OnParameterChange(
  const std::vector<rclcpp::Parameter> & p)
{
  std::lock_guard lock(mtx_config_);
  using namespace rcl_interfaces::msg;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  // ////////////////////////////////////////
  // Update sub-wrapper parameters (if any)
  // ////////////////////////////////////////

  // Currently, HW interface and monitor wrappers have only read-only parameters, so their update
  // logic is not implemented
  if (decoder_wrapper_) {
    auto result = decoder_wrapper_->OnParameterChange(p);
    if (!result.successful) {
      return result;
    }
  }

  // ////////////////////////////////////////
  // Create new, updated sensor configuration
  // ////////////////////////////////////////

  SeyondSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string _return_mode = "";
  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) | get_param(p, "min_range", new_cfg.min_range) |
    get_param(p, "max_range", new_cfg.max_range) |
    get_param(p, "rotation_speed", new_cfg.rotation_speed) |
    get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold);

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::ReturnModeFromString(_return_mode);

  // ////////////////////////////////////////
  // Validate and use new config
  // ////////////////////////////////////////

  auto new_cfg_ptr = std::make_shared<const SeyondSensorConfiguration>(new_cfg);
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

void SeyondRosWrapper::ReceiveCloudPacketCallback(std::vector<uint8_t> & packet)
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

  // If the decoder is too slow (= the queue becomes full), packets are dropped here
  if (!packet_queue_.try_push(std::move(msg_ptr))) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packet(s) dropped");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SeyondRosWrapper)
}  // namespace ros
}  // namespace nebula
