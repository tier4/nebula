// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/velodyne_ros_wrapper.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <pthread.h>  // debug only

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
VelodyneRosWrapper::VelodyneRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr),
  packet_queue_(3000),
  hw_interface_wrapper_(),
  hw_monitor_wrapper_(),
  decoder_wrapper_(),
  hw_reconfigure_timer_(this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&VelodyneRosWrapper::reconfigure_hw_interface, this)))
{
  hw_reconfigure_timer_->cancel();
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_write());
  use_udp_only_ = declare_parameter<bool>("udp_only", param_read_only());

  if (use_udp_only_) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "UDP-only mode is enabled. Settings checks, synchronization, and diagnostics publishing are "
      "disabled.");
  }

  if (launch_hw_) {
    bringup_hw(use_udp_only_);
  } else {
    create_packet_subscriber();
  }

  setup_decoder();

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&VelodyneRosWrapper::on_parameter_change, this, std::placeholders::_1));
}

void VelodyneRosWrapper::reconfigure_hw_interface()
{
  if (restart_hw_) {
    bringup_hw(use_udp_only_);
    setup_decoder();
    restart_hw_ = false;
    hw_reconfigure_timer_->cancel();
  }

  if (restart_packet_subscriber_) {
    create_packet_subscriber();
    setup_decoder();
    restart_packet_subscriber_ = false;
    hw_reconfigure_timer_->cancel();
  }
}

void VelodyneRosWrapper::create_packet_subscriber()
{
  packets_sub_ = create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&VelodyneRosWrapper::receive_scan_message_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
}

void VelodyneRosWrapper::set_decoder_wrapper()
{
  // If there's an existing decoder wrapper, reset it
  if (decoder_wrapper_) {
    decoder_wrapper_.reset();
  }
  decoder_wrapper_.emplace(
    this, hw_interface_wrapper_ ? hw_interface_wrapper_->hw_interface() : nullptr, sensor_cfg_ptr_);
}

void VelodyneRosWrapper::decoder_wrapper_thread(std::stop_token stoken)
{
  RCLCPP_DEBUG(get_logger(), "Starting stream");

  while (!stoken.stop_requested()) {
    auto [packet, valid] = packet_queue_.pop(std::chrono::milliseconds(100));
    if (valid) {
      decoder_wrapper_->process_cloud_packet(std::move(packet));
    } else {
      continue;
    }
  }
  RCLCPP_INFO(get_logger(), "Gracefully stopped decoder thread");
}

void VelodyneRosWrapper::bringup_hw(bool use_udp_only)
{
  hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_, use_udp_only);

  if (!use_udp_only) {  // hardware monitor requires HTTP connection
    hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->hw_interface(), sensor_cfg_ptr_);
  }
  hw_interface_wrapper_->hw_interface()->register_scan_callback(
    std::bind(&VelodyneRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1));
  stream_start();
}

nebula::Status VelodyneRosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::VelodyneSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::sensor_model_from_string(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::return_mode_from_string(_return_mode);

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
    descriptor.additional_constraints = "from 300 to 1200, in increments of 60";
    descriptor.integer_range = int_range(300, 1200, 60);
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

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::VelodyneSensorConfiguration>(config);
  return validate_and_set_config(new_cfg_ptr);
}

Status VelodyneRosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> & new_config)
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

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->on_config_change(new_config);
  }
  if (hw_monitor_wrapper_) {
    hw_monitor_wrapper_->on_config_change(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->on_config_change(new_config);
  }

  sensor_cfg_ptr_ = new_config;
  return Status::OK;
}

void VelodyneRosWrapper::receive_scan_message_callback(
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_msg)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring received VelodyneScan. Launch with launch_hw:=false to enable VelodyneScan "
      "replay.");
    return;
  }

  for (auto & pkt : scan_msg->packets) {
    auto nebula_pkt_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_pkt_ptr->stamp = pkt.stamp;
    std::copy(pkt.data.begin(), pkt.data.end(), std::back_inserter(nebula_pkt_ptr->data));

    packet_queue_.push(std::move(nebula_pkt_ptr));
  }
}

Status VelodyneRosWrapper::get_status()
{
  return wrapper_status_;
}

Status VelodyneRosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  return hw_interface_wrapper_->hw_interface()->sensor_interface_start();
}

void VelodyneRosWrapper::cleanup_on_hw_reconfigure()
{
  while (true) {
    if (decoder_thread_.joinable()) {
      stop_decoder_thread();
      break;
    }
  }
  decoder_thread_.join();
  if (hw_interface_wrapper_) {
    hw_interface_wrapper_.reset();
  }
  if (hw_monitor_wrapper_) {
    hw_monitor_wrapper_.reset();
  }
}

void VelodyneRosWrapper::setup_decoder()
{
  set_decoder_wrapper();
  decoder_thread_ = std::jthread(&VelodyneRosWrapper::decoder_wrapper_thread, this);
  // Set the thread name
  pthread_setname_np(decoder_thread_.native_handle(), "VelDecThread");  // debug only
}

void VelodyneRosWrapper::reset_packet_subscriber()
{
  if (packets_sub_) {
    packets_sub_.reset();
  }
}

rcl_interfaces::msg::SetParametersResult VelodyneRosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::VelodyneSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string _return_mode = "";
  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) | get_param(p, "min_range", new_cfg.min_range) |
    get_param(p, "max_range", new_cfg.max_range) |
    get_param(p, "rotation_speed", new_cfg.rotation_speed) |
    get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
    get_param(p, "launch_hw", launch_hw_);

  if (got_any && launch_hw_ && !hw_interface_wrapper_) {
    RCLCPP_INFO(get_logger(), "Cancelling packet subscription...");
    reset_packet_subscriber();
    RCLCPP_INFO(get_logger(), "Resetting HW interface and HW Monitor wrappers...");
    cleanup_on_hw_reconfigure();
    RCLCPP_INFO(get_logger(), "(re)Configuring HW interface");
    restart_hw_ = true;
    hw_reconfigure_timer_->reset();
  }

  if (got_any && !launch_hw_ && hw_interface_wrapper_) {
    RCLCPP_INFO(get_logger(), "Resetting HW interface and HW Monitor wrappers...");
    cleanup_on_hw_reconfigure();
    reset_packet_subscriber();
    RCLCPP_INFO(get_logger(), "(re) Configuring packet subscriber");
    restart_packet_subscriber_ = true;
    hw_reconfigure_timer_->reset();
  }

  // Currently, HW interface and monitor wrappers have only read-only parameters, so their update
  // logic is not implemented
  if (decoder_wrapper_) {
    auto result = decoder_wrapper_->on_parameter_change(p);
    if (!result.successful) {
      return result;
    }
  }

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::return_mode_from_string(_return_mode);

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::VelodyneSensorConfiguration>(new_cfg);
  auto status = validate_and_set_config(new_cfg_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void VelodyneRosWrapper::receive_cloud_packet_callback(std::vector<uint8_t> & packet)
{
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  const auto now = std::chrono::high_resolution_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);

  if (!packet_queue_.try_push(std::move(msg_ptr))) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packet(s) dropped");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(VelodyneRosWrapper)
}  // namespace nebula::ros
