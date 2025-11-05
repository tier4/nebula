// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/robosense/robosense_ros_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/util/string_conversions.hpp>
#include <rclcpp/qos.hpp>

#include <robosense_msgs/msg/detail/robosense_info_packet__struct.hpp>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
RobosenseRosWrapper::RobosenseRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  info_driver_.emplace(sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, sensor_cfg_ptr_);
  }

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  if (launch_hw_) {
    info_packets_pub_ =
      create_publisher<robosense_msgs::msg::RobosenseInfoPacket>("robosense_info_packets", 10);

    hw_interface_wrapper_->hw_interface()->register_scan_callback(
      std::bind(&RobosenseRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1));
    hw_interface_wrapper_->hw_interface()->register_info_callback(
      std::bind(&RobosenseRosWrapper::receive_info_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<robosense_msgs::msg::RobosenseScan>(
      "robosense_packets", rclcpp::SensorDataQoS(),
      std::bind(&RobosenseRosWrapper::receive_scan_message_callback, this, std::placeholders::_1));
    info_packets_sub_ = create_subscription<robosense_msgs::msg::RobosenseInfoPacket>(
      "robosense_info_packets", 10, [this](const robosense_msgs::msg::RobosenseInfoPacket & msg) {
        std::vector<uint8_t> raw_packet;
        std::copy(msg.packet.data.cbegin(), msg.packet.data.cend(), std::back_inserter(raw_packet));
        receive_info_packet_callback(raw_packet);
      });
    RCLCPP_INFO_STREAM(
      get_logger(), "Hardware connection disabled, listening for packets on "
                      << packets_sub_->get_topic_name() << " and "
                      << info_packets_sub_->get_topic_name());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&RobosenseRosWrapper::on_parameter_change, this, std::placeholders::_1));
}

nebula::Status RobosenseRosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::RobosenseSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::sensor_model_from_string(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::return_mode_from_string_robosense(_return_mode);

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
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    descriptor.floating_point_range = float_range(0.01, 0.5, 0.01);
    config.dual_return_distance_threshold =
      declare_parameter<double>("dual_return_distance_threshold", descriptor);
  }

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(config);
  return validate_and_set_config(new_cfg_ptr);
}

Status RobosenseRosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> & new_config)
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

void RobosenseRosWrapper::receive_scan_message_callback(
  std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_msg)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring received RobosenseScan. Launch with launch_hw:=false to enable RobosenseScan "
      "replay.");
    return;
  }

  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  for (auto & pkt : scan_msg->packets) {
    auto nebula_pkt_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_pkt_ptr->stamp = pkt.stamp;
    std::copy(pkt.data.begin(), pkt.data.end(), std::back_inserter(nebula_pkt_ptr->data));

    decoder_wrapper_->process_cloud_packet(std::move(nebula_pkt_ptr));
  }
}

void RobosenseRosWrapper::receive_info_packet_callback(std::vector<uint8_t> & packet)
{
  if (!sensor_cfg_ptr_ || !info_driver_) {
    throw std::runtime_error(
      "Wrapper already receiving packets despite not being fully initialized yet.");
  }

  if (info_packets_pub_) {
    robosense_msgs::msg::RobosenseInfoPacket info_packet{};
    if (packet.size() != info_packet.packet.data.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Could not publish info packet: size unsupported");
    } else {
      std::copy(packet.cbegin(), packet.cend(), info_packet.packet.data.begin());
      info_packet.packet.stamp = now();
      info_packets_pub_->publish(info_packet);
    }
  }

  auto status = info_driver_->decode_info_packet(packet);

  if (status != nebula::Status::OK) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *get_clock(), 1000, "Could not decode info packet: " << status);
    return;
  }

  if (!decoder_wrapper_) {
    auto new_cfg = *sensor_cfg_ptr_;
    new_cfg.return_mode = info_driver_->get_return_mode();
    new_cfg.use_sensor_time = info_driver_->get_sync_status();
    auto calib = info_driver_->get_sensor_calibration();
    calib.create_corrected_channels();

    auto new_cfg_ptr =
      std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(new_cfg);
    status = validate_and_set_config(new_cfg_ptr);

    if (status != nebula::Status::OK) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Invalid config from sensor (" << status << "): " << new_cfg);
      return;
    }

    auto calib_ptr =
      std::make_shared<const nebula::drivers::RobosenseCalibrationConfiguration>(std::move(calib));
    decoder_wrapper_.emplace(
      this, hw_interface_wrapper_ ? hw_interface_wrapper_->hw_interface() : nullptr,
      sensor_cfg_ptr_, calib_ptr);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Initialized decoder wrapper: " << decoder_wrapper_->status());
  }

  if (!hw_monitor_wrapper_) {
    return;
  }

  hw_monitor_wrapper_->diagnostics_callback(info_driver_->get_sensor_info());
}

Status RobosenseRosWrapper::get_status()
{
  return wrapper_status_;
}

Status RobosenseRosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  auto info_status = hw_interface_wrapper_->hw_interface()->info_interface_start();

  if (info_status != Status::OK) {
    return info_status;
  }

  return hw_interface_wrapper_->hw_interface()->sensor_interface_start();
}

rcl_interfaces::msg::SetParametersResult RobosenseRosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::RobosenseSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string _return_mode = "";
  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold);

  // Currently, none of the wrappers have writeable parameters, so their update logic is not
  // implemented

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::return_mode_from_string(_return_mode);

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::RobosenseSensorConfiguration>(new_cfg);
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

void RobosenseRosWrapper::receive_cloud_packet_callback(std::vector<uint8_t> & packet)
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

  decoder_wrapper_->process_cloud_packet(std::move(msg_ptr));
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseRosWrapper)
}  // namespace nebula::ros
