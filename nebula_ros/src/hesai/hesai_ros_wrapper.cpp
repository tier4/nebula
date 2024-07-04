// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hesai_ros_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/util/string_conversions.hpp>
#include <nebula_decoders/nebula_decoders_common/angles.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
HesaiRosWrapper::HesaiRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr),
  hw_interface_wrapper_(),
  hw_monitor_wrapper_(),
  decoder_wrapper_()
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());
  bool use_udp_only = declare_parameter<bool>("udp_only", param_read_only());

  if (use_udp_only) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "UDP-only mode is enabled. Settings checks, synchronization, and diagnostics publishing are "
      "disabled.");
  }

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_, use_udp_only);
    if (!use_udp_only) {  // hardware monitor requires TCP connection
      hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->hw_interface(), sensor_cfg_ptr_);
    }
  }

  bool force_load_caibration_from_file =
    use_udp_only;  // Downloading from device requires TCP connection
  auto calibration_result =
    get_calibration_data(sensor_cfg_ptr_->calibration_path, force_load_caibration_from_file);
  if (!calibration_result.has_value()) {
    throw std::runtime_error(
      "No valid calibration found: " + util::to_string(calibration_result.error()));
  }

  bool lidar_range_supported =
    sensor_cfg_ptr_->sensor_model != drivers::SensorModel::HESAI_PANDARAT128 &&
    sensor_cfg_ptr_->sensor_model != drivers::SensorModel::HESAI_PANDAR64;

  if (hw_interface_wrapper_ && !use_udp_only && lidar_range_supported) {
    auto status =
      hw_interface_wrapper_->hw_interface()->check_and_set_lidar_range(*calibration_result.value());
    if (status != Status::OK) {
      throw std::runtime_error("Could not set sensor FoV: " + util::to_string(status));
    }
  }

  decoder_wrapper_.emplace(this, sensor_cfg_ptr_, calibration_result.value(), launch_hw_);

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  if (launch_hw_) {
    hw_interface_wrapper_->hw_interface()->register_scan_callback(
      std::bind(&HesaiRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS(),
      std::bind(&HesaiRosWrapper::receive_scan_message_callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&HesaiRosWrapper::on_parameter_change, this, std::placeholders::_1));
}

nebula::Status HesaiRosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::HesaiSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::sensor_model_from_string(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::return_mode_from_string_hesai(_return_mode, config.sensor_model);

  config.host_ip = declare_parameter<std::string>("host_ip", param_read_only());
  config.sensor_ip = declare_parameter<std::string>("sensor_ip", param_read_only());
  config.multicast_ip = declare_parameter<std::string>("multicast_ip", param_read_only());
  config.data_port = declare_parameter<uint16_t>("data_port", param_read_only());
  config.gnss_port = declare_parameter<uint16_t>("gnss_port", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_write());

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    descriptor.floating_point_range = float_range(0, 360, 0.01);
    descriptor.description =
      "Angle at which to cut the scan. Cannot be equal to the start angle in a non-360 deg "
      "FoV. Choose the end angle instead.";
    config.cut_angle = declare_parameter<double>("cut_angle", descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_write();
    if (config.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
      descriptor.integer_range = int_range(30, 150, 1);
    } else {
      descriptor.integer_range = int_range(0, 359, 1);
    }
    config.sync_angle = declare_parameter<uint16_t>("sync_angle", descriptor);
  }

  config.min_range = declare_parameter<double>("min_range", param_read_write());
  config.max_range = declare_parameter<double>("max_range", param_read_write());
  config.packet_mtu_size = declare_parameter<uint16_t>("packet_mtu_size", param_read_only());

  config.hires_mode = false;
  if (
    config.sensor_model == drivers::SensorModel::HESAI_PANDAR128_E4X ||
    config.sensor_model == drivers::SensorModel::HESAI_PANDAR128_E3X) {
    config.hires_mode = this->declare_parameter<bool>("hires_mode", param_read_write());
  }

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

  std::string calibration_parameter_name = get_calibration_parameter_name(config.sensor_model);
  config.calibration_path =
    declare_parameter<std::string>(calibration_parameter_name, param_read_write());

  auto ptp_profile = declare_parameter<std::string>("ptp_profile", param_read_only());
  config.ptp_profile = drivers::ptp_profile_from_string(ptp_profile);

  auto ptp_transport = declare_parameter<std::string>("ptp_transport_type", param_read_only());
  config.ptp_transport_type = drivers::ptp_transport_type_from_string(ptp_transport);

  if (
    config.ptp_transport_type != drivers::PtpTransportType::L2 &&
    config.ptp_profile != drivers::PtpProfile::IEEE_1588v2 &&
    config.ptp_profile != drivers::PtpProfile::UNKNOWN_PROFILE) {
    RCLCPP_WARN_STREAM(
      get_logger(), "PTP transport was set to '" << ptp_transport << "' but PTP profile '"
                                                 << ptp_profile
                                                 << "' only supports 'L2'. Setting it to 'L2'.");
    config.ptp_transport_type = drivers::PtpTransportType::L2;
    set_parameter(rclcpp::Parameter("ptp_transport_type", "L2"));
  }

  auto ptp_switch = declare_parameter<std::string>("ptp_switch_type", param_read_only());
  config.ptp_switch_type = drivers::ptp_switch_type_from_string(ptp_switch);

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.integer_range = int_range(0, 127, 1);
    config.ptp_domain = declare_parameter<uint8_t>("ptp_domain", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.integer_range = int_range(1, 100, 1);
    config.ptp_lock_threshold = declare_parameter<uint8_t>("ptp_lock_threshold", descriptor);
  }

  {
    auto downsample_mask_path =
      declare_parameter<std::string>("point_filters.downsample_mask.path", "", param_read_write());
    if (downsample_mask_path.empty()) {
      config.downsample_mask_path = std::nullopt;
    } else {
      config.downsample_mask_path = downsample_mask_path;
    }
  }

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::HesaiSensorConfiguration>(config);
  return validate_and_set_config(new_cfg_ptr);
}

Status HesaiRosWrapper::validate_and_set_config(
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
  if (new_config->host_ip == "255.255.255.255") {
    RCLCPP_ERROR(
      get_logger(),
      "Due to potential network performance issues when using IP broadcast for sensor data, Nebula "
      "disallows use of the broadcast IP. Please specify the concrete host IP instead.");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_profile == nebula::drivers::PtpProfile::UNKNOWN_PROFILE) {
    RCLCPP_ERROR(
      get_logger(), "Invalid PTP Profile Provided. Please use '1588v2', '802.1as' or 'automotive'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_transport_type == nebula::drivers::PtpTransportType::UNKNOWN_TRANSPORT) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid PTP Transport Provided. Please use 'udp' or 'l2', 'udp' is only available when "
      "using the '1588v2' PTP Profile");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (new_config->ptp_switch_type == nebula::drivers::PtpSwitchType::UNKNOWN_SWITCH) {
    RCLCPP_ERROR(get_logger(), "Invalid PTP Switch Type Provided. Please use 'tsn' or 'non_tsn'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (!drivers::angle_is_between<double>(
        new_config->cloud_min_angle, new_config->cloud_max_angle, new_config->cut_angle)) {
    RCLCPP_ERROR(get_logger(), "Cannot cut scan outside of the FoV.");
    return Status::SENSOR_CONFIG_ERROR;
  }

  bool fov_is_360 = new_config->cloud_min_angle == 0 && new_config->cloud_max_angle == 360;
  if (!fov_is_360 && new_config->cut_angle == new_config->cloud_min_angle) {
    RCLCPP_ERROR(
      get_logger(), "Cannot cut scan right at the start of the FoV. Cut at the end instead.");
    return Status::SENSOR_CONFIG_ERROR;
  }

  // Handling cutting at 360deg (as opposed to 0deg) for a full 360deg FoV requires a special case
  // in the cutting logic. Thus, require the user to cut at 0deg.
  if (fov_is_360 && new_config->cut_angle == 360) {
    RCLCPP_ERROR(get_logger(), "Cannot cut a 360deg FoV at 360deg. Cut at 0deg instead.");
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (
    new_config->downsample_mask_path &&
    !std::filesystem::exists(new_config->downsample_mask_path.value())) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Downsample mask not found: " << new_config->downsample_mask_path.value());
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

void HesaiRosWrapper::receive_scan_message_callback(
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

    decoder_wrapper_->process_cloud_packet(std::move(nebula_pkt_ptr));
  }
}

Status HesaiRosWrapper::get_status()
{
  return wrapper_status_;
}

Status HesaiRosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  return hw_interface_wrapper_->hw_interface()->sensor_interface_start();
}

rcl_interfaces::msg::SetParametersResult HesaiRosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::HesaiSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string return_mode{};
  std::string calibration_parameter_name =
    get_calibration_parameter_name(sensor_cfg_ptr_->sensor_model);
  std::string downsample_mask_path = new_cfg.downsample_mask_path.value_or("");

  bool got_any =
    get_param(p, "return_mode", return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "sync_angle", new_cfg.sync_angle) | get_param(p, "cut_angle", new_cfg.cut_angle) |
    get_param(p, "min_range", new_cfg.min_range) | get_param(p, "max_range", new_cfg.max_range) |
    get_param(p, "rotation_speed", new_cfg.rotation_speed) |
    get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold) |
    get_param(p, "hires_mode", new_cfg.hires_mode) |
    get_param(p, calibration_parameter_name, new_cfg.calibration_path) |
    get_param(p, "point_filters.downsample_mask.path", downsample_mask_path);

  // Currently, all of the sub-wrappers read-only parameters, so they do not be queried for updates

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (!return_mode.empty()) {
    new_cfg.return_mode =
      nebula::drivers::return_mode_from_string_hesai(return_mode, sensor_cfg_ptr_->sensor_model);
  }

  if (!downsample_mask_path.empty()) {
    new_cfg.downsample_mask_path = downsample_mask_path;
  } else {
    new_cfg.downsample_mask_path = std::nullopt;
  }

  // ////////////////////////////////////////
  // Get and validate new calibration, if any
  // ////////////////////////////////////////

  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> new_calibration_ptr{};

  bool new_calibration_set = new_cfg.calibration_path != sensor_cfg_ptr_->calibration_path;
  if (new_calibration_set) {
    // Calibration paths set during runtime are always queried from the filesystem, never fetched
    // from the sensor.
    if (!std::filesystem::exists(new_cfg.calibration_path)) {
      auto result = SetParametersResult();
      result.successful = false;
      result.reason =
        "The given calibration path does not exist, ignoring: '" + new_cfg.calibration_path + "'";
      return result;
    }

    // Fail early and do not set the new config if getting calibration data failed.
    auto get_calibration_result = get_calibration_data(new_cfg.calibration_path, true);
    if (!get_calibration_result.has_value()) {
      auto result = SetParametersResult();
      result.successful = false;
      std::stringstream ss{};
      ss << "Could not change calibration file to '" << new_cfg.calibration_path << "': ";
      ss << get_calibration_result.error();
      result.reason = ss.str();
      return result;
    }

    new_calibration_ptr = get_calibration_result.value();
  }

  auto new_cfg_ptr = std::make_shared<const nebula::drivers::HesaiSensorConfiguration>(new_cfg);
  auto status = validate_and_set_config(new_cfg_ptr);
  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  // Set calibration (if any) only once all parameters have been validated
  if (new_calibration_ptr) {
    decoder_wrapper_->on_calibration_change(new_calibration_ptr);
    RCLCPP_INFO_STREAM(get_logger(), "Changed calibration to '" << new_cfg.calibration_path << "'");
  }

  if (
    new_calibration_ptr && hw_interface_wrapper_ &&
    sensor_cfg_ptr_->sensor_model != drivers::SensorModel::HESAI_PANDARAT128) {
    auto status =
      hw_interface_wrapper_->hw_interface()->check_and_set_lidar_range(*new_calibration_ptr);
    if (status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Sensor configuration updated, but setting hardware FoV failed: " << status);
    }
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void HesaiRosWrapper::receive_cloud_packet_callback(const std::vector<uint8_t> & packet)
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
  msg_ptr->data = packet;

  decoder_wrapper_->process_cloud_packet(std::move(msg_ptr));
}

std::string HesaiRosWrapper::get_calibration_parameter_name(drivers::SensorModel model) const
{
  if (model == drivers::SensorModel::HESAI_PANDARAT128) {
    return "correction_file";
  }

  return "calibration_file";
}

HesaiRosWrapper::get_calibration_result_t HesaiRosWrapper::get_calibration_data(
  const std::string & calibration_file_path, bool ignore_others)
{
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calib;
  const auto & logger = get_logger();

  if (sensor_cfg_ptr_->sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    calib = std::make_shared<drivers::HesaiCorrection>();
  } else {
    calib = std::make_shared<drivers::HesaiCalibrationConfiguration>();
  }

  std::filesystem::path calibration_from_sensor_path{calibration_file_path};
  calibration_from_sensor_path = calibration_from_sensor_path.replace_extension(
    "_from_sensor_" + sensor_cfg_ptr_->sensor_ip +
    calibration_from_sensor_path.extension().string());

  // If a sensor is connected, try to download and save its calibration data
  if (!ignore_others && launch_hw_) {
    try {
      auto raw_data = hw_interface_wrapper_->hw_interface()->get_lidar_calibration_bytes();
      RCLCPP_INFO(logger, "Downloaded calibration data from sensor.");
      auto status = calib->save_to_file_from_bytes(calibration_from_sensor_path, raw_data);
      if (status != Status::OK) {
        RCLCPP_ERROR_STREAM(logger, "Could not save calibration data: " << status);
      } else {
        RCLCPP_INFO_STREAM(logger, "Saved downloaded data to " << calibration_from_sensor_path);
      }
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger, "Could not download calibration data: " << e.what());
    }
  }

  // If saved calibration data from a sensor exists (either just downloaded above, or previously),
  // try to load it
  if (!ignore_others && std::filesystem::exists(calibration_from_sensor_path)) {
    auto status = calib->load_from_file(calibration_from_sensor_path);
    if (status == Status::OK) {
      calib->calibration_file = calibration_from_sensor_path;
      return calib;
    }

    RCLCPP_ERROR_STREAM(logger, "Could not load downloaded calibration data: " << status);
  } else if (!ignore_others) {
    RCLCPP_WARN(logger, "No downloaded calibration data found.");
  }

  if (!ignore_others) {
    RCLCPP_WARN(logger, "Falling back to generic calibration file.");
  }

  // If downloaded data did not exist or could not be loaded, fall back to a generic file.
  // If that file does not exist either, return an error code
  if (!std::filesystem::exists(calibration_file_path)) {
    RCLCPP_ERROR(logger, "No calibration data found.");
    return nebula::Status(Status::INVALID_CALIBRATION_FILE);
  }

  // Try to load the existing fallback calibration file. Return an error if this fails
  auto status = calib->load_from_file(calibration_file_path);
  if (status != Status::OK) {
    RCLCPP_ERROR_STREAM(
      logger, "Could not load calibration file at '" << calibration_file_path << "'");
    return status;
  }

  // Return the fallback calibration file
  calib->calibration_file = calibration_file_path;
  return calib;
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiRosWrapper)
}  // namespace nebula::ros
