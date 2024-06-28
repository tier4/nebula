// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hesai_ros_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>

#include <filesystem>
#include <memory>
#include <sstream>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula
{
namespace ros
{
HesaiRosWrapper::HesaiRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr),
  packet_queue_(3000),
  hw_interface_wrapper_(),
  hw_monitor_wrapper_(),
  decoder_wrapper_()
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  wrapper_status_ = DeclareAndGetSensorConfigParams();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << wrapper_status_).str());
  }

  RCLCPP_INFO_STREAM(get_logger(), "SensorConfig:" << *sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->HwInterface(), sensor_cfg_ptr_);
  }

  auto calibration_result = GetCalibrationData(sensor_cfg_ptr_->calibration_path);
  if (!calibration_result.has_value()) {
    throw std::runtime_error(
      (std::stringstream() << "No valid calibration found: " << calibration_result.error()).str());
  }

  decoder_wrapper_.emplace(this, sensor_cfg_ptr_, calibration_result.value());

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  decoder_thread_ = std::thread([this]() {
    while (true) {
      decoder_wrapper_->ProcessCloudPacket(packet_queue_.pop());
    }
  });

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

  std::string calibration_parameter_name = getCalibrationParameterName(config.sensor_model);
  config.calibration_path =
    declare_parameter<std::string>(calibration_parameter_name, param_read_write());

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

    packet_queue_.push(std::move(nebula_pkt_ptr));
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

  std::string _return_mode{};
  std::string calibration_parameter_name =
    getCalibrationParameterName(sensor_cfg_ptr_->sensor_model);

  bool got_any =
    get_param(p, "return_mode", _return_mode) | get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) | get_param(p, "min_range", new_cfg.min_range) |
    get_param(p, "max_range", new_cfg.max_range) |
    get_param(p, "rotation_speed", new_cfg.rotation_speed) |
    get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
    get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold) |
    get_param(p, calibration_parameter_name, new_cfg.calibration_path);

  // Currently, all of the sub-wrappers read-only parameters, so they do not be queried for updates

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (_return_mode.length() > 0)
    new_cfg.return_mode = nebula::drivers::ReturnModeFromString(_return_mode);

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
    auto get_calibration_result = GetCalibrationData(new_cfg.calibration_path, true);
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
  auto status = ValidateAndSetConfig(new_cfg_ptr);
  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = (std::stringstream() << "Invalid configuration: " << status).str();
    return result;
  }

  // Set calibration (if any) only once all parameters have been validated
  if (new_calibration_ptr) {
    decoder_wrapper_->OnCalibrationChange(new_calibration_ptr);
    RCLCPP_INFO_STREAM(get_logger(), "Changed calibration to '" << new_cfg.calibration_path << "'");
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

  if (!packet_queue_.try_push(std::move(msg_ptr))) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packet(s) dropped");
  }
}

std::string HesaiRosWrapper::getCalibrationParameterName(drivers::SensorModel model) const
{
  if (model == drivers::SensorModel::HESAI_PANDARAT128) {
    return "correction_file";
  }

  return "calibration_file";
}

HesaiRosWrapper::get_calibration_result_t HesaiRosWrapper::GetCalibrationData(
  const std::string & calibration_file_path, bool ignore_others)
{
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calib;
  const auto & logger = get_logger();

  if (sensor_cfg_ptr_->sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    calib = std::make_shared<drivers::HesaiCorrection>();
  } else {
    calib = std::make_shared<drivers::HesaiCalibrationConfiguration>();
  }

  std::string calibration_file_path_from_sensor;

  {
    int ext_pos = calibration_file_path.find_last_of('.');
    calibration_file_path_from_sensor = calibration_file_path.substr(0, ext_pos);
    calibration_file_path_from_sensor += "_from_sensor_" + sensor_cfg_ptr_->sensor_ip;
    calibration_file_path_from_sensor +=
      calibration_file_path.substr(ext_pos, calibration_file_path.size() - ext_pos);
  }

  // If a sensor is connected, try to download and save its calibration data
  if (!ignore_others && launch_hw_) {
    try {
      auto raw_data = hw_interface_wrapper_->HwInterface()->GetLidarCalibrationBytes();
      RCLCPP_INFO(logger, "Downloaded calibration data from sensor.");
      auto status = calib->SaveToFileFromBytes(calibration_file_path_from_sensor, raw_data);
      if (status != Status::OK) {
        RCLCPP_ERROR_STREAM(logger, "Could not save calibration data: " << status);
      } else {
        RCLCPP_INFO_STREAM(
          logger, "Saved downloaded data to " << calibration_file_path_from_sensor);
      }
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger, "Could not download calibration data: " << e.what());
    }
  }

  // If saved calibration data from a sensor exists (either just downloaded above, or previously),
  // try to load it
  if (!ignore_others && std::filesystem::exists(calibration_file_path_from_sensor)) {
    auto status = calib->LoadFromFile(calibration_file_path_from_sensor);
    if (status == Status::OK) {
      calib->calibration_file = calibration_file_path_from_sensor;
      return calib;
    }

    RCLCPP_ERROR_STREAM(logger, "Could not load downloaded calibration data: " << status);
  } else if (!ignore_others) {
    RCLCPP_ERROR(logger, "No downloaded calibration data found.");
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
  auto status = calib->LoadFromFile(calibration_file_path);
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
}  // namespace ros
}  // namespace nebula
