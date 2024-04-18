#include "nebula_ros/hesai/hesai_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiRosWrapper::HesaiRosWrapper(const rclcpp::NodeOptions& options)
  : rclcpp::Node("hesai_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true))
  , wrapper_status_(Status::NOT_INITIALIZED)
  , sensor_cfg_ptr_(nullptr)
  , packet_queue_(3000)
  , hw_interface_wrapper_()
  , hw_monitor_wrapper_()
  , decoder_wrapper_()
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = DeclareAndGetSensorConfigParams();
  wrapper_status_ = DeclareAndGetWrapperParams();

  if (launch_hw_)
  {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, hw_interface_wrapper_->HwInterface(), sensor_cfg_ptr_);
  }

  decoder_wrapper_.emplace(this, hw_interface_wrapper_ ? hw_interface_wrapper_->HwInterface() : nullptr,
                           sensor_cfg_ptr_);

  RCLCPP_INFO_STREAM(get_logger(), "SensorConfig:" << *sensor_cfg_ptr_);
  set_param_res_ =
      add_on_set_parameters_callback(std::bind(&HesaiRosWrapper::paramCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Starting stream");

  decoder_thread_ = std::thread([this]() {
    while (true)
    {
      decoder_wrapper_->ProcessCloudPacket(std::move(packet_queue_.pop()));
    }
  });

  if (launch_hw_)
  {
    StreamStart();
    hw_interface_wrapper_->HwInterface()->RegisterScanCallback(
        std::bind(&HesaiRosWrapper::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  }
  else
  {
    packets_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
        "pandar_packets", rclcpp::SensorDataQoS(),
        std::bind(&HesaiRosWrapper::ReceiveScanMessageCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(get_logger(), "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }
}

nebula::Status HesaiRosWrapper::DeclareAndGetSensorConfigParams()
{
  nebula::drivers::HesaiSensorConfiguration sensor_configuration;

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
        nebula::drivers::SensorModelFromString(get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringHesai(
        get_parameter("return_mode").as_string(), sensor_configuration.sensor_model);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("frame_id", "pandar", descriptor);
    sensor_configuration.frame_id = get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = { range };
    declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<double>("min_range", 0.3, descriptor);
    sensor_configuration.min_range = get_parameter("min_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<double>("max_range", 300., descriptor);
    sensor_configuration.max_range = get_parameter("max_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = get_parameter("packet_mtu_size").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    RCLCPP_DEBUG_STREAM(get_logger(), sensor_configuration.sensor_model);
    if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::HESAI_PANDARAT128)
    {
      descriptor.additional_constraints = "200, 300, 400, 500";
      // range.set__from_value(200).set__to_value(500).set__step(100);
      // descriptor.integer_range = {range}; //todo
      declare_parameter<uint16_t>("rotation_speed", 200, descriptor);
    }
    else
    {
      descriptor.additional_constraints = "300, 600, 1200";
      // range.set__from_value(300).set__to_value(1200).set__step(300);
      // descriptor.integer_range = {range}; //todo
      declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = { range };
    declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = { range };
    declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
    sensor_configuration.cloud_max_angle = get_parameter("cloud_max_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0.01).set__to_value(0.5).set__step(0.01);
    descriptor.floating_point_range = { range };
    declare_parameter<double>("dual_return_distance_threshold", 0.1, descriptor);
    sensor_configuration.dual_return_distance_threshold = get_parameter("dual_return_distance_threshold").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("ptp_profile", "");
    sensor_configuration.ptp_profile = nebula::drivers::PtpProfileFromString(get_parameter("ptp_profile").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("ptp_transport_type", "");
    sensor_configuration.ptp_transport_type =
        nebula::drivers::PtpTransportTypeFromString(get_parameter("ptp_transport_type").as_string());
    if (static_cast<int>(sensor_configuration.ptp_profile) > 0)
    {
      sensor_configuration.ptp_transport_type = nebula::drivers::PtpTransportType::L2;
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<std::string>("ptp_switch_type", "");
    sensor_configuration.ptp_switch_type =
        nebula::drivers::PtpSwitchTypeFromString(get_parameter("ptp_switch_type").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(127).set__step(1);
    descriptor.integer_range = { range };
    declare_parameter<uint8_t>("ptp_domain", 0, descriptor);
    sensor_configuration.ptp_domain = get_parameter("ptp_domain").as_int();
  }

  if (sensor_configuration.ptp_profile == nebula::drivers::PtpProfile::PROFILE_UNKNOWN)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid PTP Profile Provided. Please use '1588v2', '802.1as' or 'automotive'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (sensor_configuration.ptp_transport_type == nebula::drivers::PtpTransportType::UNKNOWN_TRANSPORT)
  {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Invalid PTP Transport Provided. Please use 'udp' or 'l2', 'udp' is only available when "
                        "using the '1588v2' PTP Profile");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (sensor_configuration.ptp_switch_type == nebula::drivers::PtpSwitchType::UNKNOWN_SWITCH)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid PTP Switch Type Provided. Please use 'tsn' or 'non_tsn'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN)
  {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN)
  {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360)
  {
    return Status::SENSOR_CONFIG_ERROR;
  }

  auto new_cfg_ptr_ = std::make_shared<nebula::drivers::HesaiSensorConfiguration>(sensor_configuration);
  sensor_cfg_ptr_.swap(new_cfg_ptr_);
  return Status::OK;
}

void HesaiRosWrapper::ReceiveScanMessageCallback(std::unique_ptr<pandar_msgs::msg::PandarScan> scan_msg)
{
  if (hw_interface_wrapper_)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Ignoring received PandarScan. Launch with launch_hw:=false to enable PandarScan replay.");
    return;
  }

  for (auto& pkt : scan_msg->packets)
  {
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

Status HesaiRosWrapper::DeclareAndGetWrapperParams()
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    declare_parameter<bool>("launch_hw", "", descriptor);
    launch_hw_ = get_parameter("launch_hw").as_bool();
  }

  return Status::OK;
}

HesaiRosWrapper::~HesaiRosWrapper()
{
}

Status HesaiRosWrapper::StreamStart()
{
  if (!hw_interface_wrapper_)
  {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->Status() != Status::OK)
  {
    return hw_interface_wrapper_->Status();
  }

  return hw_interface_wrapper_->HwInterface()->SensorInterfaceStart();
}

rcl_interfaces::msg::SetParametersResult HesaiRosWrapper::paramCallback(const std::vector<rclcpp::Parameter>& p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(get_logger(), p);
  RCLCPP_DEBUG_STREAM(get_logger(), *sensor_cfg_ptr_);

  drivers::HesaiSensorConfiguration new_cfg(*sensor_cfg_ptr_);

  std::string sensor_model_str;
  std::string return_mode_str;
  if (get_param(p, "sensor_model", sensor_model_str) | get_param(p, "return_mode", return_mode_str) |
      get_param(p, "host_ip", new_cfg.host_ip) | get_param(p, "sensor_ip", new_cfg.sensor_ip) |
      get_param(p, "frame_id", new_cfg.frame_id) | get_param(p, "data_port", new_cfg.data_port) |
      get_param(p, "gnss_port", new_cfg.gnss_port) | get_param(p, "scan_phase", new_cfg.scan_phase) |
      get_param(p, "packet_mtu_size", new_cfg.packet_mtu_size) |
      get_param(p, "rotation_speed", new_cfg.rotation_speed) |
      get_param(p, "cloud_min_angle", new_cfg.cloud_min_angle) |
      get_param(p, "cloud_max_angle", new_cfg.cloud_max_angle) |
      get_param(p, "dual_return_distance_threshold", new_cfg.dual_return_distance_threshold))
  {
    if (0 < sensor_model_str.length())
      new_cfg.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);
    if (0 < return_mode_str.length())
      new_cfg.return_mode = nebula::drivers::ReturnModeFromString(return_mode_str);

    auto new_cfg_ptr = std::make_shared<nebula::drivers::HesaiSensorConfiguration>(new_cfg);
    sensor_cfg_ptr_.swap(new_cfg_ptr);
    RCLCPP_INFO_STREAM(get_logger(), "Update sensor_configuration");
    RCLCPP_DEBUG_STREAM(get_logger(), "hw_interface_.SetSensorConfiguration");
    hw_interface_wrapper_->HwInterface()->SetSensorConfiguration(
        std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));
    hw_interface_wrapper_->HwInterface()->CheckAndSetConfig();
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(get_logger(), "add_on_set_parameters_callback success");

  return *result;
}

std::vector<rcl_interfaces::msg::SetParametersResult> HesaiRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_cfg_ptr_->sensor_model;
  std::ostringstream os_return_mode;
  os_return_mode << sensor_cfg_ptr_->return_mode;
  RCLCPP_INFO_STREAM(get_logger(), "set_parameters");
  auto results = set_parameters(
      { rclcpp::Parameter("sensor_model", os_sensor_model.str()),
        rclcpp::Parameter("return_mode", os_return_mode.str()), rclcpp::Parameter("host_ip", sensor_cfg_ptr_->host_ip),
        rclcpp::Parameter("sensor_ip", sensor_cfg_ptr_->sensor_ip),
        rclcpp::Parameter("frame_id", sensor_cfg_ptr_->frame_id),
        rclcpp::Parameter("data_port", sensor_cfg_ptr_->data_port),
        rclcpp::Parameter("gnss_port", sensor_cfg_ptr_->gnss_port),
        rclcpp::Parameter("scan_phase", sensor_cfg_ptr_->scan_phase),
        rclcpp::Parameter("packet_mtu_size", sensor_cfg_ptr_->packet_mtu_size),
        rclcpp::Parameter("rotation_speed", sensor_cfg_ptr_->rotation_speed),
        rclcpp::Parameter("cloud_min_angle", sensor_cfg_ptr_->cloud_min_angle),
        rclcpp::Parameter("cloud_max_angle", sensor_cfg_ptr_->cloud_max_angle),
        rclcpp::Parameter("dual_return_distance_threshold", sensor_cfg_ptr_->dual_return_distance_threshold) });
  RCLCPP_DEBUG_STREAM(get_logger(), "updateParameters end");
  return results;
}

void HesaiRosWrapper::ReceiveCloudPacketCallback(std::vector<uint8_t>& packet)
{
  if (!decoder_wrapper_ || decoder_wrapper_->Status() != Status::OK)
  {
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);

  if (!packet_queue_.try_push(std::move(msg_ptr)))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packet(s) dropped");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiRosWrapper)
}  // namespace ros
}  // namespace nebula
