#include "nebula_ros/hesai/hesai_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiHwInterfaceRosWrapper::HesaiHwInterfaceRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_hw_interface_ros_wrapper", options), hw_interface_()
{
  if (mtx_config_.try_lock()) {
    interface_status_ = GetParameters(sensor_configuration_);
    mtx_config_.unlock();
  }
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  if (this->setup_sensor) {
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
  }
#if not defined(TEST_PCAP)
  hw_interface_.InitializeTcpDriver(this->setup_sensor);

  std::vector<std::thread> thread_pool{};
  thread_pool.emplace_back([this] {
    hw_interface_.GetInventory(  // ios,
      [this](HesaiInventory & result) {
        std::cout << result << std::endl;
        hw_interface_.SetTargetModel(result.model);
      });
  });
  for (std::thread & th : thread_pool) {
    th.join();
  }
  if (this->setup_sensor) {
    hw_interface_.CheckAndSetConfig();
    updateParameters();
  }
#endif

  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  pandar_scan_pub_ =
    this->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());

#if not defined(TEST_PCAP)
  if (this->setup_sensor) {
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&HesaiHwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));
  }
#endif

#ifdef WITH_DEBUG_STDOUT_HesaiHwInterfaceRosWrapper
  if (false) {
    std::vector<std::thread> thread_pool{};
    thread_pool.emplace_back([this] { hw_interface_.SetStandbyMode(0); });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetLidarCalib(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetPtpDiagStatus(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetPtpDiagPort(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetPtpDiagTime(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetPtpDiagGrandmaster(ios);
    });
    //  thread_pool.emplace_back([&hw_interface_]{
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetInventory(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetConfig(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetLidarStatus(ios);
    });
    thread_pool.emplace_back([this] { hw_interface_.SetStandbyMode(1); });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetPtpConfig(ios);
    });
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetLidarRange(ios);
    });
    for (std::thread & th : thread_pool) {
      //    hw_interface_.IOServiceRun();
      th.join();
    }
  }
  if (false) {
    std::vector<std::thread> thread_pool{};
    thread_pool.emplace_back([this] {
      auto ios = std::make_shared<boost::asio::io_service>();
      std::cout << "GetLidarCalib" << std::endl;
      hw_interface_.GetLidarCalib(ios);
    });
    for (std::thread & th : thread_pool) {
      th.join();
    }
  }
#endif

  StreamStart();
}

HesaiHwInterfaceRosWrapper::~HesaiHwInterfaceRosWrapper() {
  RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
  hw_interface_.FinalizeTcpDriver();
}

Status HesaiHwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.CloudInterfaceStart();
  }
  return interface_status_;
}

Status HesaiHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceRosWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status HesaiHwInterfaceRosWrapper::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringHesai(
      this->get_parameter("return_mode").as_string(), sensor_configuration.sensor_model);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "pandar", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = this->get_parameter("packet_mtu_size").as_int();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::HESAI_PANDARAT128) {
      descriptor.additional_constraints = "200, 300, 400, 500";
      range.set__from_value(200).set__to_value(500).set__step(100);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("rotation_speed", 200, descriptor);
    } else {
      descriptor.additional_constraints = "300, 600, 1200";
      range.set__from_value(300).set__to_value(1200).set__step(300);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
    sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0.01).set__to_value(0.5).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("dual_return_distance_threshold", 0.1, descriptor);
    sensor_configuration.dual_return_distance_threshold =
      this->get_parameter("dual_return_distance_threshold").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("setup_sensor", true, descriptor);
    this->setup_sensor = this->get_parameter("setup_sensor").as_bool();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {  // ||
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  scan_buffer->header.stamp = scan_buffer->packets.front().stamp;
  pandar_scan_pub_->publish(*scan_buffer);
}

rcl_interfaces::msg::SetParametersResult HesaiHwInterfaceRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(this->get_logger(), p);
  RCLCPP_DEBUG_STREAM(this->get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::HesaiSensorConfiguration new_param{sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;
  std::string return_mode_str;
  if (
    get_param(p, "sensor_model", sensor_model_str) ||
    get_param(p, "return_mode", return_mode_str) || get_param(p, "host_ip", new_param.host_ip) ||
    get_param(p, "sensor_ip", new_param.sensor_ip) ||
    get_param(p, "frame_id", new_param.frame_id) ||
    get_param(p, "data_port", new_param.data_port) ||
    get_param(p, "gnss_port", new_param.gnss_port) ||
    get_param(p, "scan_phase", new_param.scan_phase) ||
    get_param(p, "packet_mtu_size", new_param.packet_mtu_size) ||
    get_param(p, "rotation_speed", new_param.rotation_speed) ||
    get_param(p, "cloud_min_angle", new_param.cloud_min_angle) ||
    get_param(p, "cloud_max_angle", new_param.cloud_max_angle) ||
    get_param(p, "dual_return_distance_threshold", new_param.dual_return_distance_threshold)) {
    if (0 < sensor_model_str.length())
      new_param.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);
    if (0 < return_mode_str.length())
      new_param.return_mode = nebula::drivers::ReturnModeFromString(return_mode_str);

    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
    hw_interface_.CheckAndSetConfig();
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");

  return *result;
}

std::vector<rcl_interfaces::msg::SetParametersResult> HesaiHwInterfaceRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  std::ostringstream os_return_mode;
  os_return_mode << sensor_configuration_.return_mode;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");
  auto results = set_parameters(
    {rclcpp::Parameter("sensor_model", os_sensor_model.str()),
     rclcpp::Parameter("return_mode", os_return_mode.str()),
     rclcpp::Parameter("host_ip", sensor_configuration_.host_ip),
     rclcpp::Parameter("sensor_ip", sensor_configuration_.sensor_ip),
     rclcpp::Parameter("frame_id", sensor_configuration_.frame_id),
     rclcpp::Parameter("data_port", sensor_configuration_.data_port),
     rclcpp::Parameter("gnss_port", sensor_configuration_.gnss_port),
     rclcpp::Parameter("scan_phase", sensor_configuration_.scan_phase),
     rclcpp::Parameter("packet_mtu_size", sensor_configuration_.packet_mtu_size),
     rclcpp::Parameter("rotation_speed", sensor_configuration_.rotation_speed),
     rclcpp::Parameter("cloud_min_angle", sensor_configuration_.cloud_min_angle),
     rclcpp::Parameter("cloud_max_angle", sensor_configuration_.cloud_max_angle),
     rclcpp::Parameter(
       "dual_return_distance_threshold", sensor_configuration_.dual_return_distance_threshold)});
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiHwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
