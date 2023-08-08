#include "nebula_ros/hesai/hesai_hw_monitor_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiHwMonitorRosWrapper::HesaiHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_hw_monitor_ros_wrapper", options), hw_interface_(), diagnostics_updater_(this)
{
  cbg_r_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_m_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbg_m2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
  hw_interface_.InitializeTcpDriver();

  message_sep = ": ";
  not_supported_message = "Not supported";
  error_message = "Error";

  switch (sensor_cfg_ptr->sensor_model) {
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
      temperature_names.emplace_back("Bottom circuit board T1");
      temperature_names.emplace_back("Bottom circuit board T2");
      temperature_names.emplace_back("Laser emitting board RT_L1(Internal)");
      temperature_names.emplace_back("Laser emitting board RT_L2");
      temperature_names.emplace_back("Receiving board RT_R");
      temperature_names.emplace_back("Receiving board RT2");
      temperature_names.emplace_back("Top circuit RT3");
      temperature_names.emplace_back("Not used");
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
    case nebula::drivers::SensorModel::HESAI_PANDAR40M:
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E3X:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
    default:
      temperature_names.emplace_back("Bottom circuit RT1");
      temperature_names.emplace_back("Bottom circuit RT2");
      temperature_names.emplace_back("Internal Temperature");
      temperature_names.emplace_back("Laser emitting board RT1");
      temperature_names.emplace_back("Laser emitting board RT2");
      temperature_names.emplace_back("Receiving board RT1");
      temperature_names.emplace_back("Top circuit RT1");
      temperature_names.emplace_back("Top circuit RT2");
      break;
  }

  std::vector<std::thread> thread_pool{};
  thread_pool.emplace_back([this] {
    hw_interface_.GetInventory(  // ios,
      [this](HesaiInventory & result) {
        current_inventory.reset(new HesaiInventory(result));
        current_inventory_time.reset(new rclcpp::Time(this->get_clock()->now()));
        std::cout << "HesaiInventory" << std::endl;
        std::cout << result << std::endl;
        info_model = result.get_str_model();
        info_serial = std::string(result.sn.begin(), result.sn.end());
        hw_interface_.SetTargetModel(result.model);
        RCLCPP_INFO_STREAM(this->get_logger(), "Model:" << info_model);
        RCLCPP_INFO_STREAM(this->get_logger(), "Serial:" << info_serial);
        InitializeHesaiDiagnostics();
      });
  });
  for (std::thread & th : thread_pool) {
    th.join();
  }

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&HesaiHwMonitorRosWrapper::paramCallback, this, std::placeholders::_1));
}

Status HesaiHwMonitorRosWrapper::MonitorStart() { return interface_status_; }

Status HesaiHwMonitorRosWrapper::MonitorStop() { return Status::OK; }
Status HesaiHwMonitorRosWrapper::Shutdown() { return Status::OK; }

Status HesaiHwMonitorRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status HesaiHwMonitorRosWrapper::GetParameters(
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

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {  // ||
    return Status::SENSOR_CONFIG_ERROR;
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "milliseconds";
    this->declare_parameter<uint16_t>("diag_span", 1000, descriptor);
    diag_span_ = this->get_parameter("diag_span").as_int();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void HesaiHwMonitorRosWrapper::InitializeHesaiDiagnostics()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "InitializeHesaiDiagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model + ": " + info_serial;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(this->get_logger(), "hardware_id: " + hardware_id);

  diagnostics_updater_.add("hesai_status", this, &HesaiHwMonitorRosWrapper::HesaiCheckStatus);
  diagnostics_updater_.add("hesai_ptp", this, &HesaiHwMonitorRosWrapper::HesaiCheckPtp);
  diagnostics_updater_.add(
    "hesai_temperature", this, &HesaiHwMonitorRosWrapper::HesaiCheckTemperature);
  diagnostics_updater_.add("hesai_rpm", this, &HesaiHwMonitorRosWrapper::HesaiCheckRpm);

  current_status.reset();
  current_monitor.reset();
  current_status_time.reset(new rclcpp::Time(this->get_clock()->now()));
  current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto on_timer_status = [this] { OnHesaiStatusTimer(); };
  diagnostics_status_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_status)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_status),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_status_timer_, cbg_m_);

  if (hw_interface_.UseHttpGetLidarMonitor()) {
    diagnostics_updater_.add(
      "hesai_voltage", this, &HesaiHwMonitorRosWrapper::HesaiCheckVoltageHttp);
    auto on_timer_lidar_monitor = [this] { OnHesaiLidarMonitorTimerHttp(); };
    diagnostics_lidar_monitor_timer_ =
      std::make_shared<rclcpp::GenericTimer<decltype(on_timer_lidar_monitor)>>(
        this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_lidar_monitor),
        this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(diagnostics_lidar_monitor_timer_, cbg_m2_);
  } else {
    diagnostics_updater_.add("hesai_voltage", this, &HesaiHwMonitorRosWrapper::HesaiCheckVoltage);
    auto on_timer_lidar_monitor = [this] { OnHesaiLidarMonitorTimer(); };
    diagnostics_lidar_monitor_timer_ =
      std::make_shared<rclcpp::GenericTimer<decltype(on_timer_lidar_monitor)>>(
        this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_lidar_monitor),
        this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(diagnostics_lidar_monitor_timer_, cbg_m2_);
  }

  auto on_timer_update = [this] {
    RCLCPP_DEBUG_STREAM(get_logger(), "OnUpdateTimer");
    auto now = this->get_clock()->now();
    auto dif = (now - *current_status_time).seconds();

    RCLCPP_DEBUG_STREAM(get_logger(), "dif(status): " << dif);

    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(get_logger(), "STALE");
    } else {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(get_logger(), "OK");
    }
    dif = (now - *current_lidar_monitor_time).seconds();
    RCLCPP_DEBUG_STREAM(get_logger(), "dif(monitor): " << dif);
    if (diag_span_ * 2.0 < dif * 1000) {
      current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(get_logger(), "STALE");
    } else {
      current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(get_logger(), "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_update)>>(
    this->get_clock(), std::chrono::milliseconds(1000), std::move(on_timer_update),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, cbg_r_);

  RCLCPP_DEBUG_STREAM(get_logger(), "add_timer");
}

std::string HesaiHwMonitorRosWrapper::GetPtreeValue(
  boost::property_tree::ptree * pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return not_supported_message;
  }
}
std::string HesaiHwMonitorRosWrapper::GetFixedPrecisionString(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

rcl_interfaces::msg::SetParametersResult HesaiHwMonitorRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(get_logger(), p);
  RCLCPP_DEBUG_STREAM(get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::HesaiSensorConfiguration new_param{sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  uint16_t new_diag_span = 0;
  if (get_param(p, "diag_span", new_diag_span)) {
    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");
  return *result;
}

void HesaiHwMonitorRosWrapper::OnHesaiStatusTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiStatusTimer" << std::endl);
  try {
    auto ios = std::make_shared<boost::asio::io_service>();
    hw_interface_.GetLidarStatus(  // ios,
      [this](HesaiLidarStatus & result) {
        std::scoped_lock lock(mtx_status);
        current_status_time.reset(new rclcpp::Time(this->get_clock()->now()));
        current_status.reset(new HesaiLidarStatus(result));
      });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorRosWrapper::OnHesaiStatusTimer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorRosWrapper::OnHesaiStatusTimer(boost::system::system_error)"),
      error.what());
  }
}

void HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimerHttp()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimerHttp");
  try {
    hw_interface_.GetLidarMonitorAsyncHttp([this](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor);
      current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
      current_lidar_monitor_tree =
        std::make_unique<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
    });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimerHttp(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimerHttp(boost::system::system_error)"),
      error.what());
  }
}

void HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimer");
  try {
    auto ios = std::make_shared<boost::asio::io_service>();
    hw_interface_.GetLidarMonitor([this](HesaiLidarMonitor & result) {
      std::scoped_lock lock(mtx_lidar_monitor);
      current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
      current_monitor.reset(new HesaiLidarMonitor(result));
    });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorRosWrapper::OnHesaiLidarMonitorTimer(boost::system::system_error)"),
      error.what());
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    diagnostics.add("system_uptime", std::to_string(current_status->system_uptime));
    diagnostics.add("startup_times", std::to_string(current_status->startup_times));
    diagnostics.add("total_operation_time", std::to_string(current_status->total_operation_time));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckPtp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    auto gps_status = current_status->get_str_gps_pps_lock();
    auto gprmc_status = current_status->get_str_gps_gprmc_status();
    auto ptp_status = current_status->get_str_ptp_clock_status();
    std::transform(gps_status.cbegin(), gps_status.cend(), gps_status.begin(), toupper);
    std::transform(gprmc_status.cbegin(), gprmc_status.cend(), gprmc_status.begin(), toupper);
    std::transform(ptp_status.cbegin(), ptp_status.cend(), ptp_status.begin(), toupper);
    diagnostics.add("gps_pps_lock", gps_status);
    diagnostics.add("gps_gprmc_status", gprmc_status);
    diagnostics.add("ptp_clock_status", ptp_status);
    if (gps_status != "UNKNOWN") {
      msg.emplace_back("gps_pps_lock: " + gps_status);
    }
    if (gprmc_status != "UNKNOWN") {
      msg.emplace_back("gprmc_status: " + gprmc_status);
    }
    if (ptp_status != "UNKNOWN") {
      msg.emplace_back("ptp_status: " + ptp_status);
    }
    if (ptp_status == "FREE RUN" && gps_status == "UNKNOWN") {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    for (size_t i = 0; i < current_status->temperature.size(); i++) {
      diagnostics.add(
        temperature_names[i], GetFixedPrecisionString(current_status->temperature[i] * 0.01));
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    diagnostics.add("motor_speed", std::to_string(current_status->motor_speed));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckVoltageHttp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor);
  if (current_lidar_monitor_tree) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    std::string key = "";

    std::string mes;
    key = "lidarInCur";
    try {
      mes = GetPtreeValue(current_lidar_monitor_tree.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = error_message + std::string(ex.what());
    }
    diagnostics.add(key, mes);
    key = "lidarInVol";
    try {
      mes = GetPtreeValue(current_lidar_monitor_tree.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = error_message + std::string(ex.what());
    }
    diagnostics.add(key, mes);

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorRosWrapper::HesaiCheckVoltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor);
  if (current_monitor) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    diagnostics.add(
      "input_voltage", GetFixedPrecisionString(current_monitor->input_voltage * 0.01) + " V");
    diagnostics.add(
      "input_current", GetFixedPrecisionString(current_monitor->input_current * 0.01) + " mA");
    diagnostics.add(
      "input_power", GetFixedPrecisionString(current_monitor->input_power * 0.01) + " W");

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

  HesaiHwMonitorRosWrapper::~HesaiHwMonitorRosWrapper() {
    RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
    hw_interface_.FinalizeTcpDriver();
  }

  RCLCPP_COMPONENTS_REGISTER_NODE(HesaiHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
