#include "velodyne/velodyne_hw_interface_ros_wrapper.hpp"

#include <future>
#include <curl/curl.h>

namespace nebula
{
namespace ros
{

VelodyneHwInterfaceRosWrapper::VelodyneHwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options), hw_interface_(), diagnostics_updater_(this)
{
//  cbg_r_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//  cbg_m_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if(mtx_config_.try_lock()){
    interface_status_ = GetParameters(sensor_configuration_);
    mtx_config_.unlock();
  }
  if (Status::OK != interface_status_)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
  // Initialize sensor_configuration
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize sensor_configuration");
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::VelodyneSensorConfiguration>(sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
  updateParameters();

  // register scan callback and publisher
  hw_interface_.RegisterScanCallback(
    std::bind(&VelodyneHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  velodyne_scan_pub_ =
    this->create_publisher<velodyne_msgs::msg::VelodyneScan>("velodyne_packets", rclcpp::SensorDataQoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());
//    this->create_publisher<velodyne_msgs::msg::VelodyneScan>("velodyne_packets", rclcpp::SensorDataQoS());

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VelodyneHwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));


  key_volt_temp_top_hv = "volt_temp.top.hv";
  key_volt_temp_top_ad_temp = "volt_temp.top.ad_temp";
  key_volt_temp_top_lm20_temp = "volt_temp.top.lm20_temp";
  key_volt_temp_top_pwr_5v = "volt_temp.top.pwr_5v";
  key_volt_temp_top_pwr_2_5v = "volt_temp.top.pwr_2_5v";
  key_volt_temp_top_pwr_3_3v = "volt_temp.top.pwr_3_3v";
  key_volt_temp_top_pwr_raw = "volt_temp.top.pwr_raw";
  key_volt_temp_top_pwr_vccint = "volt_temp.top.pwr_vccint";
//  key_volt_temp_top_pwr_vccint = "volt_temp.top.pwr_vccint2";
  key_volt_temp_bot_i_out = "volt_temp.bot.i_out";
  key_volt_temp_bot_pwr_1_2v = "volt_temp.bot.pwr_1_2v";
  key_volt_temp_bot_lm20_temp = "volt_temp.bot.lm20_temp";
  key_volt_temp_bot_pwr_5v = "volt_temp.bot.pwr_5v";
  key_volt_temp_bot_pwr_2_5v = "volt_temp.bot.pwr_2_5v";
  key_volt_temp_bot_pwr_3_3v = "volt_temp.bot.pwr_3_3v";
  key_volt_temp_bot_pwr_v_in = "volt_temp.bot.pwr_v_in";
  key_volt_temp_bot_pwr_1_25v = "volt_temp.bot.pwr_1_25v";
  key_vhv = "vhv";
  key_adc_nf = "adc_nf";
  key_adc_stats = "adc_stats";
  key_ixe = "ixe";
  key_adctp_stat = "adctp_stat";
  key_status_gps_pps_state = "gps.pps_state";
  key_status_gps_pps_position = "gps.position";   
  key_status_motor_state = "motor.state";
  key_status_motor_rpm = "motor.rpm";
  key_status_motor_lock = "motor.lock";
  key_status_motor_phase = "motor.phase";
  key_status_laser_state = "laser.state";

  not_supported_message = "Not supported";



//  InitializeVelodyneDiagnostics();


}

Status VelodyneHwInterfaceRosWrapper::StreamStart()
{
  if(Status::OK == interface_status_ ){
    interface_status_ = hw_interface_.CloudInterfaceStart();
  }
  return interface_status_;
}

Status VelodyneHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
Status VelodyneHwInterfaceRosWrapper::Shutdown() { return Status::OK; }

Status VelodyneHwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status VelodyneHwInterfaceRosWrapper::GetParameters(
  drivers::VelodyneSensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(this->declare_parameter<std::string>("sensor_model", ""));
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//      nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", "", descriptor));
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.host_ip = this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//      sensor_configuration.sensor_ip = this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "velodyne", descriptor);
    this->declare_parameter<std::string>("frame_id", "velodyne", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.data_port = this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.gnss_port = this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range= {range};
//    sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0., descriptor);
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.frequency_ms = this->declare_parameter<uint16_t>("frequency_ms", 100, descriptor);
    this->declare_parameter<uint16_t>("frequency_ms", 100, descriptor);
    sensor_configuration.frequency_ms = this->get_parameter("frequency_ms").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.packet_mtu_size = this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = this->get_parameter("packet_mtu_size").as_int();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "range from 300 to 1200, in increments of 60";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(300).set__to_value(1200).set__step(1);
    descriptor.integer_range= {range};
//    sensor_configuration.rotation_speed = this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(359).set__step(1);
    descriptor.integer_range= {range};
//    sensor_configuration.cloud_min_angle = this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(359).set__step(1);
    descriptor.integer_range= {range};
//    sensor_configuration.cloud_max_angle = this->declare_parameter<uint16_t>("cloud_max_angle", 359, descriptor);
    this->declare_parameter<uint16_t>("cloud_max_angle", 359, descriptor);
    sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (
    sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360 ||
    sensor_configuration.frequency_ms == 0) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "It may be safe if it is 5000 milliseconds or more...";
//    diag_span_ = this->declare_parameter<uint16_t>("diag_span", 3000, descriptor);
    this->declare_parameter<uint16_t>("diag_span", 3000, descriptor);
    diag_span_ = this->get_parameter("diag_span").as_int();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);


/*
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("sensor_model", 4, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("return_mode", 4, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("host_ip", 4, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("sensor_ip", 4, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("frame_id", 4, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("data_port", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("gnss_port", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range= {range};
    this->declare_parameter("scan_phase", 3, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("frequency_ms", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    this->declare_parameter("packet_mtu_size", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "range from 300 to 1200, in increments of 60";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(300).set__to_value(1200).set__step(1);
    descriptor.integer_range= {range};
    this->declare_parameter("rotation_speed", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(359).set__step(1);
    descriptor.integer_range= {range};
    this->declare_parameter("cloud_min_angle", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(359).set__step(1);
    descriptor.integer_range= {range};
    this->declare_parameter("cloud_max_angle", 2, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;
    descriptor.dynamic_typing = true;
    descriptor.additional_constraints = "It may be safe if it is 5000 milliseconds or more...";
    this->declare_parameter("diag_span", 2, descriptor);
  }
*/

  
  return Status::OK;
}

void VelodyneHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  scan_buffer->header.stamp = scan_buffer->packets.front().stamp;
  velodyne_scan_pub_->publish(*scan_buffer);
}


void VelodyneHwInterfaceRosWrapper::InitializeVelodyneDiagnostics()
{
  std::cout << "InitializeVelodyneDiagnostics" << std::endl;
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  os << sensor_configuration_.sensor_model << "_" << sensor_configuration_.frame_id;
  diagnostics_updater_.setHardwareID(os.str());
  std::cout << "os.str()" << std::endl;

  diagnostics_updater_.add(
      "velodyne_snapshot-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckSnapshot);

  diagnostics_updater_.add(
      "velodyne_volt_temp_top_hv-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopHv);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_ad_temp-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopAdTemp);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_lm20_temp-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopLm20Temp);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_5v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr5v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_2_5v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr25v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_3_3v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr33v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_raw-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwrRaw);
  diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_vccint-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwrVccint);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_i_out-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotIOut);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_2v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr12v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_lm20_temp-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotLm20Temp);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_5v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr5v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_2_5v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr25v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_3_3v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr33v);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_v_in-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwrVIn);
  diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_25v-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr125v);
  diagnostics_updater_.add(
      "velodyne_vhv-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckVhv);
  diagnostics_updater_.add(
      "velodyne_adc_nf-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckAdcNf);
  diagnostics_updater_.add(
      "velodyne_adc_stats-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckAdcStats);
  diagnostics_updater_.add(
      "velodyne_ixe-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckIxe);
  diagnostics_updater_.add(
      "velodyne_adctp_stat-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckAdctpStat);

/*
  std::cout << "diagnostics_updater_.add" << std::endl;

  auto on_timer_diag = [this] { OnVelodyneDiagnosticsTimer(); };
  diagnostics_diag_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_diag)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_diag), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
//  this->get_node_timers_interface()->add_timer(diagnostics_diag_timer_, nullptr);
  this->get_node_timers_interface()->add_timer(diagnostics_diag_timer_, nullptr);
*/


  diagnostics_updater_.add(
      "velodyne_status_gps_pps_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckGpsPpsState);
  diagnostics_updater_.add(
      "velodyne_status_gps_pps_position-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckGpsPosition);
  diagnostics_updater_.add(
      "velodyne_status_motor_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorState);
  diagnostics_updater_.add(
      "velodyne_status_motor_rpm-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorRpm);
  diagnostics_updater_.add(
      "velodyne_status_motor_lock-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorLock);
  diagnostics_updater_.add(
      "velodyne_status_motor_phase-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorPhase);
  diagnostics_updater_.add(
      "velodyne_status_laser_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckLaserState);

  /*
  auto on_timer_status = [this] { OnVelodyneStatusTimer(); };
  diagnostics_status_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_status)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_status), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
  this->get_node_timers_interface()->add_timer(diagnostics_status_timer_, nullptr);
  */

  current_snapshot.reset(new std::string(""));
//  current_snapshot_time.reset(this->get_clock()->now());
  current_snapshot_time.reset(new rclcpp::Time(this->get_clock()->now()));
//  current_snapshot_time = this->get_clock()->now();
//  current_diag_status.reset(diagnostic_msgs::msg::DiagnosticStatus::STALE);
  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;


  auto on_timer_shnapshot = [this] { OnVelodyneSnapshotTimer(); };
  diagnostics_snapshot_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_shnapshot)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_shnapshot), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
//  this->get_node_timers_interface()->add_timer(diagnostics_snapshot_timer_, nullptr);
//  this->get_node_timers_interface()->add_timer(diagnostics_snapshot_timer_, cbg_m_); //220721 killed for reconfigure


  auto on_timer_update = [this] {
    std::cout << "OnUpdateTimer" << std::endl;
    auto now = this->get_clock()->now();
    auto dif = (now - *current_snapshot_time).seconds();
    std::cout << "dif: " << dif <<  std::endl;
//    if(diag_span_ < dif*1000){
    if(diag_span_*2.0 < dif*1000){
//      current_diag_status.reset(diagnostic_msgs::msg::DiagnosticStatus::STALE);
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      std::cout << "STALE" << std::endl;
    }else{
//      current_diag_status.reset(diagnostic_msgs::msg::DiagnosticStatus::OK);
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      std::cout << "OK" << std::endl;
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_update)>>(
//    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_update), this->get_node_base_interface()->get_context());
//    this->get_clock(), std::chrono::milliseconds(1000), std::move(on_timer_update), this->get_node_base_interface()->get_context());
    this->get_clock(), std::chrono::milliseconds(100), std::move(on_timer_update), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
//  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, nullptr);
//  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, cbg_r_); //220721 killed for reconfigure

//  std::cout << "add_timer" << std::endl;
}

std::string VelodyneHwInterfaceRosWrapper::GetPtreeValue(std::shared_ptr<boost::property_tree::ptree> pt, const std::string& key){
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if(value)
  {
    return value.get();
  }else{
    return not_supported_message;
  }
}

// https://memo.appri.me/programming/cpp-curl-http-client
using namespace std;
typedef void (*CurlCallback)(string err, string body);

class Curl {
private:

    /** response body */
    string body;

    // TIP: CURLOPT_WRITEFUNCTION ÅÍ øÆÈéÖÉ static µ©ó¯t¯È¢ÌÅ­øÉ static cast µÄ¢Ü·:
    // see: https://curl.se/docs/faq.html#Using_C_non_static_functions_f
    static size_t invoke_write_data(char *buffer, size_t size, size_t nmemb, void *f) {
        // Call non-static member function.
        return static_cast<Curl*>(f)->write_data(buffer, size, nmemb, f);
    }

    /** a callback function for libcurl request */
    size_t write_data(char *buffer, size_t size, size_t nmemb, void *f) {
        int dataLength = size * nmemb;
        this->body.append(buffer, dataLength);
        return dataLength;
    }

public:

    /** user-agent */
    string useragent = "libcurl-agent/1.0";
    /** timeout */
    int timeout = 30L; // timeout 30 seconds

    /**
     * Constructor
     */
    Curl() {
        //
    }

    /**
     * HTTP GET
     */
    void get(const string url, const CurlCallback cb) {
        CURL* curl;
        CURLcode ret;

        this->body = ""; // init result body.
        string err = "";

        curl_global_init(CURL_GLOBAL_ALL);
        curl = curl_easy_init();

        if (curl == NULL) {
            err = "curl_easy_init() failed on " + url;
            return cb(err, "");
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, this->invoke_write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, this);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, this->useragent.c_str()); // UA
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, this->timeout); // timeout
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // verbose
        ret = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_global_cleanup();

        if (ret != CURLE_OK) {
            err = "curl_easy_perform() failed on " + url + " (ret:" + to_string(ret) + ")";
            return cb(err, "");
        }
        return cb(err, this->body);
    }

    /**
     * HTTP POST
     */
    void post(const string url, const string data, const CurlCallback cb) {
        CURL* curl;
        CURLcode ret;

        this->body = ""; // init result body.
        string err = "";

        curl_global_init(CURL_GLOBAL_ALL);
        curl = curl_easy_init();

        if (curl == NULL) {
            err = "curl_easy_init() failed on " + url;
            return cb(err, "");
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, this->invoke_write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, this);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, this->useragent.c_str()); // UA
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, this->timeout); // timeout
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // verbose
        ret = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_global_cleanup();

        if (ret != CURLE_OK) {
            err = "curl_easy_perform() failed on " + url + " (ret:" + to_string(ret) + ")";
            return cb(err, "");
        }
        return cb(err, this->body);
    }

};

void VelodyneHwInterfaceRosWrapper::curl_callback(std::string err, std::string body)
 {
    if (err != "") {
      std::cerr << "Error:" << err << std::endl;
    } else {
      std::cout << body << std::endl;
      current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(body));
      std::cout << "diagnostics_updater_.force_update()" << std::endl;
      diagnostics_updater_.force_update();
    }
 }
 
void VelodyneHwInterfaceRosWrapper::OnVelodyneDiagnosticsTimer()
{
  std::cout << "OnVelodyneDiagnosticsTimer" << std::endl;
  /*
  uint32_t lidar_status_code;
  if (driver_.GetLidarStatusCode(lidar_status_code)) {
    current_diagnostics_status_ =
      livox_driver::diagnostics::StatusCodeToLivoxSensorStatus(lidar_status_code);
  }
  diagnostics_updater_.force_update();
  */
  if(true){
 //*
  if(mtx_diag.try_lock() || true){
    std::cout << "mtx_diag lock" << std::endl;
    hw_interface_.GetDiagAsync(
      [this](const std::string &str)
      {
  //        std::cout << "make_shared" << std::endl;
        current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
        diagnostics_updater_.force_update();
        mtx_diag.unlock();
        std::cout << "mtx_diag unlock" << std::endl;
      });
//    std::cout << "run hw_interface_.GetDiagAsync" << std::endl;
  }else{
    std::cout << "mtx_diag is locked..." << std::endl;
  }
  //*/

 }else if(false){
  auto self(shared_from_this());
  std::future<VelodyneStatus> future = std::async(std::launch::async, [self, this](){
    return hw_interface_.GetDiagAsync(
      [this](const std::string &str)
      {
  //        std::cout << "make_shared" << std::endl;
        current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
        diagnostics_updater_.force_update();
      });
  });
  auto future_status = future.wait_for(std::chrono::milliseconds(1000));
  switch(future_status) {
      case std::future_status::deferred: std::cout << "deferred\n"; break;
      case std::future_status::timeout: std::cout << "timeout\n"; break;
      case std::future_status::ready: std::cout << "ready!\n"; break;
  }
 }else if(false){
    auto str = hw_interface_.GetDiag();
    std::cout << "hw_interface_.GetDiag() : " << str << std::endl;
    std::cout << "ParseJson" << std::endl;
    current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
    std::cout << "diagnostics_updater_.force_update()" << std::endl;
    diagnostics_updater_.force_update();

  }else if(false){
    boost::asio::io_context ioc;
    boost::asio::ip::tcp::resolver resolver(ioc);
    beast::tcp_stream stream(ioc);
    auto const results = resolver.resolve(sensor_configuration_.sensor_ip, "80");

    stream.connect(results);

    boost::beast::http::request<http::string_body> req{boost::beast::http::verb::get, "/cgi/diag.json", 11};
    req.set(boost::beast::http::field::host, sensor_configuration_.sensor_ip);
    req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);

    boost::beast::http::write(stream, req);

    beast::flat_buffer buffer;

    boost::beast::http::response<boost::beast::http::dynamic_body> res;

    // Receive the HTTP response
    boost::beast::http::read(stream, buffer, res);

    std::cout << res << std::endl;
    auto m_res_string = beast::buffers_to_string(res.body().data());
    current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(m_res_string));
    std::cout << "diagnostics_updater_.force_update()" << std::endl;
    diagnostics_updater_.force_update();

    // Gracefully close the socket
    beast::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);

    // not_connected happens sometimes
    // so don't bother reporting it.
    //
    if(ec && ec != beast::errc::not_connected)
        throw beast::system_error{ec};
  }else{
    Curl* curl = new Curl();
    std::string url = "http://" + sensor_configuration_.sensor_ip + "/cgi/diag.json";
    //*
    curl->get(url, [](std::string err, std::string body) {
      if (err != "") {
        std::cerr << "ERROR: " << err << std::endl;
      } else {
        std::cout << body << std::endl;
//        current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(body));
//        std::cout << "diagnostics_updater_.force_update()" << std::endl;
//        diagnostics_updater_.force_update();
      }
    });
    //*/
//     curl->get(url, VelodyneHwInterfaceRosWrapper::curl_callback);

  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopHv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopHv" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.hv"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_ad_temp));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopAdTemp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopAdTemp" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.ad_temp"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_ad_temp));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopLm20Temp" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.lm20_temp"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_lm20_temp));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr5v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_5v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_5v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr25v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_2_5v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_2_5v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr33v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_3_3v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_3_3v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwrRaw(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwrRaw" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_raw"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_raw));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwrVccint(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwrVccint" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_vccint"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_vccint));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotIOut(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotIOut" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.i_out"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_i_out));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr12v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr12v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_1_2v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_2v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotIOut" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.lm20_temp"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_lm20_temp));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr5v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_5v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_5v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr25v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_2_5v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_2_5v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr33v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_3_3v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_3_3v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwrVIn(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwrVIn" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_v_in"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_v_in));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckBotPwr125v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr125v" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_1_25v"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_25v));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckVhv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckVhv" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("vhv"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_vhv));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckAdcNf(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdcNf" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);

    std::ostringstream os;
    /*
    boost::property_tree::ptree pt_array = VelodyneHwInterfaceRosWrapper::current_diag_tree->get_child("adc_nf");
    for (auto v = pt_array.begin(); v != pt_array.end(); ++v) {
        os << v->second.get<std::string>("") << ", ";
    }
    diagnostics.summary(level, os.str());
    */
    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adc_nf);
    if(child)
    {
      std::ostringstream os;
      for (auto v = child->begin(); v != child->end(); ++v) {
          os << v->second.get<std::string>("") << ", ";
      }
      diagnostics.summary(level, os.str());
    }else{
      diagnostics.summary(level, not_supported_message);
    }
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckAdcStats(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdcStats" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);

    std::ostringstream os;
    /*
    boost::property_tree::ptree pt_array = VelodyneHwInterfaceRosWrapper::current_diag_tree->get_child("adc_stats");
    for (auto v = pt_array.begin(); v != pt_array.end(); ++v) {
        os << "(";
        os << "mean:" << v->second.get<std::string>("mean") << ", ";
        os << "stddev:" << v->second.get<std::string>("stddev") << ", ";
        os << "), ";
    }
    diagnostics.summary(level, os.str());
    */

    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adc_stats);
    if(child)
    {
      std::ostringstream os;
      /*
      boost::property_tree::ptree pt_array = child->get_child("adctp_stat");
      for (auto v = pt_array.begin(); v != pt_array.end(); ++v) {
          os << v->second.get<std::string>("") << ", ";
      }
      */
      for (auto v = child->begin(); v != child->end(); ++v) {
          os << "(";
          os << "mean:" << v->second.get<std::string>("mean") << ", ";
          os << "stddev:" << v->second.get<std::string>("stddev") << ", ";
          os << "), ";
      }
      diagnostics.summary(level, os.str());
    }else{
      diagnostics.summary(level, not_supported_message);
    }
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckIxe(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckIxe" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_diag_tree->get<std::string>("ixe"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_ixe));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckAdctpStat(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdctpStat" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_diag_tree && !VelodyneHwInterfaceRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    diagnostics.add("sensor", sensor_configuration_.frame_id);
    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adctp_stat);
    if(child)
    {
      std::ostringstream os;
//      boost::property_tree::ptree pt_array = child->get_child("adctp_stat");
//      for (auto v = pt_array.begin(); v != pt_array.end(); ++v) {
      for (auto v = child->begin(); v != child->end(); ++v) {
          os << v->second.get<std::string>("") << ", ";
      }
      diagnostics.summary(level, os.str());
    }else{
      diagnostics.summary(level, not_supported_message);
    }
  }
}
/*
void VelodyneHwInterfaceRosWrapper::InitializeVelodyneStatus()
{
  std::cout << "InitializeVelodyneStatus" << std::endl;
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  os << sensor_configuration_.sensor_model << "_" << sensor_configuration_.frame_id;
  diagnostics_updater_.setHardwareID(os.str());
  diagnostics_updater_.add(
      "velodyne_status_gps_pps_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopHv);
  diagnostics_updater_.add(
      "velodyne_status_gps_pps_position-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopAdTemp);
  diagnostics_updater_.add(
      "velodyne_status_motor_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopLm20Temp);
  diagnostics_updater_.add(
      "velodyne_status_motor_rpm-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr5v);
  diagnostics_updater_.add(
      "velodyne_status_motor_lock-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr25v);
  diagnostics_updater_.add(
      "velodyne_status_motor_phase-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwr33v);
  diagnostics_updater_.add(
      "velodyne_status_laser_state-" + sensor_configuration_.frame_id,
      this, &VelodyneHwInterfaceRosWrapper::VelodyneCheckTopPwrRaw);

  auto on_timer = [this] { OnVelodyneStatusTimer(); };
  diagnostics_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
    this->get_clock(), std::chrono::milliseconds(3000), std::move(on_timer), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
  this->get_node_timers_interface()->add_timer(diagnostics_timer_, nullptr);
  std::cout << "add_timer" << std::endl;
}
*/

void VelodyneHwInterfaceRosWrapper::OnVelodyneStatusTimer()
{
  std::cout << "OnVelodyneStatusTimer" << std::endl;
  if(mtx_status.try_lock()){
    std::cout << "mtx_status lock" << std::endl;
    hw_interface_.GetStatusAsync(
      [this](const std::string &str)
      {
        current_status_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
        diagnostics_updater_.force_update();
        mtx_status.unlock();
          std::cout << "mtx_status unlock" << std::endl;
      });
    std::cout << "run hw_interface_.GetStatusAsync" << std::endl;
  }else{
    std::cout << "mtx_status is locked..." << std::endl;
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckGpsPpsState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckGpsPpsState" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("gps.pps_state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_gps_pps_state));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckGpsPosition(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckGpsPosition" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("gps.position"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_gps_pps_position));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorState" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("motor.state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_state));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorRpm" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("motor.rpm"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_rpm));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorLock(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorLock" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("motor.lock"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_lock));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckMotorPhase(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorPhase" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("motor.phase"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_phase));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckLaserState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckLaserState" << std::endl;
  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwInterfaceRosWrapper::current_status_tree->get<std::string>("laser.state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_laser_state));
  }
}

void VelodyneHwInterfaceRosWrapper::VelodyneCheckSnapshot(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckLaserState" << std::endl;
//  if(VelodyneHwInterfaceRosWrapper::current_status_tree && !VelodyneHwInterfaceRosWrapper::current_status_tree->empty()){
//    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    uint8_t level = current_diag_status;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(level, *current_snapshot);
//  }
}



void VelodyneHwInterfaceRosWrapper::OnVelodyneSnapshotTimer()
{
  std::cout << "OnVelodyneSnapshotTimer" << std::endl;
  hw_interface_.GetSnapshotAsync(
    [this](const std::string &str)
    {
      current_snapshot_time.reset(new rclcpp::Time(this->get_clock()->now()));
//      current_snapshot_time = this->get_clock()->now();

//      std::cout << str << std::endl;
      current_snapshot_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
//      current_diag_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
      current_diag_tree = std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("diag"));
//      current_status_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
      current_status_tree = std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("status"));
//        diagnostics_updater_.force_update();
      current_snapshot.reset(new std::string(str));
    });

  /*  
  auto parameter_list_future = this->list_parameters({"rotation_speed", "cloud_min_angle"}, 10);
  for (auto & name : parameter_list_future.names) {
    std::cout << "Parameter name: " << name << std::endl;
  }
  */
}


rcl_interfaces::msg::SetParametersResult VelodyneHwInterfaceRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  std::cout << "add_on_set_parameters_callback" << std::endl;
  std::cout << p << std::endl;
  std::cout << sensor_configuration_ << std::endl;

  drivers::VelodyneSensorConfiguration new_param{sensor_configuration_};
  std::cout << new_param << std::endl;
  std::string sensor_model_str;
  std::string return_mode_str;
  uint16_t new_diag_span = 0;
  if (
    get_param(p, "sensor_model", sensor_model_str) ||
    get_param(p, "return_mode", return_mode_str) ||
    get_param(p, "host_ip", new_param.host_ip) ||
    get_param(p, "sensor_ip", new_param.sensor_ip) ||
    get_param(p, "frame_id", new_param.frame_id) ||
    get_param(p, "data_port", new_param.data_port) ||
    get_param(p, "gnss_port", new_param.gnss_port) ||
    get_param(p, "scan_phase", new_param.scan_phase) ||
    get_param(p, "frequency_ms", new_param.frequency_ms) ||
    get_param(p, "packet_mtu_size", new_param.packet_mtu_size) ||
    get_param(p, "rotation_speed", new_param.rotation_speed) ||
    get_param(p, "cloud_min_angle", new_param.cloud_min_angle) ||
    get_param(p, "cloud_max_angle", new_param.cloud_max_angle) ||
    get_param(p, "diag_span", new_diag_span)) {

    if(0 < sensor_model_str.length())
      new_param.sensor_model =
        nebula::drivers::SensorModelFromString(sensor_model_str);
    if(0 < return_mode_str.length())
      new_param.return_mode =
        nebula::drivers::ReturnModeFromString(return_mode_str);
    if(0 < new_diag_span)
      diag_span_ = new_diag_span;

    sensor_configuration_ = new_param;
    // Update sensor_configuration
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::VelodyneSensorConfiguration>(sensor_configuration_);
    RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
//    updateParameters();
  }

//  rcl_interfaces::msg::SetParametersResult result;
  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  std::cout << "add_on_set_parameters_callback success" << std::endl;

//  return result;
  return *result;
}

std::vector<rcl_interfaces::msg::SetParametersResult> VelodyneHwInterfaceRosWrapper::updateParameters(){
  std::scoped_lock lock(mtx_config_);
  std::cout << "!!!!!!!!!!!updateParameters!!!!!!!!!!!!" << std::endl;
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  std::ostringstream os_return_mode;
  os_return_mode << sensor_configuration_.return_mode;
  std::cout << "set_parameters start" << std::endl;
  auto results = set_parameters({
    rclcpp::Parameter("sensor_model", os_sensor_model.str()),
    rclcpp::Parameter("return_mode", os_return_mode.str()),
    rclcpp::Parameter("host_ip", sensor_configuration_.host_ip),
    rclcpp::Parameter("sensor_ip", sensor_configuration_.sensor_ip),
    rclcpp::Parameter("frame_id", sensor_configuration_.frame_id),
    rclcpp::Parameter("data_port", sensor_configuration_.data_port),
    rclcpp::Parameter("gnss_port", sensor_configuration_.gnss_port),
    rclcpp::Parameter("scan_phase", sensor_configuration_.scan_phase),
    rclcpp::Parameter("frequency_ms", sensor_configuration_.frequency_ms),
    rclcpp::Parameter("packet_mtu_size", sensor_configuration_.packet_mtu_size),
    rclcpp::Parameter("rotation_speed", sensor_configuration_.rotation_speed),
    rclcpp::Parameter("cloud_min_angle", sensor_configuration_.cloud_min_angle),
    rclcpp::Parameter("cloud_max_angle", sensor_configuration_.cloud_max_angle),
    rclcpp::Parameter("diag_span", diag_span_)
  });
  std::cout << "set_parameters fin" << std::endl;
  return results;
}

}  // namespace ros
}  // namespace nebula
