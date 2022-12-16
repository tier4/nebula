#include "velodyne/velodyne_hw_monitor_ros_wrapper.hpp"

#include <future>
#include <curl/curl.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/join.hpp>
#include <math.h>

namespace nebula
{
namespace ros
{

VelodyneHwMonitorRosWrapper::VelodyneHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_hw_monitor_ros_wrapper", options), hw_interface_(), diagnostics_updater_(this)
{
  cbg_r_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_m_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.InitializeSensorConfiguration");
  hw_interface_.InitializeSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VelodyneHwMonitorRosWrapper::paramCallback, this, std::placeholders::_1));



  key_volt_temp_top_hv = "volt_temp.top.hv";
  key_volt_temp_top_ad_temp = "volt_temp.top.ad_temp";//only32
  key_volt_temp_top_lm20_temp = "volt_temp.top.lm20_temp";
  key_volt_temp_top_pwr_5v = "volt_temp.top.pwr_5v";
  key_volt_temp_top_pwr_2_5v = "volt_temp.top.pwr_2_5v";
  key_volt_temp_top_pwr_3_3v = "volt_temp.top.pwr_3_3v";
  key_volt_temp_top_pwr_5v_raw = "volt_temp.top.pwr_5v_raw";//only16
  key_volt_temp_top_pwr_raw = "volt_temp.top.pwr_raw";//only32
  key_volt_temp_top_pwr_vccint = "volt_temp.top.pwr_vccint";
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

  name_volt_temp_top_hv = "Top HV";
  name_volt_temp_top_ad_temp = "Top A/D TD";
  name_volt_temp_top_lm20_temp = "Top Temp";
  name_volt_temp_top_pwr_5v = "Top 5v";
  name_volt_temp_top_pwr_2_5v = "Top 2.5v";
  name_volt_temp_top_pwr_3_3v = "Top 3.3v";
  name_volt_temp_top_pwr_5v_raw = "Top 5v(RAW)";
  name_volt_temp_top_pwr_raw = "Top RAW";
  name_volt_temp_top_pwr_vccint = "Top VCCINT";
  name_volt_temp_bot_i_out = "Bot I out";
  name_volt_temp_bot_pwr_1_2v = "Bot 1.2v";
  name_volt_temp_bot_lm20_temp = "Bot Temp";
  name_volt_temp_bot_pwr_5v = "Bot 5v";
  name_volt_temp_bot_pwr_2_5v = "Bot 2.5v";
  name_volt_temp_bot_pwr_3_3v = "Bot 3.3v";
  name_volt_temp_bot_pwr_v_in = "Bot V in";
  name_volt_temp_bot_pwr_1_25v = "Bot 1.25v";// N/A?
  name_vhv = "VHV";
  name_adc_nf = "adc_nf";
  name_adc_stats = "adc_stats";
  name_ixe = "ixe";
  name_adctp_stat = "adctp_stat";
  name_status_gps_pps_state = "GPS PPS";
  name_status_gps_pps_position = "GPS Position";   
  name_status_motor_state = "Motor State";
  name_status_motor_rpm = "Motor RPM";
  name_status_motor_lock = "Motor Lock";
  name_status_motor_phase = "Motor Phase";
  name_status_laser_state = "Laser State";

  message_sep = ": ";

  not_supported_message = "Not supported";
  error_message = "Error";

  key_info_model = "info.model";
  key_info_serial = "info.serial";

  temperature_cold_message = "temperature cold";
  temperature_hot_message = "temperature hot";
  voltage_low_message = "voltage low";
  voltage_high_message = "voltage high";
  ampere_low_message = "ampere low";
  ampere_high_message = "ampere high";

//  InitializeVelodyneDiagnostics();

  std::cout << "Get model name and serial." << std::endl;
  hw_interface_.GetSnapshotAsync(
    [this](const std::string &str)
    {
      current_snapshot_time.reset(new rclcpp::Time(this->get_clock()->now()));
      current_snapshot_tree = std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
      current_diag_tree = std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("diag"));
      current_status_tree = std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("status"));
      current_snapshot.reset(new std::string(str));

      try{
        info_model = GetPtreeValue(current_snapshot_tree, key_info_model);
        info_serial = GetPtreeValue(current_snapshot_tree, key_info_serial);
        RCLCPP_INFO_STREAM(this->get_logger(), "Model:" << info_model);
        RCLCPP_INFO_STREAM(this->get_logger(), "Serial:" << info_serial);
      }catch(boost::bad_lexical_cast &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << "Can't get model and serial");
        return;
      }

      InitializeVelodyneDiagnostics();
    });


}

Status VelodyneHwMonitorRosWrapper::MonitorStart()
{
  return interface_status_;
}

Status VelodyneHwMonitorRosWrapper::MonitorStop() { return Status::OK; }
Status VelodyneHwMonitorRosWrapper::Shutdown() { return Status::OK; }


Status VelodyneHwMonitorRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status VelodyneHwMonitorRosWrapper::GetParameters(
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
    descriptor.read_only = true;
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
    descriptor.read_only = true;
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
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range= {range};
//    sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0., descriptor);
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  /*
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
//    sensor_configuration.frequency_ms = this->declare_parameter<uint16_t>("frequency_ms", 100, descriptor);
    this->declare_parameter<uint16_t>("frequency_ms", 100, descriptor);
    sensor_configuration.frequency_ms = this->get_parameter("frequency_ms").as_int();
  }
  */
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
    descriptor.read_only = true;
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
    descriptor.read_only = true;
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
    descriptor.read_only = true;
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
    sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {// ||
//    sensor_configuration.frequency_ms == 0) {
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

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 1;
    descriptor.read_only = true; // because it affects initialization
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Showing advanced diagnostics";
    this->declare_parameter<bool>("advanced_diagnostics", false, descriptor);
    use_advanced_diagnostics = this->get_parameter("advanced_diagnostics").as_bool();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);

  
  return Status::OK;
}


void VelodyneHwMonitorRosWrapper::InitializeVelodyneDiagnostics()
{
  std::cout << "InitializeVelodyneDiagnostics" << std::endl;
  using std::chrono_literals::operator""s;
  std::ostringstream os;
//  os << sensor_configuration_.sensor_model << "_" << sensor_configuration_.frame_id;
//  os << info_model << ": " << info_serial;
  auto hardware_id = info_model + ": " + info_serial;
//  diagnostics_updater_.setHardwareID(os.str());
  diagnostics_updater_.setHardwareID(hardware_id);
//  std::cout << os.str() << std::endl;
  std::cout << "hardware_id: " << hardware_id << std::endl;

  if(use_advanced_diagnostics){
    diagnostics_updater_.add(
        "velodyne_snapshot-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckSnapshot);

    diagnostics_updater_.add(
        "velodyne_volt_temp_top_hv-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopHv);
    if(sensor_configuration_.sensor_model != nebula::drivers::SensorModel::VELODYNE_VLP16){
      diagnostics_updater_.add(
          "velodyne_volt_temp_top_ad_temp-" + sensor_configuration_.frame_id,
          this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopAdTemp);
    }
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_lm20_temp-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopLm20Temp);
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_pwr_5v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr5v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_pwr_2_5v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr25v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_pwr_3_3v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr33v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_pwr_raw-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwrRaw);
    diagnostics_updater_.add(
        "velodyne_volt_temp_top_pwr_vccint-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwrVccint);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_i_out-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotIOut);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_1_2v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr12v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_lm20_temp-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotLm20Temp);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_5v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr5v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_2_5v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr25v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_3_3v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr33v);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_v_in-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwrVIn);
    diagnostics_updater_.add(
        "velodyne_volt_temp_bot_pwr_1_25v-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr125v);
    diagnostics_updater_.add(
        "velodyne_vhv-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckVhv);
    diagnostics_updater_.add(
        "velodyne_adc_nf-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckAdcNf);
    diagnostics_updater_.add(
        "velodyne_adc_stats-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckAdcStats);
    diagnostics_updater_.add(
        "velodyne_ixe-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckIxe);
    diagnostics_updater_.add(
        "velodyne_adctp_stat-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckAdctpStat);

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
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckGpsPpsState);
    diagnostics_updater_.add(
        "velodyne_status_gps_pps_position-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckGpsPosition);
    diagnostics_updater_.add(
        "velodyne_status_motor_state-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckMotorState);
    diagnostics_updater_.add(
        "velodyne_status_motor_rpm-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckMotorRpm);
    diagnostics_updater_.add(
        "velodyne_status_motor_lock-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckMotorLock);
    diagnostics_updater_.add(
        "velodyne_status_motor_phase-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckMotorPhase);
    diagnostics_updater_.add(
        "velodyne_status_laser_state-" + sensor_configuration_.frame_id,
        this, &VelodyneHwMonitorRosWrapper::VelodyneCheckLaserState);

  /*
  auto on_timer_status = [this] { OnVelodyneStatusTimer(); };
  diagnostics_status_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_status)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_status), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
  this->get_node_timers_interface()->add_timer(diagnostics_status_timer_, nullptr);
  */
  }

  diagnostics_updater_.add(
      "velodyne_status",
      this, &VelodyneHwMonitorRosWrapper::VelodyneCheckStatus);
  diagnostics_updater_.add(
      "velodyne_pps",
      this, &VelodyneHwMonitorRosWrapper::VelodyneCheckPps);
  diagnostics_updater_.add(
      "velodyne_temperature",
      this, &VelodyneHwMonitorRosWrapper::VelodyneCheckTemperature);
  diagnostics_updater_.add(
      "velodyne_rpm",
      this, &VelodyneHwMonitorRosWrapper::VelodyneCheckRpm);
  diagnostics_updater_.add(
      "velodyne_voltage",
      this, &VelodyneHwMonitorRosWrapper::VelodyneCheckVoltage);


  current_snapshot.reset(new std::string(""));
//  current_snapshot_time.reset(this->get_clock()->now());
  current_snapshot_time.reset(new rclcpp::Time(this->get_clock()->now()));
//  current_snapshot_time = this->get_clock()->now();
//  current_diag_status.reset(diagnostic_msgs::msg::DiagnosticStatus::STALE);
  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;


  auto on_timer_snapshot = [this] { OnVelodyneSnapshotTimer(); };
  diagnostics_snapshot_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_snapshot)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_snapshot), this->get_node_base_interface()->get_context());
  // Stopped in less than 2.5 seconds...
//  this->get_node_timers_interface()->add_timer(diagnostics_snapshot_timer_, nullptr);
  this->get_node_timers_interface()->add_timer(diagnostics_snapshot_timer_, cbg_m_); //220721 killed for reconfigure

  //*
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
  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, cbg_r_); //220721 killed for reconfigure
  //*/
  /*
  diagnostics_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] {
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
  });
  */

  std::cout << "add_timer" << std::endl;
}

std::string VelodyneHwMonitorRosWrapper::GetPtreeValue(std::shared_ptr<boost::property_tree::ptree> pt, const std::string& key){
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if(value)
  {
    return value.get();
  }else{
    return not_supported_message;
  }
}
std::string VelodyneHwMonitorRosWrapper::GetFixedPrecisionString(double val, int pre){
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
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

void VelodyneHwMonitorRosWrapper::curl_callback(std::string err, std::string body)
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
 
void VelodyneHwMonitorRosWrapper::OnVelodyneDiagnosticsTimer()
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
//     curl->get(url, VelodyneHwMonitorRosWrapper::curl_callback);

  }
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopHv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_hv));
    val = 101.0 * (val * 5.0 / 4096.0 - 5.0);
    if(val < -150.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep + voltage_low_message;
    }else if(-132.0 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
//  return std::tuple<bool, diagnostic_msgs::msg::DiagnosticStatus, std::string>(not_ex, level, mes);
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopAdTemp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_ad_temp));
    val = val * 5.0 / 4096.0;
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopLm20Temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if(val < -25.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep + temperature_cold_message;
    }else if(90.0 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep + temperature_hot_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = GetFixedPrecisionString(val) + " C";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if(val < 4.8){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep + voltage_low_message;
    }else if(5.2 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if(val < 2.3){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep + voltage_low_message;
    }else if(2.7 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if(val < 3.1){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep + voltage_low_message;
    }else if(3.5 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwr5vRaw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_5v_raw));
    val = 2.0 * val * 5.0 / 4096.0;
    if(val < 2.3){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep + voltage_low_message;
    }else if(2.7 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwrRaw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_raw));
    val = val * 5.0 / 4096.0;
    if(val < 1.6){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep + voltage_low_message;
    }else if(1.9 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetTopPwrVccint()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_vccint));
    val = val * 5.0 / 4096.0;
    if(val < 1.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep + voltage_low_message;
    }else if(1.4 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotIOut()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_i_out));
    val = 10.0 * (val * 5.0 / 4096.0 - 2.5);
    if(val < 0.3){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep + ampere_low_message;
    }else if(1.0 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep + ampere_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " A";
    mes = GetFixedPrecisionString(val) + " A";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwr12v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_2v));
    val = val * 5.0 / 4096.0;
    if(val < 1.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep + voltage_low_message;
    }else if(1.4 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotLm20Temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if(val < -25.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep + temperature_cold_message;
    }else if(90.0 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep + temperature_hot_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = GetFixedPrecisionString(val) + " C";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if(val < 4.8){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep + voltage_low_message;
    }else if(5.2 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if(val < 2.3){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep + voltage_low_message;
    }else if(2.7 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if(val < 3.1){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep + voltage_low_message;
    }else if(3.5 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwrVIn()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_v_in));
    val = 11.0 * val * 5.0 / 4096.0;
    if(val < 8.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep + voltage_low_message;
    }else if(19.0 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetBotPwr125v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_25v));
    val = val * 5.0 / 4096.0;
    if(val < 1.0){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep + voltage_low_message;
    }else if(1.4 < val){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep + voltage_high_message;
    }
//    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetVhv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_vhv));
    mes = boost::lexical_cast<std::string>(val);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetAdcNf()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adc_nf);
    if(child)
    {
      std::ostringstream os;
      for (auto v = child->begin(); v != child->end(); ++v) {
          os << v->second.get<std::string>("") << ", ";
      }
      mes = os.str();
    }else{
      mes = not_supported_message;
    }
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetAdcStats()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    std::ostringstream os;
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
      mes = os.str();
    }else{
      mes = not_supported_message;
    }
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetIxe()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_diag_tree, key_ixe);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetAdctpStat()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adctp_stat);
    if(child)
    {
      std::ostringstream os;
//      boost::property_tree::ptree pt_array = child->get_child("adctp_stat");
//      for (auto v = pt_array.begin(); v != pt_array.end(); ++v) {
      for (auto v = child->begin(); v != child->end(); ++v) {
          os << v->second.get<std::string>("") << ", ";
      }
      mes = os.str();
    }else{
      mes = not_supported_message;
    }
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}


std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetGpsPpsState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_gps_pps_state);
    if(mes == "Absent"){
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = mes;
    }else if(mes == "Error"){
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      error_mes = mes;
    }
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetGpsPosition()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_gps_pps_position);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetMotorState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_motor_state);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetMotorRpm()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_motor_rpm);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetMotorLock()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_motor_lock);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetMotorPhase()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_motor_phase);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorRosWrapper::VelodyneGetLaserState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try{
    mes = GetPtreeValue(current_status_tree, key_status_laser_state);
  }catch(boost::bad_lexical_cast &ex){
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}





void VelodyneHwMonitorRosWrapper::VelodyneCheckTopHv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopHv" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.hv"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_hv));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_hv));
      val = 101.0 * (val * 5.0 / 4096.0 - 5.0);
      if(val < -150.0 || -132.0 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopHv();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }else{
//    std::cout << "Not VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()" << std::endl;
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopAdTemp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopAdTemp" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.ad_temp"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_ad_temp));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_ad_temp));
      val = val * 5.0 / 4096.0;
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopAdTemp();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopLm20Temp" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.lm20_temp"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_lm20_temp));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_lm20_temp));
      val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
      if(val < -25.0 || 90.0 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " C");
    }catch(boost::bad_lexical_cast &ex){
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopLm20Temp();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr5v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_5v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_5v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_5v));
      val = 2.0 * val * 5.0 / 4096.0;
      if(val < 4.8 || 5.2 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopPwr5v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr25v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_2_5v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_2_5v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_2_5v));
      val = val * 5.0 / 4096.0;
      if(val < 2.3 || 2.7 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopPwr25v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwr33v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_3_3v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_3_3v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_3_3v));
      val = val * 5.0 / 4096.0;
      if(val < 3.1 || 3.5 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopPwr33v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwrRaw(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwrRaw" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_raw"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_raw));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_raw));
      val = val * 5.0 / 4096.0;
      if(val < 1.6 || 1.9 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopPwrRaw();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTopPwrVccint(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckTopPwrVccint" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.top.pwr_vccint"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_vccint));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_top_pwr_vccint));
      val = val * 5.0 / 4096.0;
      if(val < 1.0 || 1.4 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetTopPwrVccint();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotIOut(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotIOut" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.i_out"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_i_out));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_i_out));
      val = 10.0 * (val * 5.0 / 4096.0 - 2.5);
      if(val < 0.3 || 1.0 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " A");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotIOut();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr12v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr12v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_1_2v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_2v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_2v));
      val = val * 5.0 / 4096.0;
      if(val < 1.0 || 1.4 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwr12v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotIOut" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.lm20_temp"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_lm20_temp));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_lm20_temp));
      val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
      if(val < -25.0 || 90.0 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " C");
    }catch(boost::bad_lexical_cast &ex){
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotLm20Temp();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr5v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_5v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_5v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_5v));
      val = 2.0 * val * 5.0 / 4096.0;
      if(val < 4.8 || 5.2 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwr5v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr25v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_2_5v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_2_5v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_2_5v));
      val = val * 5.0 / 4096.0;
      if(val < 2.3 || 2.7 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwr25v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr33v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_3_3v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_3_3v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_3_3v));
      val = val * 5.0 / 4096.0;
      if(val < 3.1 || 3.5 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwr33v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwrVIn(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwrVIn" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_v_in"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_v_in));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_v_in));
      val = 11.0 * val * 5.0 / 4096.0;
      if(val < 8.0 || 19.0 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwrVIn();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckBotPwr125v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckBotPwr125v" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("volt_temp.bot.pwr_1_25v"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_25v));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_volt_temp_bot_pwr_1_25v));
      val = val * 5.0 / 4096.0;
      if(val < 1.0 || 1.4 < val){
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + " V");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetBotPwr125v();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckVhv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckVhv" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("vhv"));
//    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_vhv));
    double val = 0.0;
    try{
      val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, key_vhv));
      diagnostics.summary(level, boost::lexical_cast<std::string>(val) + "");
    }catch(boost::bad_lexical_cast &ex){
      diagnostics.summary(level, error_message);
    }
    */
    auto tpl = VelodyneGetVhv();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckAdcNf(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdcNf" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);

    std::ostringstream os;
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
    */
    auto tpl = VelodyneGetAdcNf();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckAdcStats(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdcStats" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);

    std::ostringstream os;

    boost::optional<boost::property_tree::ptree&> child = current_diag_tree->get_child_optional(key_adc_stats);
    if(child)
    {
      std::ostringstream os;
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
    */
    auto tpl = VelodyneGetAdcStats();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckIxe(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckIxe" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_diag_tree->get<std::string>("ixe"));
    diagnostics.summary(level, GetPtreeValue(current_diag_tree, key_ixe));
    */
    auto tpl = VelodyneGetIxe();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckAdctpStat(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckAdctpStat" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    /*
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
    */
    auto tpl = VelodyneGetAdctpStat();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::OnVelodyneStatusTimer()
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

void VelodyneHwMonitorRosWrapper::VelodyneCheckGpsPpsState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckGpsPpsState" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("gps.pps_state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_gps_pps_state));
    */
    auto tpl = VelodyneGetGpsPpsState();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckGpsPosition(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckGpsPosition" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("gps.position"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_gps_pps_position));
    */
    auto tpl = VelodyneGetGpsPosition();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckMotorState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorState" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("motor.state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_state));
    */
    auto tpl = VelodyneGetMotorState();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckMotorRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorRpm" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("motor.rpm"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_rpm));
    */
    auto tpl = VelodyneGetMotorRpm();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckMotorLock(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorLock" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("motor.lock"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_lock));
    */
    auto tpl = VelodyneGetMotorLock();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckMotorPhase(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckMotorPhase" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("motor.phase"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_motor_phase));
    */
    auto tpl = VelodyneGetMotorPhase();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckLaserState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckLaserState" << std::endl;
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    /*
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
//    diagnostics.summary(level, VelodyneHwMonitorRosWrapper::current_status_tree->get<std::string>("laser.state"));
    diagnostics.summary(level, GetPtreeValue(current_status_tree, key_status_laser_state));
    */
    auto tpl = VelodyneGetLaserState();
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckSnapshot(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
//  std::cout << "VelodyneCheckLaserState" << std::endl;
//  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
//    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    uint8_t level = current_diag_status;
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(level, *current_snapshot);
//  }
}



void VelodyneHwMonitorRosWrapper::OnVelodyneSnapshotTimer()
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




void VelodyneHwMonitorRosWrapper::VelodyneCheckStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetMotorState();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_state, std::get<2>(tpl));

    tpl = VelodyneGetLaserState();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_laser_state, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));

  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckPps(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if(VelodyneHwMonitorRosWrapper::current_status_tree && !VelodyneHwMonitorRosWrapper::current_status_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetGpsPpsState();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_gps_pps_state, std::get<2>(tpl));

    tpl = VelodyneGetGpsPosition();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_gps_pps_position, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));

  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetTopLm20Temp();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_lm20_temp, std::get<2>(tpl));

    tpl = VelodyneGetBotLm20Temp();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_lm20_temp, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));

  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetMotorRpm();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_rpm, std::get<2>(tpl));

    tpl = VelodyneGetMotorLock();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_lock, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));

  }
}

void VelodyneHwMonitorRosWrapper::VelodyneCheckVoltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if(VelodyneHwMonitorRosWrapper::current_diag_tree && !VelodyneHwMonitorRosWrapper::current_diag_tree->empty()){
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetTopHv();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_hv, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr5v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_5v, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr25v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_2_5v, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr33v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_3_3v, std::get<2>(tpl));

    if(sensor_configuration_.sensor_model == nebula::drivers::SensorModel::VELODYNE_VLP16){
      tpl = VelodyneGetTopPwr5vRaw();
      if(std::get<0>(tpl)){
        level = std::max(level, std::get<1>(tpl));
        if(0 < std::get<3>(tpl).length()){
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_5v_raw, std::get<2>(tpl));
    }else{
      tpl = VelodyneGetTopPwrRaw();
      if(std::get<0>(tpl)){
        level = std::max(level, std::get<1>(tpl));
        if(0 < std::get<3>(tpl).length()){
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_raw, std::get<2>(tpl));
    }

    tpl = VelodyneGetTopPwrVccint();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_vccint, std::get<2>(tpl));

    tpl = VelodyneGetBotIOut();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_i_out, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr12v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_1_2v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr5v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_5v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr25v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_2_5v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr33v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_3_3v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwrVIn();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_v_in, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr125v();
    if(std::get<0>(tpl)){
      level = std::max(level, std::get<1>(tpl));
      if(0 < std::get<3>(tpl).length()){
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_1_25v, std::get<2>(tpl));


    diagnostics.summary(level, boost::algorithm::join(msg, ", "));

  }
}

rcl_interfaces::msg::SetParametersResult VelodyneHwMonitorRosWrapper::paramCallback(
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
    /*
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
    */
    get_param(p, "diag_span", new_diag_span)) {
/*
    if(0 < sensor_model_str.length())
      new_param.sensor_model =
        nebula::drivers::SensorModelFromString(sensor_model_str);
    if(0 < return_mode_str.length())
      new_param.return_mode =
        nebula::drivers::ReturnModeFromString(return_mode_str);
    if(0 < return_mode_str.length())
      diag_span_ = new_diag_span;
*/
    sensor_configuration_ = new_param;
    // Update sensor_configuration
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::VelodyneSensorConfiguration>(sensor_configuration_);
    RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
//    hw_interface_.SetSensorConfiguration(
//      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
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
RCLCPP_COMPONENTS_REGISTER_NODE(VelodyneHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
