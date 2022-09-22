#include "hesai/hesai_hw_interface_ros_wrapper.hpp"


#include "tcp_driver/tcp_driver.hpp"

#include <boost/asio.hpp>
//#include <boost/thread/thread.hpp>
#include <thread>

//#define WITH_DEBUG_STDOUT_HesaiHwInterfaceRosWrapper
//#define TEST_PCAP

namespace nebula
{
namespace ros
{
HesaiHwInterfaceRosWrapper::HesaiHwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options), hw_interface_()
//  m_owned_ios{new boost::asio::io_service(1)},
//  m_tcp_driver{new ::drivers::tcp_driver::TcpDriver(m_owned_ios)}
{
//  interface_status_ = GetParameters(sensor_configuration_);
  if(mtx_config_.try_lock()){
    interface_status_ = GetParameters(sensor_configuration_);
    mtx_config_.unlock();
  }
  if (Status::OK != interface_status_)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
//  hw_interface_.SetNode(std::make_shared<nebula::ros::HesaiHwInterfaceRosWrapper>(this));
//  hw_interface_.SetNode(std::make_shared<rclcpp::Node>(this));
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
//  hw_interface_.SetNode(this);
  // Initialize sensor_configuration
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
#if not defined(TEST_PCAP)
  hw_interface_.InitializeTcpDriver();
  
  std::vector<std::thread> thread_pool{};
  thread_pool.emplace_back([this]{
//      auto ios = std::make_shared<boost::asio::io_service>();
      hw_interface_.GetInventory(//ios,
      [this](HesaiInventory &result)
      {
        std::cout << result << std::endl;
        hw_interface_.SetTargetModel(result.model);
      });
    });
  for (std::thread &th : thread_pool) {
    th.join();
  }
#ifdef WITH_DEBUG_STDOUT_HesaiHwInterfaceRosWrapper
  std::cout << "hw_interface_.CheckAndSetConfig();";
#endif
  hw_interface_.CheckAndSetConfig();
  updateParameters();
#endif

  // register scan callback and publisher
  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  pandar_scan_pub_ =
    this->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());

#if not defined(TEST_PCAP)
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&HesaiHwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));
#endif

  /*
  hw_interface_.GetInventory();
  hw_interface_.IOServiceRun();
  hw_interface_.GetConfig();
  hw_interface_.IOServiceRun();
  hw_interface_.GetLidarStatus();
  hw_interface_.IOServiceRun();
  */
#ifdef WITH_DEBUG_STDOUT_HesaiHwInterfaceRosWrapper
  if(false)
  {
    std::vector<std::thread> thread_pool{};
    thread_pool.emplace_back([this]{
        hw_interface_.SetStandbyMode(0);
  //      auto ios = std::make_shared<boost::asio::io_service>();
  //      hw_interface_.SetStandbyMode(ios, 0);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetLidarCalib(ios);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetPtpDiagStatus(ios);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetPtpDiagPort(ios);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetPtpDiagTime(ios);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetPtpDiagGrandmaster(ios);
      });
  //  thread_pool.emplace_back([&hw_interface_]{
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetInventory(ios);
  //      hw_interface_.IOServiceRun();
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetConfig(ios);
  //      hw_interface_.IOServiceRun();
      });
    thread_pool.emplace_back([this]{
  //      auto ios = std::make_shared<boost::asio::io_service>(new boost::asio::io_service(1));
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetLidarStatus(ios);
  //      hw_interface_.IOServiceRun();
      });
    thread_pool.emplace_back([this]{
        hw_interface_.SetStandbyMode(1);
  //      auto ios = std::make_shared<boost::asio::io_service>();
  //      hw_interface_.SetStandbyMode(ios, 1);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetPtpConfig(ios);
      });
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        hw_interface_.GetLidarRange(ios);
      });
    for (std::thread &th : thread_pool) {
  //    hw_interface_.IOServiceRun();
      th.join();
    }
  }
  if(false)
  {
    std::vector<std::thread> thread_pool{};
    thread_pool.emplace_back([this]{
        auto ios = std::make_shared<boost::asio::io_service>();
        std::cout << "GetLidarCalib" << std::endl;
        hw_interface_.GetLidarCalib(ios);
      });
    for (std::thread &th : thread_pool) {
  //    hw_interface_.IOServiceRun();
      th.join();
    }
  }
#endif
}

Status HesaiHwInterfaceRosWrapper::StreamStart()
{
  if(Status::OK == interface_status_ ){
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
//  sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(
//    this->declare_parameter<std::string>("sensor_model", ""));
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
//  sensor_configuration.return_mode =
//    nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", ""));
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
  }
//  sensor_configuration.host_ip = this->declare_parameter<std::string>("host_ip", "255.255.255.255");
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
//  sensor_configuration.sensor_ip =
//    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201");
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
//  sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "pandar");
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "pandar", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
//  sensor_configuration.data_port = this->declare_parameter<uint16_t>("data_port", 2368);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
//  sensor_configuration.gnss_port = this->declare_parameter<uint16_t>("gnss_port", 2369);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
//  sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0.);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range= {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
//  sensor_configuration.frequency_ms = this->declare_parameter<uint16_t>("frequency_ms", 100);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("frequency_ms", 100, descriptor);
    sensor_configuration.frequency_ms = this->get_parameter("frequency_ms").as_int();
  }
//  sensor_configuration.packet_mtu_size = this->declare_parameter<uint16_t>("packet_mtu_size", 1500);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = this->get_parameter("packet_mtu_size").as_int();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::HESAI_PANDARAT128){
      descriptor.additional_constraints = "200, 300, 400, 500";
      range.set__from_value(200).set__to_value(500).set__step(100);
      descriptor.integer_range= {range};
      this->declare_parameter<uint16_t>("rotation_speed", 200, descriptor);
    }else{
      descriptor.additional_constraints = "300, 600, 1200";
      range.set__from_value(300).set__to_value(1200).set__step(300);
      descriptor.integer_range= {range};
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range= {range};
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
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range= {range};
    this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
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

  /*
  std::shared_ptr<::drivers::tcp_driver::TcpSocket> m_socket;
//  m_socket.reset(new ::drivers::tcp_driver::TcpSocket(m_owned_ios, sensor_configuration.sensor_ip, sensor_configuration.data_port, sensor_configuration.host_ip, sensor_configuration.data_port));
  m_socket.reset(new ::drivers::tcp_driver::TcpSocket(m_owned_ios, sensor_configuration.sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration.host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT));
  m_socket->open();
  */

   /* Convert std::string --> boost::asio::streambuf */
   /*
   boost::asio::streambuf sbuf;
   std::iostream os(&sbuf);
//   std::string message("Teststring");
//   os << message;
  os << (uint8_t)0x47;//Protocol identifier
  os << (uint8_t)0x74;//Protocol identifier
  os << (uint8_t)0x1c;//Cmd PTC_COMMAND_SET_STANDBY_MODE
  os << (uint8_t)0x00;//Return Code
//  os << (uint8_t)0x00;//Payload Length(1/4)
//  os << (uint8_t)0x00;//Payload Length(2/4)
//  os << (uint8_t)0x00;//Payload Length(3/4)
//  os << (uint8_t)0x01;//Payload Length(4/4)
  os << (uint8_t)0x01;//Payload Length(1/4)
  os << (uint8_t)0x00;//Payload Length(2/4)
  os << (uint8_t)0x00;//Payload Length(3/4)
  os << (uint8_t)0x00;//Payload Length(4/4)
  os << (uint8_t)0x01;//Payload
  std::vector<unsigned char> buf_vec;
  */
  /*
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x1c);//Cmd PTC_COMMAND_SET_STANDBY_MODE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
//  buf_vec.emplace_back(0x01);
  buf_vec.emplace_back(0x01);

//  std::string str((std::istreambuf_iterator<char>(&sbuf)),
//                  std::istreambuf_iterator<char>());
//  std::cout << "m_socket->send: " << str << std::endl;
//  m_socket->send(sbuf);
//  m_socket->send(boost::asio::buffer(buf_vec, sizeof(buf_vec)));
//  m_socket->send(buf_vec);
  m_socket->asyncSend(buf_vec);
  */
 /*
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x08);//Cmd PTC_COMMAND_GET_CONFIG_INFO
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
*/
/*
  m_socket->asyncReceive(
  [this](const std::vector<uint8_t> & received_bytes)
  {
    for(const auto &b :received_bytes){
      std::cout << b << ", ";
    }
    std::cout << std::endl;
  });
  */
 /*
  m_socket->asyncSendReceiveHesai(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
  });
  m_owned_ios->run();
  */
  //*/
 /*
//  char* buffer;
  unsigned char buffer[128];
  int index = 0;
  int len = 1;
  int cmd = 1;
  buffer[index++] = 0x47;
  buffer[index++] = 0x74;
  buffer[index++] = 0x1c;
//  buffer[index++] = 0x1e;
  buffer[index++] = 0x00;  // color or mono
  buffer[index++] = (len >> 24) & 0xff;
  buffer[index++] = (len >> 16) & 0xff;
  buffer[index++] = (len >> 8) & 0xff;
  buffer[index++] = (len >> 0) & 0xff;
  buffer[index++] = 0x01;
//  buffer[index++] = 0;
//  boost::asio::streambuf sbuf;
//  std::iostream os(&sbuf);
//  os << buffer;
//  m_socket->send(sbuf);
  m_socket->send(boost::asio::buffer(buffer, 128));
  */
  
  /*
  boost::asio::io_service io_service;
  boost::asio::ip::tcp::socket socket(io_service);
  boost::system::error_code error;
  socket.connect(tcp::endpoint(boost::asio::ip::address::from_string(sensor_configuration.sensor_ip), PANDARGENERALSDK_TCP_COMMAND_PORT), error);
  if (error) {
      std::cout << "connect failed : " << error.message() << std::endl;
  }
  else {
      std::cout << "connected" << std::endl;
  }
  boost::asio::write(socket, boost::asio::buffer(buffer, 128), error);
  if (error) {
      std::cout << "send failed: " << error.message() << std::endl;
  }
  else {
      std::cout << "send correct!" << std::endl;
  }
  */

/*
  RCLCPP_INFO_STREAM(this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model <<
                                           ", Return mode: " << sensor_configuration.return_mode <<
                                           ", Scan Phase: " << sensor_configuration.scan_phase);
*/
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
#ifdef WITH_DEBUG_STDOUT_HesaiHwInterfaceRosWrapper
  std::cout << "add_on_set_parameters_callback" << std::endl;
  std::cout << p << std::endl;
  std::cout << sensor_configuration_ << std::endl;
#endif
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::HesaiSensorConfiguration new_param{sensor_configuration_};
//  std::cout << new_param << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;
  std::string return_mode_str;
//  uint16_t new_diag_span = 0;
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
    get_param(p, "cloud_max_angle", new_param.cloud_max_angle))
    {

    if(0 < sensor_model_str.length())
      new_param.sensor_model =
        nebula::drivers::SensorModelFromString(sensor_model_str);
    if(0 < return_mode_str.length())
      new_param.return_mode =
        nebula::drivers::ReturnModeFromString(return_mode_str);

    sensor_configuration_ = new_param;
    // Update sensor_configuration
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
#ifdef WITH_DEBUG_STDOUT_HesaiHwMonitorRosWrapper
    RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
#endif
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
    hw_interface_.CheckAndSetConfig();
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

#ifdef WITH_DEBUG_STDOUT_HesaiHwMonitorRosWrapper
  std::cout << "add_on_set_parameters_callback success" << std::endl;
#endif

//  return result;
  return *result;
}

std::vector<rcl_interfaces::msg::SetParametersResult> HesaiHwInterfaceRosWrapper::updateParameters(){
  std::scoped_lock lock(mtx_config_);
#ifdef WITH_DEBUG_STDOUT_HesaiHwMonitorRosWrapper
  std::cout << "!!!!!!!!!!!updateParameters!!!!!!!!!!!!" << std::endl;
#endif
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  std::ostringstream os_return_mode;
  os_return_mode << sensor_configuration_.return_mode;
//  std::cout << "set_parameters start" << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");
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
    rclcpp::Parameter("cloud_max_angle", sensor_configuration_.cloud_max_angle)
  });
#ifdef WITH_DEBUG_STDOUT_HesaiHwMonitorRosWrapper
  std::cout << "set_parameters fin" << std::endl;
#endif
  return results;
}

}  // namespace ros
}  // namespace nebula
