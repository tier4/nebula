#include "ros2_driver_wrapper.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <fstream>
#include <iostream>

namespace lidar_driver
{
/// @brief constructor
RosDriverWrapper::RosDriverWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options), driver_(), diagnostics_updater_(this)
{
  future_ = exit_signal_.get_future();

  this->declare_parameter("sensor_type", "livox");
  this->declare_parameter("frame_id", "frameid_default");
  this->declare_parameter("debug_mode", 0);
  this->declare_parameter("debug_str1", "");
  this->declare_parameter("debug_str2", "test");

  this->get_parameter("sensor_type", config_.sensor_type);
  this->get_parameter("frame_id", config_.frame_id);
  this->get_parameter("debug_mode", config_.debug_mode);
  this->get_parameter("debug_str1", config_.debug_str1);
  this->get_parameter("debug_str2", config_.debug_str2);

  InitializeLivoxDiagnostics();

  //config_.npackets = static_cast<int>(std::ceil(packet_rate / frequency));  // velodyne
  config_.npackets = 1;

  main_thread_ = std::thread(&RosDriverWrapper::MainThread, this);
}

/// @brief destructor
RosDriverWrapper::~RosDriverWrapper()
{
  Shutdown();
  main_thread_.join();
}

/// @brief Shutdown
void RosDriverWrapper::Shutdown() { exit_signal_.set_value(); }

/// @brief StreamStop
void RosDriverWrapper::StreamStop()
{
  RCLCPP_INFO(this->get_logger(), "StopHwRxInterface: StreamStop request.");
  driver_.StopHwRxInterface();  // stream stop request.
  driver_.StopHwTxInterface();  // stop heart beat.
}

/// @brief StartHwTxInterface
bool RosDriverWrapper::StartInterface()
{
  bool ret = false;
  CommandResult pre_result = CommandResult::kAck;
  CommandResult cmd_result;
  int wait_ms = 0;
  std::future_status status = std::future_status::timeout;

  do {
    cmd_result = driver_.StartHwTxInterface();
    if (cmd_result == CommandResult::kAck) {
      ret = true;
      RCLCPP_INFO(this->get_logger(), "StartHwTxInterface: Handshake success.");
      break;
    } else if (cmd_result == CommandResult::kTimeout) {
      // Timeout.
      if (pre_result != cmd_result) {
        pre_result = cmd_result;
        RCLCPP_WARN(this->get_logger(), "StartHwTxInterface: Handshake Timeout");
      }
      wait_ms = 400;
    } else if (cmd_result == CommandResult::kNack) {
      // Ack fail.
      if (pre_result != cmd_result) {
        pre_result = cmd_result;
        RCLCPP_WARN(this->get_logger(), "StartHwTxInterface: Handshake Ack fail.");
      }
      wait_ms = 500;
    } else {
      // Socket Error.
      RCLCPP_ERROR(this->get_logger(), "Sensor Socket error !");
      break;
    }
    status = future_.wait_for(std::chrono::milliseconds(wait_ms));
  } while (status == std::future_status::timeout);

  if (status != std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "StartHwTxInterface Cancel %d", static_cast<int>(cmd_result));
    ret = false;
  }

  return ret;
}

/// @brief publisher subscriber callback function entry.
void RosDriverWrapper::CreatePubSub()
{
  // 2. Wait Point Cloud. It is defined as the function that will wait and receive the processed
  // point cloud by the parser inside LidarDriver.
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu_packet", 16);

  driver_scan_ = this->create_publisher<livox_msgs::msg::LidarScan>(
    "livox/driver_scan_topic", rclcpp::SensorDataQoS());

  // advertise output point cloud (before subscribing to input data)
  point_cloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/cloud", rclcpp::SensorDataQoS());

  // 3. Launch LidarDriver. It creates a LidarDriver instance of the requested sensor with the
  // specified configuration, and registers the WaitPointCloud callback.
  // This interface will register the callback WaitDataPacketFromHwRx that will receive and publish the raw data to the ROS system.
  driver_.SetPublishLidarDataFunc(std::bind(
    &RosDriverWrapper::PublishLidarData, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4));
  driver_.SetPublishImuPacketFunc(std::bind(
    &RosDriverWrapper::PublishImuPacket, this, std::placeholders::_1, std::placeholders::_2));

  // 4. WaitRosConfig. It registers the ROS services that will be called to forward the configure requests to the sensor.
  // ToDo: Next Phase

  // 5. RosLidarDataSubscriber (a nodelet in ROS1). It subscribes to a specified ROS topic to listen
  //    to the data packets generated by the HwRxInterface, or the data received from previously recorded log data.
  subscriber_ = this->create_subscription<livox_msgs::msg::LidarScan>(
    "livox/driver_scan_topic", rclcpp::SensorDataQoS(),
    std::bind(&RosDriverWrapper::RosLidarDataSubscriber, this, std::placeholders::_1));

  return;
}

/// @brief StreamStart
bool RosDriverWrapper::StreamStart()
{
  /// InstantiateLidarDriver
  /// Instantiates the corresponding Lidar Driver for the selected sensor.

  // Initiates the corresponding hardware interface defined in the sensor for data reception.
  if (!StartInterface()) {
    return false;
  }

#if 0   // TODO(Next phase)
    driver_.SetCalibration();
    driver_.ConfigureSensor();
    driver_.GetSensorConfig( sensor_configuration_ );
#endif  // TODO(Next phase)

  bool ret = driver_.StartHwRxInterface();
  if (ret) {
    RCLCPP_INFO(this->get_logger(), "StartHwRxInterface: StreamStart request.");
  } else {
    driver_.StopHwTxInterface();
    RCLCPP_ERROR(this->get_logger(), "StartHwRxInterface: StreamStart failed.");
  }

  return ret;
}

/// @brief Main thread
void RosDriverWrapper::MainThread()
{
  bool running = false;

  // yaml get_parameter
  GetParameter();

  // 1. Check Configuration.
  if (!CheckConfiguration(sensor_config_ex_, cloud_configuration_)) {
  }
  // It ensures the requested configuration applied for the selected sensor.
  else if (!driver_.SetConfiguration(sensor_config_ex_.sensor_config)) {
    RCLCPP_ERROR(this->get_logger(), "SetSensorConfiguration Error !");
  } else if (!driver_.SetConfiguration(cloud_configuration_)) {
    RCLCPP_ERROR(this->get_logger(), "SetCloudConfiguration Error !");
  }
  // 6. LaunchRosHwRxInterface (a nodelet in ROS1). It launches the RosHwRxInterface that will
  // connect to the sensor, if available, and publish the received data to the specified topic
  // being listened to by the RosLidarDataSubscriber.
  else {
    CreatePubSub();
    running = StreamStart();
  }

  if (!running) {
    rclcpp::shutdown();
  } else {
    std::future_status status;

    do {
      status = future_.wait_for(std::chrono::seconds(3));
    } while (status == std::future_status::timeout);

    StreamStop();

    RCLCPP_INFO(this->get_logger(), "LidarDriver Ros2Wrapper MainThread() end %d", (int)status);
  }

  return;
}

/// @brief imu packet publish.
void RosDriverWrapper::PublishImuPacket(
  const livox_driver::LivoxImuPoint & imu_pkt, uint64_t time_stamp)
{
  sensor_msgs::msg::Imu imu_data;
  imu_data.header.frame_id = config_.frame_id;
  if (time_stamp != 0) {
    imu_data.header.stamp = rclcpp::Time(time_stamp);  // to ros time stamp
  }
  imu_data.angular_velocity.x = imu_pkt.gyro_x;
  imu_data.angular_velocity.y = imu_pkt.gyro_y;
  imu_data.angular_velocity.z = imu_pkt.gyro_z;
  imu_data.linear_acceleration.x = imu_pkt.acc_x;
  imu_data.linear_acceleration.y = imu_pkt.acc_y;
  imu_data.linear_acceleration.z = imu_pkt.acc_z;
  imu_pub_->publish(imu_data);

  return;
}

/// Figure 10.
/// @brief Publishes the Raw data in packet format to the ROS System.
/// @details WaitDataPacketFromRx
/// Callback that receives the raw data packets from the LidarDriver.
void RosDriverWrapper::PublishLidarData(
  const std::vector<uint8_t> & buff, int pkt_len, uint64_t time_stamp, uint32_t point_num)
{
  if (point_num == 0) {
    return;
  }

  livox_msgs::msg::LidarScan scan_msg;
  scan_msg.packets.resize(config_.npackets);
  scan_msg.packets[0].data.resize(pkt_len);
  scan_msg.header.frame_id = config_.frame_id;
  if (time_stamp != 0) {
    scan_msg.header.stamp = rclcpp::Time(time_stamp);
    scan_msg.packets[0].stamp = rclcpp::Time(time_stamp);
  }
  scan_msg.packets[0].time_stamp = time_stamp;
  scan_msg.packets[0].point_num = point_num;
  scan_msg.packets[0].size = pkt_len;
  scan_msg.packets[0].data = std::move(buff);
  driver_scan_->publish(scan_msg);

  return;
}

/// @brief set PointCloud2 message header.
void RosDriverWrapper::InitPointcloud2MsgHeader(sensor_msgs::msg::PointCloud2 & cloud)
{
  cloud.header.frame_id.assign(config_.frame_id);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(6);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;

  cloud.point_step = sizeof(livox_driver::LivoxPointXyzrtl);
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  return;
}

/// @brief PublishCloud
void RosDriverWrapper::PublishCloud(int data_cnt)
{
  sensor_msgs::msg::PointCloud2 cloud;
  InitPointcloud2MsgHeader(cloud);

  cloud.data.resize(data_cnt * cloud.point_step);

  std::unique_ptr<livox_driver::LivoxPublishData> pub_ptr = driver_.GenerateCloud();
  if (pub_ptr->time != 0) {
    cloud.header.stamp = rclcpp::Time(pub_ptr->time);
  }
  cloud.width = pub_ptr->num;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data = std::move(pub_ptr->data);

  cloud.data.resize(cloud.row_step); /** Adjust to the real size */

  point_cloud_->publish(cloud);

  return;
}

/// @brief subscribes to a specified ROS topic to listen to the data packets.
/// @param scan_msg: data packets
/// @details
/// 5. RosLidarDataSubscriber (a nodelet in ROS1). It subscribes to a specified ROS topic to listen
/// to the data packets generated by the HwRxInterface, or the data received from previously recorded log data.
void RosDriverWrapper::RosLidarDataSubscriber(const livox_msgs::msg::LidarScan::SharedPtr scan_msg)
{
  if (
    point_cloud_->get_subscription_count() == 0 &&
    point_cloud_->get_intra_process_subscription_count() == 0)  // no one listening?
  {
    return;  // avoid much work
  }

  // 7. RosPublishPointCloud. It will publish the final point cloud generated by the GeneratePointCloud module.
  for (size_t ii = 0; ii < scan_msg->packets.size(); ii++) {
    livox_driver::LivoxLidarPacket lidar_packet;
    lidar_packet.time_stamp = scan_msg->packets[ii].time_stamp;
    lidar_packet.data_cnt = scan_msg->packets[ii].point_num;
    lidar_packet.data = std::move(scan_msg->packets[ii].data);

    int32_t data_cnt = driver_.ParsePacket(lidar_packet);
    if (data_cnt > 0) {
      PublishCloud(data_cnt);
    }
  }  //for

  return;
}

void RosDriverWrapper::InitializeLivoxDiagnostics()
{
  using std::chrono_literals::operator""s;
  diagnostics_updater_.setHardwareID(config_.sensor_type + "_" + config_.frame_id);
  diagnostics_updater_.add(
      "livox_temperature-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckTemperature);
  diagnostics_updater_.add(
      "livox_internal_voltage-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckVoltage);
  diagnostics_updater_.add(
      "livox_motor_status-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckMotor);
  diagnostics_updater_.add(
      "livox_optical_window-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckWindowDirt);
  diagnostics_updater_.add(
      "livox_firmware_status-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckFirmware);
  diagnostics_updater_.add(
      "livox_pps_signal-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckPps);
  diagnostics_updater_.add(
      "livox_service_life-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckServiceLife);
  diagnostics_updater_.add(
      "livox_fan_status-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckFan);
  diagnostics_updater_.add(
      "livox_time_sync-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckTimeSync);
  diagnostics_updater_.add(
      "livox_connection-" + config_.frame_id,
      this, &RosDriverWrapper::LivoxCheckConnection);

  auto on_timer = [this] { OnLivoxDiagnosticsTimer(); };
  diagnostics_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
    this->get_clock(), 1s, std::move(on_timer), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_timer_, nullptr);
}

void RosDriverWrapper::OnLivoxDiagnosticsTimer()
{
  uint32_t lidar_status_code;
  if (driver_.GetLidarStatusCode(lidar_status_code)) {
    current_diagnostics_status_ =
      livox_driver::diagnostics::StatusCodeToLivoxSensorStatus(lidar_status_code);
  }
  diagnostics_updater_.force_update();
}

void RosDriverWrapper::LivoxCheckConnection(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  LidarDriver::LidarDriverStatus lidar_driver_status = driver_.GetLidarStatus();
  switch (lidar_driver_status) {
    case LidarDriver::LidarDriverStatus::kDisconnect:
      level = DiagStatus::ERROR;
      break;
    case LidarDriver::LidarDriverStatus::kConnect:
      level = DiagStatus::WARN;
      break;
    case LidarDriver::LidarDriverStatus::kStreaming:
      level = DiagStatus::OK;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, driver_.lidar_driver_status_dict_.at(lidar_driver_status));
}

void RosDriverWrapper::LivoxCheckTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Temp = livox_driver::diagnostics::LivoxTemperatureStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.temperature_status
      : Temp::kUnknown;
  switch (status) {
    case Temp::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case Temp::kNormal:
      level = DiagStatus::OK;
      break;
    case Temp::kHighLow:
      level = DiagStatus::WARN;
      break;
    case Temp::kExtremelyHighLow:
      level = DiagStatus::ERROR;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_temperature_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckVoltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Volt = livox_driver::diagnostics::LivoxVoltageStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.voltage_status
      : Volt::kUnknown;
  switch (status) {
    case Volt::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case Volt::kNormal:
      level = DiagStatus::OK;
      break;
    case Volt::kHigh:
      level = DiagStatus::WARN;
      break;
    case Volt::kExtremelyHigh:
      level = DiagStatus::ERROR;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_voltage_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckMotor(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Motor = livox_driver::diagnostics::LivoxMotorStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.motor_status
      : Motor::kUnknown;
  switch (status) {
    case Motor::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case Motor::kNormal:
      level = DiagStatus::OK;
      break;
    case Motor::kWarning:
      level = DiagStatus::WARN;
      break;
    case Motor::kError:
      level = DiagStatus::ERROR;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_motor_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckWindowDirt(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using WindowDirt = livox_driver::diagnostics::LivoxDirtStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.dirt_status
      : WindowDirt::kUnknown;
  switch (status) {
    case WindowDirt::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case WindowDirt::kNotDirty:
      level = DiagStatus::OK;
      break;
    case WindowDirt::kDirty:
      level = DiagStatus::WARN;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_dirt_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckFirmware(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Firmware = livox_driver::diagnostics::LivoxFirmwareStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.firmware_status
      : Firmware::kUnknown;
  switch (status) {
    case Firmware::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case Firmware::kOk:
      level = DiagStatus::OK;
      break;
    case Firmware::kAbnormal:
      level = DiagStatus::WARN;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_firmware_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckTimeSync(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using TimeSync = livox_driver::diagnostics::LivoxTimeSyncStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.time_sync_status
      : TimeSync::kUnknown;
  switch (status) {
    case TimeSync::kUnknown:
    case TimeSync::kNoSync:
    case TimeSync::kAbnormal:
      level = DiagStatus::ERROR;
      break;
    case TimeSync::kUsingPtp:
    case TimeSync::kUsingGps:
    case TimeSync::kUsingPps:
      level = DiagStatus::OK;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_timesync_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckServiceLife(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using ServiceLife = livox_driver::diagnostics::LivoxDeviceLifeStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.device_life_status
      : ServiceLife::kUnknown;
  switch (status) {
    case ServiceLife::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case ServiceLife::kNormal:
      level = DiagStatus::OK;
      break;
    case ServiceLife::kWarning:
      level = DiagStatus::WARN;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_life_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckFan(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Fan = livox_driver::diagnostics::LivoxFanStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.fan_status
      : Fan::kUnknown;
  switch (status) {
    case Fan::kUnknown:
      level = DiagStatus::ERROR;
      break;
    case Fan::kNormal:
      level = DiagStatus::OK;
      break;
    case Fan::kWarning:
      level = DiagStatus::WARN;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_fan_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckPtp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Ptp = livox_driver::diagnostics::LivoxPtpStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.ptp_status
      : Ptp::kUnknown;
  switch (status) {
    case Ptp::kNoSignal:
    case Ptp::kUnknown:
      level = DiagStatus::WARN;
      break;
    case Ptp::kSignalOk:
      level = DiagStatus::OK;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_ptp_dict_.at(status));
}

void RosDriverWrapper::LivoxCheckPps(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level;
  using Pps = livox_driver::diagnostics::LivoxPpsStatus;
  auto status = (LidarDriver::LidarDriverStatus::kDisconnect != driver_.GetLidarStatus())
      ? current_diagnostics_status_.pps_status
      : Pps::kUnknown;
  switch (status) {
    case Pps::kUnknown:
    case Pps::kNoSignal:
      level = DiagStatus::WARN;
      break;
    case Pps::kSignalOk:
      level = DiagStatus::OK;
      break;
  }
  diagnostics.add("sensor", config_.frame_id);
  diagnostics.summary(level, livox_driver::diagnostics::livox_pps_dict_.at(status));
}

}  // namespace lidar_driver
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_driver::RosDriverWrapper)