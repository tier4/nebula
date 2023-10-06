#include "nebula_ros/robosense/robosense_decoder_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
RobosenseDriverRosWrapper::RobosenseDriverRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_driver_ros_wrapper", options), hw_interface_()
{
  RCLCPP_WARN_STREAM(this->get_logger(), "RobosenseDriverRosWrapper");
  drivers::RobosenseCalibrationConfiguration calibration_configuration;
  drivers::RobosenseSensorConfiguration sensor_configuration;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::RobosenseCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeInfoDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);

  robosense_scan_sub_ = create_subscription<robosense_msgs::msg::RobosenseScan>(
    "robosense_packets", rclcpp::SensorDataQoS(),
    std::bind(&RobosenseDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  robosense_info_sub_ = create_subscription<robosense_msgs::msg::RobosensePacket>(
    "robosense_difop_packets", rclcpp::SensorDataQoS(),
    std::bind(&RobosenseDriverRosWrapper::ReceiveInfoMsgCallback, this, std::placeholders::_1));
  nebula_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "robosense_points", rclcpp::SensorDataQoS());
  aw_points_base_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", rclcpp::SensorDataQoS());
  aw_points_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", rclcpp::SensorDataQoS());

  RCLCPP_WARN_STREAM(this->get_logger(), "Initialized decoder ros wrapper.");
}

void RobosenseDriverRosWrapper::ReceiveScanMsgCallback(
  const robosense_msgs::msg::RobosenseScan::SharedPtr scan_msg)
{
  if (!is_received_info) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for info packet.");
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts =
    driver_ptr_->ConvertScanToPointcloud(scan_msg);
  nebula::drivers::NebulaPointCloudPtr pointcloud = std::get<0>(pointcloud_ts);

  if (pointcloud == nullptr) {
    RCLCPP_WARN_STREAM(get_logger(), "Empty cloud parsed.");
    return;
  };
  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud = nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  auto runtime = std::chrono::high_resolution_clock::now() - t_start;
  RCLCPP_DEBUG(
    get_logger(), "PROFILING {'d_total': %lu, 'n_out': %lu}", runtime.count(), pointcloud->size());
}

void RobosenseDriverRosWrapper::ReceiveInfoMsgCallback(
  const robosense_msgs::msg::RobosensePacket::SharedPtr info_msg)
{
  if (!sensor_cfg_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Sensor configuration has not been initialized yet.");
    return;
  }
  if (info_msg->data.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Empty info packet received.");
    return;
  }

  std::vector<uint8_t> info_data(info_msg->data.begin(), info_msg->data.end());
  const auto decode_status = info_driver_ptr_->DecodeInfoPacket(info_data);
  if (decode_status != Status::OK) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to decode DIFOP packet.");
    return;
  }

  sensor_cfg_ptr_->return_mode = info_driver_ptr_->GetReturnMode();
  *calibration_cfg_ptr_ = info_driver_ptr_->GetSensorCalibration();
  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << *sensor_cfg_ptr_);

  wrapper_status_ = InitializeDriver(sensor_cfg_ptr_, calibration_cfg_ptr_);
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);

  is_received_info = true;

  // Unsubscribe from info topic
  robosense_info_sub_.reset();
}

void RobosenseDriverRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
    pointcloud->header.stamp = this->now();
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status RobosenseDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing driver...");
  driver_ptr_ = std::make_shared<drivers::RobosenseDriver>(
    std::static_pointer_cast<drivers::RobosenseSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::RobosenseCalibrationConfiguration>(
      calibration_configuration));

  return driver_ptr_->GetStatus();
}

Status RobosenseDriverRosWrapper::InitializeInfoDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing info driver...");
  info_driver_ptr_ = std::make_shared<drivers::RobosenseInfoDriver>(
    std::static_pointer_cast<drivers::RobosenseSensorConfiguration>(sensor_configuration));

  return info_driver_ptr_->GetStatus();
}

Status RobosenseDriverRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status RobosenseDriverRosWrapper::GetParameters(
  drivers::RobosenseSensorConfiguration & sensor_configuration,
  drivers::RobosenseCalibrationConfiguration & calibration_configuration)
{
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
    this->declare_parameter<uint16_t>("gnss_port", 7788, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
    calibration_configuration.SetChannelSize(
      nebula::drivers::GetChannelSize(sensor_configuration.sensor_model));
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);

    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringRobosense(
      this->get_parameter("return_mode").as_string());
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "robosense", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (calibration_configuration.calibration_file.empty()) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Empty Calibration_file File: '" << calibration_configuration.calibration_file << "'");
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status =
      calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);

    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Load calibration data from: '" << calibration_configuration.calibration_file << "'");
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

std::string RobosenseDriverRosWrapper::CreateCalibrationPath(const std::string & original_path)
{
  // New suffix to add
  const std::string suffix = "_from_sensor.csv";

  // Find the last occurrence of '.' to get the directory path
  size_t last_pos = original_path.find_last_of('.');
  if (last_pos != std::string::npos) {
    // Extract the path
    const std::string calib_path = original_path.substr(0, last_pos);

    // Create the new file path by appending the new filename
    std::string new_path = calib_path + suffix;

    // Print the new file path
    RCLCPP_INFO_STREAM(this->get_logger(), "New calibration file path: " << new_path);
    return new_path;

  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid file path format.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseDriverRosWrapper)
}  // namespace ros
}  // namespace nebula
