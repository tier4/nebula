#include "hesai/hesai_driver_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiDriverRosWrapper::HesaiDriverRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::HesaiCalibrationConfiguration calibration_configuration;
  drivers::HesaiSensorConfiguration sensor_configuration;

  wrapper_status_ =
    GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
  pandar_scan_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
    "pandar_packets", rclcpp::SensorDataQoS(),
    std::bind(&HesaiDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  pandar_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::SensorDataQoS());
}

void HesaiDriverRosWrapper::ReceiveScanMsgCallback(
  const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  // take packets out of scan msg
  std::vector<pandar_msgs::msg::PandarPacket> pkt_msgs = scan_msg->packets;

  nebula::drivers::PointCloudXYZIRADTPtr pointcloud =
    driver_ptr_->ConvertScanToPointcloud(scan_msg);

  auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
  if (!pointcloud->points.empty()) {
    double first_point_timestamp = pointcloud->points.front().time_stamp;
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(first_point_timestamp).count());
  }
  ros_pc_msg_ptr->header.frame_id = sensor_cfg_ptr_->frame_id;
  pandar_points_pub_->publish(std::move(ros_pc_msg_ptr));
}

Status HesaiDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status HesaiDriverRosWrapper::GetStatus() { return wrapper_status_; }

Status HesaiDriverRosWrapper::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration)
{
  sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(
    this->declare_parameter<std::string>("sensor_model", ""));

  sensor_configuration.return_mode =
    nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", ""));
  sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "pandar");
  sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0.);

  calibration_configuration.calibration_file =
    this->declare_parameter<std::string>("calibration_file", "");

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
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status =
      calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

}  // namespace ros
}  // namespace nebula