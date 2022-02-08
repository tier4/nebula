#include "velodyne/velodyne_driver_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
VelodyneDriverRosWrapper::VelodyneDriverRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::VelodyneCalibrationConfiguration calibration_configuration;
  drivers::VelodyneSensorConfiguration sensor_configuration;

  wrapper_status_ =
    GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::VelodyneCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::VelodyneSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);

  velodyne_scan_sub_ = create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&VelodyneDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  velodyne_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS());
}

void VelodyneDriverRosWrapper::ReceiveScanMsgCallback(
  const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg)
{
  // take packets out of scan msg
  std::vector<velodyne_msgs::msg::VelodynePacket> pkt_msgs = scan_msg->packets;

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
  velodyne_points_pub_->publish(std::move(ros_pc_msg_ptr));
}

Status VelodyneDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(
    std::static_pointer_cast<drivers::VelodyneSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::VelodyneCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status VelodyneDriverRosWrapper::GetStatus() { return wrapper_status_; }

Status VelodyneDriverRosWrapper::GetParameters(
  drivers::VelodyneSensorConfiguration & sensor_configuration,
  drivers::VelodyneCalibrationConfiguration & calibration_configuration)
{
  sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(
    this->declare_parameter<std::string>("sensor_model", ""));
  sensor_configuration.return_mode =
    nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", ""));
  sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "velodyne");
  sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0.);

  calibration_configuration.calibration_file = this->declare_parameter<std::string>("calibration_file", "");

  sensor_configuration.min_range = this->declare_parameter<double>("min_range", 0.3);
  sensor_configuration.max_range = this->declare_parameter<double>("max_range", 300.);
  double view_direction = sensor_configuration.scan_phase * M_PI / 180;
  double view_width = this->declare_parameter<double>("view_width", 360) * M_PI / 180;
  double min_angle = fmod(fmod(view_direction + view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  double max_angle = fmod(fmod(view_direction - view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  sensor_configuration.cloud_min_angle = 100 * (2 * M_PI - min_angle) * 180 / M_PI + 0.5;
  sensor_configuration.cloud_max_angle = 100 * (2 * M_PI - max_angle) * 180 / M_PI + 0.5;
  if (sensor_configuration.cloud_min_angle == sensor_configuration.cloud_max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    sensor_configuration.cloud_min_angle = 0;
    sensor_configuration.cloud_max_angle = 36000;
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (
    sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
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

  RCLCPP_INFO_STREAM(this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model <<
                     ", Return mode: " << sensor_configuration.return_mode <<
                     ", Scan Phase: " << sensor_configuration.scan_phase);
  return Status::OK;
}

}
}