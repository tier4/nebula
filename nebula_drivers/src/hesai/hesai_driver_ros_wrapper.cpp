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
  drivers::HesaiCloudConfiguration cloud_configuration;
  drivers::HesaiSensorConfiguration sensor_configuration;

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration, cloud_configuration);
  if (Status::OK != wrapper_status_)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }

  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr =
    std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);

  std::shared_ptr<drivers::CloudConfigurationBase> cloud_cfg_ptr =
    std::make_shared<drivers::HesaiCloudConfiguration>(cloud_configuration);

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr),
    std::const_pointer_cast<drivers::CloudConfigurationBase>(cloud_cfg_ptr),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr));
  pandar_scan_sub_ = this->create_subscription<pandar_msgs::msg::PandarScan>(
    "pandar_packets", rclcpp::SensorDataQoS(),
    std::bind(&HesaiDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  pandar_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::SensorDataQoS());
}

Status HesaiDriverRosWrapper::ReceiveScanMsgCallback(
  const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  // take packets out of scan msg
  std::vector<pandar_msgs::msg::PandarPacket> pkt_msgs = scan_msg->packets;
  RCLCPP_INFO_STREAM(this->get_logger(), "Received Packet:" << pkt_msgs.size());
  // send to driver & receive pointcloud2 msg
  sensor_msgs::msg::PointCloud2 pointcloud = driver_ptr_->ParsePacketToPointcloud(pkt_msgs);

  // fix pointcloud2 header and stuff

  // publish
  pandar_points_pub_->publish(pointcloud);
  return Status::OK;
}

Status HesaiDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CloudConfigurationBase> cloud_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::HesaiCloudConfiguration>(cloud_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status HesaiDriverRosWrapper::GetStatus() { return wrapper_status_; }

Status HesaiDriverRosWrapper::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCloudConfiguration & cloud_configuration)
{
  sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(
    this->declare_parameter<std::string>("sensor_model", ""));

  sensor_configuration.return_mode =
    nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", ""));

  sensor_configuration.host_ip = this->declare_parameter<std::string>("host_ip", "127.0.0.1");
  sensor_configuration.sensor_ip =
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201");
  sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "pandar");
  sensor_configuration.data_port = this->declare_parameter<uint16_t>("data_port", 2368);
  sensor_configuration.gnss_port = this->declare_parameter<uint16_t>("gnss_port", 2369);
  sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0.);
  sensor_configuration.frequency_ms = this->declare_parameter<uint16_t>("frequency_ms", 100);

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (
    sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 365 ||
    sensor_configuration.frequency_ms == 0) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

}  // namespace ros
}  // namespace nebula