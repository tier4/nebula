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

  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr =
    std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);
  std::shared_ptr<drivers::CloudConfigurationBase> cloud_cfg_ptr =
    std::make_shared<drivers::HesaiCloudConfiguration>(cloud_configuration);
  InitializeDriver(
    std::const_pointer_cast<drivers::CloudConfigurationBase>(cloud_cfg_ptr),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr));
  pandar_scan_sub_ = this->create_subscription<pandar_msgs::msg::PandarScan>(
    "pandar_packets", rclcpp::SensorDataQoS(),
    std::bind(&HesaiDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  pandar_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::SensorDataQoS());
}

Status HesaiDriverRosWrapper::StreamStart() { return Status::OK; }

Status HesaiDriverRosWrapper::StreamStop() { return Status::OK; }

Status HesaiDriverRosWrapper::Shutdown() { return Status::OK; }

Status HesaiDriverRosWrapper::ReceiveScanMsgCallback(
  const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  // take packets out of scan msg
  std::vector<pandar_msgs::msg::PandarPacket> pkt_msgs = scan_msg->packets;

  // send to driver & receive pointcloud2 msg
  sensor_msgs::msg::PointCloud2 pointcloud = driver_.ParsePacketToPointcloud(pkt_msgs);

  // fix pointcloud2 header and stuff

  // publish
  pandar_points_pub_->publish(pointcloud);
}

Status HesaiDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::CloudConfigurationBase> cloud_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  drivers::HesaiDriver driver_{
    std::static_pointer_cast<drivers::HesaiCloudConfiguration>(cloud_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration)};
  // proper decoder depends on lidar model and return mode in cloud_configuration
  return Status::OK;
}

}  // namespace ros
}  // namespace nebula