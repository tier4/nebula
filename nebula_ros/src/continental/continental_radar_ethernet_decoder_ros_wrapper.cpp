// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebula_ros/continental/continental_radar_ethernet_decoder_ros_wrapper.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace nebula
{
namespace ros
{
ContinentalRadarEthernetDriverRosWrapper::ContinentalRadarEthernetDriverRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("continental_driver_ros_wrapper", options), hw_interface_()
{
  drivers::ContinentalRadarEthernetSensorConfiguration sensor_configuration;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  wrapper_status_ = GetParameters(sensor_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  sensor_cfg_ptr_ =
    std::make_shared<drivers::ContinentalRadarEthernetSensorConfiguration>(sensor_configuration);

  wrapper_status_ =
    InitializeDriver(std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
  packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
    "nebula_packets", rclcpp::SensorDataQoS(),
    std::bind(
      &ContinentalRadarEthernetDriverRosWrapper::ReceivePacketsMsgCallback, this,
      std::placeholders::_1));

  detection_list_pub_ =
    this->create_publisher<continental_msgs::msg::ContinentalArs548DetectionList>(
      "detection_array", rclcpp::SensorDataQoS());
  object_list_pub_ = this->create_publisher<continental_msgs::msg::ContinentalArs548ObjectList>(
    "object_array", rclcpp::SensorDataQoS());

  detection_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "detection_points", rclcpp::SensorDataQoS());
  object_pointcloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("object_points", rclcpp::SensorDataQoS());
}

void ContinentalRadarEthernetDriverRosWrapper::ReceivePacketsMsgCallback(
  const nebula_msgs::msg::NebulaPackets::SharedPtr scan_msg)
{
  decoder_ptr_->ProcessPackets(*scan_msg);
}

void ContinentalRadarEthernetDriverRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status ContinentalRadarEthernetDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration)
{
  decoder_ptr_ = std::make_shared<drivers::continental_ars548::ContinentalARS548Decoder>(
    std::static_pointer_cast<drivers::ContinentalRadarEthernetSensorConfiguration>(
      sensor_configuration));

  decoder_ptr_->RegisterDetectionListCallback(std::bind(
    &ContinentalRadarEthernetDriverRosWrapper::detectionListCallback, this, std::placeholders::_1));
  decoder_ptr_->RegisterObjectListCallback(std::bind(
    &ContinentalRadarEthernetDriverRosWrapper::objectListCallback, this, std::placeholders::_1));

  return Status::OK;
}

Status ContinentalRadarEthernetDriverRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status ContinentalRadarEthernetDriverRosWrapper::GetParameters(
  drivers::ContinentalRadarEthernetSensorConfiguration & sensor_configuration)
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
    this->declare_parameter<std::string>("frame_id", "continental_ars548", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::ContinentalRadarEthernetSensorConfiguration>(sensor_configuration);

  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void ContinentalRadarEthernetDriverRosWrapper::detectionListCallback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)
{
  if (
    detection_pointcloud_pub_->get_subscription_count() > 0 ||
    detection_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto detection_pointcloud_ptr =
      nebula::drivers::continental_ars548::convertToPointcloud(*msg);
    auto detection_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*detection_pointcloud_ptr, *detection_pointcloud_msg_ptr);

    detection_pointcloud_msg_ptr->header = msg->header;
    detection_pointcloud_pub_->publish(std::move(detection_pointcloud_msg_ptr));
  }
  if (
    detection_list_pub_->get_subscription_count() > 0 ||
    detection_list_pub_->get_intra_process_subscription_count() > 0) {
    detection_list_pub_->publish(std::move(msg));
  }
}

void ContinentalRadarEthernetDriverRosWrapper::objectListCallback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)
{
  if (
    object_pointcloud_pub_->get_subscription_count() > 0 ||
    object_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto object_pointcloud_ptr =
      nebula::drivers::continental_ars548::convertToPointcloud(*msg);
    auto object_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*object_pointcloud_ptr, *object_pointcloud_msg_ptr);

    object_pointcloud_msg_ptr->header = msg->header;
    object_pointcloud_pub_->publish(std::move(object_pointcloud_msg_ptr));
  }
  if (
    object_list_pub_->get_subscription_count() > 0 ||
    object_list_pub_->get_intra_process_subscription_count() > 0) {
    object_list_pub_->publish(std::move(msg));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalRadarEthernetDriverRosWrapper)
}  // namespace ros
}  // namespace nebula
