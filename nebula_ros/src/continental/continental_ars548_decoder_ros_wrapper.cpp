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

#include "nebula_ros/continental/continental_ars548_decoder_ros_wrapper.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace nebula
{
namespace ros
{
ContinentalARS548DriverRosWrapper::ContinentalARS548DriverRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("continental_ars548_driver_ros_wrapper", options), hw_interface_()
{
  drivers::ContinentalARS548SensorConfiguration sensor_configuration;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  wrapper_status_ = GetParameters(sensor_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  sensor_cfg_ptr_ =
    std::make_shared<drivers::ContinentalARS548SensorConfiguration>(sensor_configuration);

  wrapper_status_ =
    InitializeDriver(std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
  packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
    "nebula_packets", rclcpp::SensorDataQoS(),
    std::bind(
      &ContinentalARS548DriverRosWrapper::ReceivePacketsMsgCallback, this, std::placeholders::_1));

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

void ContinentalARS548DriverRosWrapper::ReceivePacketsMsgCallback(
  const nebula_msgs::msg::NebulaPackets::SharedPtr scan_msg)
{
  decoder_ptr_->ProcessPackets(*scan_msg);
}

Status ContinentalARS548DriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration)
{
  decoder_ptr_ = std::make_shared<drivers::continental_ars548::ContinentalARS548Decoder>(
    std::static_pointer_cast<drivers::ContinentalARS548SensorConfiguration>(sensor_configuration));

  decoder_ptr_->RegisterDetectionListCallback(std::bind(
    &ContinentalARS548DriverRosWrapper::DetectionListCallback, this, std::placeholders::_1));
  decoder_ptr_->RegisterObjectListCallback(
    std::bind(&ContinentalARS548DriverRosWrapper::ObjectListCallback, this, std::placeholders::_1));

  return Status::OK;
}

Status ContinentalARS548DriverRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status ContinentalARS548DriverRosWrapper::GetParameters(
  drivers::ContinentalARS548SensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_ip", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::ContinentalARS548SensorConfiguration>(sensor_configuration);

  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void ContinentalARS548DriverRosWrapper::DetectionListCallback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)
{
  if (
    detection_pointcloud_pub_->get_subscription_count() > 0 ||
    detection_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto detection_pointcloud_ptr = ConvertToPointcloud(*msg);
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

void ContinentalARS548DriverRosWrapper::ObjectListCallback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)
{
  if (
    object_pointcloud_pub_->get_subscription_count() > 0 ||
    object_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto object_pointcloud_ptr = ConvertToPointcloud(*msg);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr ContinentalARS548DriverRosWrapper::ConvertToPointcloud(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  output_pointcloud->reserve(msg.detections.size());

  pcl::PointXYZ point{};
  for (const auto & detection : msg.detections) {
    point.x =
      std::cos(detection.elevation_angle) * std::cos(detection.azimuth_angle) * detection.range;
    point.y =
      std::cos(detection.elevation_angle) * std::sin(detection.azimuth_angle) * detection.range;
    point.z = std::sin(detection.elevation_angle) * detection.range;

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ContinentalARS548DriverRosWrapper::ConvertToPointcloud(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  output_pointcloud->reserve(msg.objects.size());

  pcl::PointXYZ point{};
  for (const auto & detection : msg.objects) {
    point.x = static_cast<float>(detection.position.x);
    point.y = static_cast<float>(detection.position.y);
    point.z = static_cast<float>(detection.position.z);

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

radar_msgs::msg::RadarScan ConvertToRadarScan(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  radar_msgs::msg::RadarScan output_msg;
  output_msg.header = msg.header;
  output_msg.returns.reserve(msg.detections.size());

  radar_msgs::msg::RadarReturn return_msg;
  for (const auto & detection : msg.detections) {
    if (
      detection.invalid_azimuth || detection.invalid_distance || detection.invalid_elevation ||
      detection.invalid_range_rate) {
      continue;
    }

    return_msg.range = detection.range;
    return_msg.azimuth = detection.azimuth_angle;
    return_msg.elevation = detection.elevation_angle;
    return_msg.doppler_velocity = detection.range_rate;
    return_msg.amplitude = detection.rcs;
    output_msg.returns.emplace_back(return_msg);
  }

  return output_msg;
}

radar_msgs::msg::RadarTracks ConvertToRadarTracks(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  radar_msgs::msg::RadarTracks output_msg;
  output_msg.tracks.reserve(msg.objects.size());
  output_msg.header = msg.header;

  constexpr int16_t UNKNOWN_ID = 32000;
  constexpr int16_t CAR_ID = 32001;
  constexpr int16_t TRUCK_ID = 32002;
  constexpr int16_t MOTORCYCLE_ID = 32005;
  constexpr int16_t BICYCLE_ID = 32006;
  constexpr int16_t PEDESTRIAN_ID = 32007;

  radar_msgs::msg::RadarTrack track_msg;
  for (const auto & detection : msg.objects) {
    track_msg.uuid.uuid[0] = static_cast<uint8_t>(detection.object_id & 0xff);
    track_msg.uuid.uuid[1] = static_cast<uint8_t>((detection.object_id >> 8) & 0xff);
    track_msg.uuid.uuid[2] = static_cast<uint8_t>((detection.object_id >> 16) & 0xff);
    track_msg.uuid.uuid[3] = static_cast<uint8_t>((detection.object_id >> 24) & 0xff);
    track_msg.position = detection.position;
    track_msg.velocity = detection.absolute_velocity;
    track_msg.acceleration = detection.absolute_acceleration;
    track_msg.size.x = detection.shape_length_edge_mean;
    track_msg.size.y = detection.shape_width_edge_mean;
    track_msg.size.z = 1.f;

    uint8_t max_score = detection.classification_unknown;
    track_msg.classification = UNKNOWN_ID;

    if (detection.classification_car > max_score) {
      max_score = detection.classification_car;
      track_msg.classification = CAR_ID;
    }
    if (detection.classification_truck > max_score) {
      max_score = detection.classification_truck;
      track_msg.classification = TRUCK_ID;
    }
    if (detection.classification_motorcycle > max_score) {
      max_score = detection.classification_motorcycle;
      track_msg.classification = MOTORCYCLE_ID;
    }
    if (detection.classification_bicycle > max_score) {
      max_score = detection.classification_bicycle;
      track_msg.classification = BICYCLE_ID;
    }
    if (detection.classification_pedestrian > max_score) {
      max_score = detection.classification_pedestrian;
      track_msg.classification = PEDESTRIAN_ID;
    }

    track_msg.position_covariance[0] = static_cast<float>(detection.position_std.x);
    track_msg.position_covariance[1] = detection.position_covariance_xy;
    track_msg.position_covariance[2] = 0.f;
    track_msg.position_covariance[3] = static_cast<float>(detection.position_std.y);
    track_msg.position_covariance[4] = 0.f;
    track_msg.position_covariance[5] = static_cast<float>(detection.position_std.z);

    track_msg.velocity_covariance[0] = static_cast<float>(detection.absolute_velocity_std.x);
    track_msg.velocity_covariance[1] = detection.absolute_velocity_covariance_xy;
    track_msg.velocity_covariance[2] = 0.f;
    track_msg.velocity_covariance[3] = static_cast<float>(detection.absolute_velocity_std.y);
    track_msg.velocity_covariance[4] = 0.f;
    track_msg.velocity_covariance[5] = static_cast<float>(detection.absolute_velocity_std.z);

    track_msg.acceleration_covariance[0] =
      static_cast<float>(detection.absolute_acceleration_std.x);
    track_msg.acceleration_covariance[1] = detection.absolute_acceleration_covariance_xy;
    track_msg.acceleration_covariance[2] = 0.f;
    track_msg.acceleration_covariance[3] =
      static_cast<float>(detection.absolute_acceleration_std.y);
    track_msg.acceleration_covariance[4] = 0.f;
    track_msg.acceleration_covariance[5] =
      static_cast<float>(detection.absolute_acceleration_std.z);

    output_msg.tracks.emplace_back(track_msg);
  }

  return output_msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalARS548DriverRosWrapper)
}  // namespace ros
}  // namespace nebula
