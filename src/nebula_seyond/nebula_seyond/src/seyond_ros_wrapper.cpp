// Copyright 2026 TIER IV, Inc.
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

#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <nebula_seyond/hw_interface_wrapper.hpp>
#include <nebula_seyond/hw_monitor_wrapper.hpp>
#include <nebula_seyond/seyond_ros_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/format.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::ros
{

namespace
{

builtin_interfaces::msg::Time clamp_ros_time(uint64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp{};
  const uint64_t max_ros_time_ns =
    static_cast<uint64_t>(std::numeric_limits<int32_t>::max()) * 1000000000ULL + 999999999ULL;
  const uint64_t clamped_timestamp_ns = std::min(timestamp_ns, max_ros_time_ns);
  stamp.sec = static_cast<int32_t>(clamped_timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(clamped_timestamp_ns % 1000000000ULL);
  return stamp;
}

nebula::drivers::SeyondCalibrationData load_calibration_file_or_throw(
  const std::string & calibration_file)
{
  auto calibration_or_error =
    nebula::drivers::SeyondCalibrationData::load_from_file(calibration_file);
  if (!calibration_or_error.has_value()) {
    throw std::runtime_error(
      "Failed to load Seyond calibration file '" + calibration_file +
      "': " + calibration_or_error.error().message);
  }
  return calibration_or_error.value();
}

}  // namespace

SeyondRosWrapper::SeyondRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("seyond_ros_wrapper", options), diagnostic_updater_(this)
{
  declare_parameters();
  get_parameters();

  diagnostic_updater_.setHardwareID(config_.connection.sensor_ip);

  cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("seyond_points", rclcpp::SensorDataQoS());

  nebula::drivers::SeyondCalibrationData calibration{};

  if (launch_hw_) {
    auto config_ptr = std::make_shared<const nebula::drivers::SeyondSensorConfiguration>(config_);
    try {
      hw_interface_wrapper_ = std::make_unique<SeyondHwInterfaceWrapper>(this, config_ptr);
      hw_interface_ = hw_interface_wrapper_->hw_interface();
      hw_interface_->register_scan_callback(
        std::bind(
          &SeyondRosWrapper::receive_packet_callback, this, std::placeholders::_1,
          std::placeholders::_2));
      hw_monitor_wrapper_ = std::make_unique<SeyondHwMonitorWrapper>(
        this, diagnostic_updater_, hw_interface_, config_ptr);
      calibration = *hw_interface_wrapper_->calibration();
      if (!calibration_file_.empty()) {
        calibration = load_calibration_file_or_throw(calibration_file_);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize Seyond hardware wrappers: %s", ex.what());
    }
  } else if (!calibration_file_.empty()) {
    calibration = load_calibration_file_or_throw(calibration_file_);
  }

  decoder_ = std::make_unique<nebula::drivers::SeyondDecoder>(
    config_,
    std::bind(&SeyondRosWrapper::publish_cloud, this, std::placeholders::_1, std::placeholders::_2),
    calibration);

  if (launch_hw_ && hw_interface_) {
    auto status = hw_interface_->sensor_interface_start();
    if (status != nebula::Status::OK) {
      RCLCPP_ERROR(
        get_logger(), "Failed to start Seyond hardware interface: %s",
        (boost::format("%1%") % status).str().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Seyond hardware interface started successfully");
    }
  }
}

void SeyondRosWrapper::receive_packet_callback(
  std::vector<uint8_t> & packet,
  const nebula::drivers::connections::UdpSocket::RxMetadata & metadata)
{
  (void)metadata;
  decoder_->unpack(packet);
}

void SeyondRosWrapper::publish_cloud(
  nebula::drivers::NebulaPointCloudPtr cloud, uint64_t base_timestamp_ns)
{
  if (!cloud || cloud->empty()) {
    return;
  }

  if (
    cloud_pub_->get_subscription_count() == 0 &&
    cloud_pub_->get_intra_process_subscription_count() == 0) {
    return;
  }

  auto ros_msg = nebula::ros::to_ros_msg(cloud);

  ros_msg.header.frame_id = config_.frame_id;

  if (config_.use_sensor_time) {
    uint64_t ts_ns = base_timestamp_ns + cloud->front().time_stamp;
    ros_msg.header.stamp = clamp_ros_time(ts_ns);
  } else {
    ros_msg.header.stamp = now();
  }

  cloud_pub_->publish(ros_msg);
}

void SeyondRosWrapper::declare_parameters()
{
  declare_parameter<bool>("launch_hw", true);
  declare_parameter<std::string>("sensor_model", "FalconK");
  declare_parameter<std::string>("host_ip", "172.168.1.100");
  declare_parameter<std::string>("sensor_ip", "172.168.1.10");
  declare_parameter<std::string>("netmask", "255.255.255.0");
  declare_parameter<std::string>("gateway", "0.0.0.0");
  declare_parameter<int>("udp_port", 8010);
  declare_parameter<int>("udp_message_port", 8010);
  declare_parameter<int>("udp_status_port", 8010);
  declare_parameter<bool>("use_sensor_time", false);
  declare_parameter<std::string>("frame_id", "seyond");
  declare_parameter<bool>("setup_sensor", true);
  declare_parameter<std::string>("return_mode", "Single");
  declare_parameter<std::string>("reflectance_mode", "Reflectivity");
  declare_parameter<std::string>("time_sync", "Host");
  declare_parameter<double>("frame_rate", 0.0);
  declare_parameter<double>("horizontal_roi", 10000.0);
  declare_parameter<double>("vertical_roi", 10000.0);
  declare_parameter<std::string>("calibration_file", "");
}

void SeyondRosWrapper::get_parameters()
{
  launch_hw_ = get_parameter("launch_hw").as_bool();

  std::string model_str = get_parameter("sensor_model").as_string();
  config_.sensor_model = nebula::drivers::seyond_sensor_model_from_string(model_str);
  if (config_.sensor_model == nebula::drivers::SeyondSensorModel::UNKNOWN) {
    RCLCPP_WARN(get_logger(), "Unknown Seyond sensor model: '%s'", model_str.c_str());
  }

  config_.connection.host_ip = get_parameter("host_ip").as_string();
  config_.connection.sensor_ip = get_parameter("sensor_ip").as_string();
  config_.connection.netmask = get_parameter("netmask").as_string();
  config_.connection.gateway = get_parameter("gateway").as_string();
  config_.connection.udp_port = static_cast<uint16_t>(get_parameter("udp_port").as_int());
  config_.connection.udp_message_port =
    static_cast<uint16_t>(get_parameter("udp_message_port").as_int());
  config_.connection.udp_status_port =
    static_cast<uint16_t>(get_parameter("udp_status_port").as_int());
  config_.use_sensor_time = get_parameter("use_sensor_time").as_bool();
  config_.frame_id = get_parameter("frame_id").as_string();
  config_.setup_sensor = get_parameter("setup_sensor").as_bool();
  config_.return_mode =
    nebula::drivers::return_mode_from_string_seyond(get_parameter("return_mode").as_string());
  config_.reflectance_mode = nebula::drivers::reflectance_mode_from_string_seyond(
    get_parameter("reflectance_mode").as_string());
  config_.sync_mode =
    nebula::drivers::sync_mode_from_string_seyond(get_parameter("time_sync").as_string());
  config_.frame_rate = get_parameter("frame_rate").as_double();
  config_.horizontal_roi = get_parameter("horizontal_roi").as_double();
  config_.vertical_roi = get_parameter("vertical_roi").as_double();
  calibration_file_ = get_parameter("calibration_file").as_string();
}

}  // namespace nebula::ros

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SeyondRosWrapper)
