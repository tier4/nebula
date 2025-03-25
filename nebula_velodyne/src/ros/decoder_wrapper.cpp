// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/decoder_wrapper.hpp"

#include <nebula_common/util/string_conversions.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

VelodyneDecoderWrapper::VelodyneDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("VelodyneDecoder")),
  hw_interface_(hw_interface),
  sensor_cfg_(config)
{
  if (!config) {
    throw std::runtime_error(
      "VelodyneDecoderWrapper cannot be instantiated without a valid config!");
  }

  calibration_file_path_ =
    parent_node->declare_parameter<std::string>("calibration_file", param_read_write());
  auto calibration_result = get_calibration_data(calibration_file_path_);

  if (!calibration_result.has_value()) {
    throw std::runtime_error(
      "No valid calibration found: " + util::to_string(calibration_result.error()));
  }

  calibration_cfg_ptr_ = calibration_result.value();
  RCLCPP_INFO_STREAM(
    logger_, "Using calibration data from " << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(config, calibration_cfg_ptr_);
  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(status_));
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<velodyne_msgs::msg::VelodyneScan>();
    packets_pub_ = parent_node->create_publisher<velodyne_msgs::msg::VelodyneScan>(
      "velodyne_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", pointcloud_qos);
  aw_points_base_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void VelodyneDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::VelodyneDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void VelodyneDecoderWrapper::on_calibration_change(
  const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> & new_calibration)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::VelodyneDriver>(sensor_cfg_, new_calibration);
  driver_ptr_ = new_driver;
  calibration_cfg_ptr_ = new_calibration;
  calibration_file_path_ = calibration_cfg_ptr_->calibration_file;
}

rcl_interfaces::msg::SetParametersResult VelodyneDecoderWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  std::string calibration_path = "";

  bool got_any = get_param(p, "calibration_file", calibration_path);
  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (!std::filesystem::exists(calibration_path)) {
    auto result = SetParametersResult();
    result.successful = false;
    result.reason =
      "The given calibration path does not exist, ignoring: '" + calibration_path + "'";
    return result;
  }

  auto get_calibration_result = get_calibration_data(calibration_path);
  if (!get_calibration_result.has_value()) {
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Could not change calibration file to '" + calibration_path +
                    "': " + util::to_string(get_calibration_result.error());
    return result;
  }

  on_calibration_change(get_calibration_result.value());
  RCLCPP_INFO_STREAM(
    logger_, "Changed calibration to '" << calibration_cfg_ptr_->calibration_file << "'");
  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

VelodyneDecoderWrapper::get_calibration_result_t VelodyneDecoderWrapper::get_calibration_data(
  const std::string & calibration_file_path)
{
  auto calib = std::make_shared<drivers::VelodyneCalibrationConfiguration>();

  // If downloaded data did not exist or could not be loaded, fall back to a generic file.
  // If that file does not exist either, return an error code
  if (!std::filesystem::exists(calibration_file_path)) {
    RCLCPP_ERROR(logger_, "No calibration data found.");
    return nebula::Status(Status::INVALID_CALIBRATION_FILE);
  }

  // Try to load the existing fallback calibration file. Return an error if this fails
  auto status = calib->load_from_file(calibration_file_path);
  if (status != Status::OK) {
    RCLCPP_ERROR_STREAM(
      logger_, "Could not load calibration file at '" << calibration_file_path << "'");
    return status;
  }

  // Return the fallback calibration file
  calib->calibration_file = calibration_file_path;
  return calib;
}

void VelodyneDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (
    hw_interface_ && (packets_pub_->get_subscription_count() > 0 ||
                      packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    velodyne_msgs::msg::VelodynePacket velodyne_packet_msg{};
    velodyne_packet_msg.stamp = packet_msg->stamp;
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), velodyne_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(std::move(velodyne_packet_msg));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;
  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts =
      driver_ptr_->parse_cloud_packet(packet_msg->data, rclcpp::Time(packet_msg->stamp).seconds());
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr) {
    return;
  }

  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<velodyne_msgs::msg::VelodyneScan>();
  }

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud = nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }
}

void VelodyneDecoderWrapper::publish_cloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

nebula::Status VelodyneDecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}
}  // namespace nebula::ros
