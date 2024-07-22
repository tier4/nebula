// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/aeva/aeva_ros_wrapper.hpp"

#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/pointcloud.hpp"
#include "nebula_ros/aeva/hw_monitor_wrapper.hpp"
#include "nebula_ros/common/nebula_packet_stream.hpp"
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/rclcpp_logger.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/util/parsing.hpp>
#include <nebula_decoders/nebula_decoders_aeva/aeva_aeries2_decoder.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_aeva/aeva_hw_interface.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_common/connections/stream_buffer.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
using drivers::AevaAeries2Decoder;
using drivers::aeva::Aeries2Config;
using drivers::connections::PointcloudParser;
using nlohmann::json;
using namespace std::chrono_literals;  // NOLINT
using AevaPointCloudUniquePtr = AevaAeries2Decoder::AevaPointCloudUniquePtr;

AevaRosWrapper::AevaRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("aeva_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  auto status = declareAndGetSensorConfigParams();

  if (status != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << status).str());
  }

  RCLCPP_INFO_STREAM(get_logger(), "SensorConfig: " << *sensor_cfg_ptr_);

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  auto launch_hw = declare_parameter<bool>("launch_hw", param_read_only());
  auto setup_sensor = declare_parameter<bool>("setup_sensor", param_read_only());
  auto hw_interface_logger = drivers::loggers::RclcppLogger(get_logger()).child("HwInterface");

  if (!launch_hw && setup_sensor) {
    setup_sensor = false;
    RCLCPP_WARN(get_logger(), "Ignoring setup_sensor:=true in offline mode.");
  }

  if (launch_hw) {
    // ////////////////////////////////////////
    // If HW is connected, also publish packets
    // ////////////////////////////////////////
    hw_interface_.emplace(hw_interface_logger, setup_sensor, sensor_cfg_ptr_);
    hw_monitor_.emplace(this, sensor_cfg_ptr_);

    packets_pub_ =
      create_publisher<nebula_msgs::msg::NebulaPackets>("nebula_packets", pointcloud_qos);

    drivers::connections::ObservableByteStream::callback_t raw_packet_cb = [&](const auto & bytes) {
      this->recordRawPacket(std::move(bytes));
    };

    hw_interface_->registerRawCloudPacketCallback(std::move(raw_packet_cb));

    hw_interface_->registerTelemetryCallback(
      [&](const auto & msg) { hw_monitor_->onTelemetryFragment(msg); });
    hw_interface_->registerHealthCallback([&](auto codes) { hw_monitor_->onHealthCodes(codes); });
  } else {
    // ////////////////////////////////////////
    // If HW is disconnected, subscribe to
    // packets topic
    // ////////////////////////////////////////
    auto packet_stream = std::make_shared<NebulaPacketStream>();
    auto packet_buffer =
      std::make_shared<drivers::connections::StreamBuffer>(packet_stream, 100, [&]() {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Packet stream buffer overflowed, packet loss occurred.");
      });

    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS(),
      [=](std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets) {
        packet_stream->onNebulaPackets(std::move(packets));
      });

    auto pointcloud_parser = std::make_shared<PointcloudParser>(packet_buffer);
    hw_interface_.emplace(
      hw_interface_logger, setup_sensor, sensor_cfg_ptr_, pointcloud_parser, nullptr, nullptr,
      nullptr);

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  RCLCPP_INFO(get_logger(), "Starting stream");

  PointcloudParser::callback_t pointcloud_message_cb = [this](const auto & message) {
    decoder_.processPointcloudMessage(message);
  };

  hw_interface_->registerCloudPacketCallback(std::move(pointcloud_message_cb));

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("nebula_points", pointcloud_qos);

  cloud_watchdog_ = std::make_shared<WatchdogTimer>(*this, 110'000us, [&](bool ok) {
    if (ok) return;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Missed pointcloud output deadline");
  });

  AevaAeries2Decoder::callback_t pointcloud_cb =
    [&](AevaPointCloudUniquePtr cloud_ptr, auto timestamp) {
      auto now = this->now();
      cloud_watchdog_->update();

      if (
        cloud_pub_->get_subscription_count() > 0 ||
        cloud_pub_->get_intra_process_subscription_count() > 0) {
        auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_ptr, *ros_pc_msg_ptr);
        ros_pc_msg_ptr->header.frame_id = sensor_cfg_ptr_->frame_id;
        ros_pc_msg_ptr->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp));
        cloud_pub_->publish(std::move(ros_pc_msg_ptr));
      }

      std::lock_guard lock(mtx_current_scan_msg_);
      if (current_scan_msg_ && packets_pub_) {
        packets_pub_->publish(std::move(current_scan_msg_));
        current_scan_msg_ = {};
      }
    };

  decoder_.registerPointCloudCallback(std::move(pointcloud_cb));

  parameter_event_cb_ =
    add_on_set_parameters_callback([this](const auto & p) { return onParameterChange(p); });
}

Status AevaRosWrapper::declareAndGetSensorConfigParams()
{
  Aeries2Config config;

  std::string raw_sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::SensorModelFromString(raw_sensor_model);
  config.sensor_ip = declare_parameter<std::string>("sensor_ip", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_only());

  declareJsonParam<float>("scanner.dithering_enable_ego_speed", config.tree);
  declareJsonParam<std::string>("scanner.dithering_pattern_option", config.tree);
  declareJsonParam<float>("scanner.ele_offset_rad", config.tree);
  declareJsonParam<bool>("scanner.elevation_auto_adjustment", config.tree);
  declareJsonParam<bool>("scanner.enable_frame_dithering", config.tree);
  declareJsonParam<bool>("scanner.enable_frame_sync", config.tree);
  declareJsonParam<bool>("scanner.flip_pattern_vertically", config.tree);
  declareJsonParam<uint16_t>("scanner.frame_sync_offset_in_ms", config.tree);
  declareJsonParam<std::string>("scanner.frame_sync_type", config.tree);
  declareJsonParam<bool>("scanner.frame_synchronization_on_rising_edge", config.tree);
  declareJsonParam<float>("scanner.hfov_adjustment_deg", config.tree);
  declareJsonParam<float>("scanner.hfov_rotation_deg", config.tree);
  declareJsonParam<bool>("scanner.highlight_ROI", config.tree);
  declareJsonParam<std::string>("scanner.horizontal_fov_degrees", config.tree);
  declareJsonParam<float>("scanner.roi_az_offset_rad", config.tree);
  declareJsonParam<std::string>("scanner.vertical_pattern", config.tree);
  declareJsonParam<std::string>("system_config.range_modes", config.tree);
  declareJsonParam<std::string>("system_config.sensitivity_mode", config.tree);
  declareJsonParam<std::string>("system_config.thermal_throttling_setting", config.tree);
  declareJsonParam<bool>("spc_converter.discard_points_in_ambiguity_region", config.tree);
  declareJsonParam<bool>("spc_converter.display_all_points", config.tree);
  declareJsonParam<bool>("spc_converter.enable_min_range_filter", config.tree);
  declareJsonParam<std::string>("dsp_control.second_peak_type", config.tree);
  declareJsonParam<bool>("dsp_control.use_foveated_velocity_bias", config.tree);
  declareJsonParam<std::string>("dsp_control.velocity_bias_pattern_options", config.tree);

  auto new_cfg_ptr = std::make_shared<const Aeries2Config>(config);
  return validateAndSetConfig(new_cfg_ptr);
}

Status AevaRosWrapper::validateAndSetConfig(std::shared_ptr<const Aeries2Config> & new_config)
{
  if (!new_config) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (new_config->sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  if (new_config->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_) {
    try {
      hw_interface_->onConfigChange(new_config);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Sending configuration to sensor failed: " << e.what());
      return Status::SENSOR_CONFIG_ERROR;
    }
  }

  auto return_mode_opt = new_config->getReturnMode();

  if (return_mode_opt && *return_mode_opt == drivers::ReturnMode::UNKNOWN) {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid return mode");
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (return_mode_opt) {
    decoder_.onParameterChange(*return_mode_opt);
  }

  sensor_cfg_ptr_ = new_config;
  return Status::OK;
}

rcl_interfaces::msg::SetParametersResult AevaRosWrapper::onParameterChange(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;
  Aeries2Config config = *sensor_cfg_ptr_;

  bool got_any =
    getJsonParam<float>(p, "scanner.dithering_enable_ego_speed", config.tree) |
    getJsonParam<std::string>(p, "scanner.dithering_pattern_option", config.tree) |
    getJsonParam<float>(p, "scanner.ele_offset_rad", config.tree) |
    getJsonParam<bool>(p, "scanner.elevation_auto_adjustment", config.tree) |
    getJsonParam<bool>(p, "scanner.enable_frame_dithering", config.tree) |
    getJsonParam<bool>(p, "scanner.enable_frame_sync", config.tree) |
    getJsonParam<bool>(p, "scanner.flip_pattern_vertically", config.tree) |
    getJsonParam<uint16_t>(p, "scanner.frame_sync_offset_in_ms", config.tree) |
    getJsonParam<std::string>(p, "scanner.frame_sync_type", config.tree) |
    getJsonParam<bool>(p, "scanner.frame_synchronization_on_rising_edge", config.tree) |
    getJsonParam<float>(p, "scanner.hfov_adjustment_deg", config.tree) |
    getJsonParam<float>(p, "scanner.hfov_rotation_deg", config.tree) |
    getJsonParam<bool>(p, "scanner.highlight_ROI", config.tree) |
    getJsonParam<std::string>(p, "scanner.horizontal_fov_degrees", config.tree) |
    getJsonParam<float>(p, "scanner.roi_az_offset_rad", config.tree) |
    getJsonParam<std::string>(p, "scanner.vertical_pattern", config.tree) |
    getJsonParam<std::string>(p, "system_config.range_modes", config.tree) |
    getJsonParam<std::string>(p, "system_config.sensitivity_mode", config.tree) |
    getJsonParam<std::string>(p, "system_config.thermal_throttling_setting", config.tree) |
    getJsonParam<bool>(p, "spc_converter.discard_points_in_ambiguity_region", config.tree) |
    getJsonParam<bool>(p, "spc_converter.display_all_points", config.tree) |
    getJsonParam<bool>(p, "spc_converter.enable_min_range_filter", config.tree) |
    getJsonParam<std::string>(p, "dsp_control.second_peak_type", config.tree) |
    getJsonParam<bool>(p, "dsp_control.use_foveated_velocity_bias", config.tree) |
    getJsonParam<std::string>(p, "dsp_control.velocity_bias_pattern_options", config.tree);

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  auto new_cfg_ptr = std::make_shared<const Aeries2Config>(config);
  auto status = validateAndSetConfig(new_cfg_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = (std::stringstream() << "Invalid configuration: " << status).str();
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void AevaRosWrapper::recordRawPacket(const std::vector<uint8_t> & vector)
{
  std::lock_guard lock(mtx_current_scan_msg_);

  if (
    !packets_pub_ || (packets_pub_->get_subscription_count() == 0 &&
                      packets_pub_->get_intra_process_subscription_count() == 0)) {
    return;
  }

  auto packet_stamp = now();

  if (!current_scan_msg_) {
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    auto & header = current_scan_msg_->header;
    header.frame_id = sensor_cfg_ptr_->frame_id;
    header.stamp = packet_stamp;
  }

  auto & packet = current_scan_msg_->packets.emplace_back();
  packet.stamp = packet_stamp;
  packet.data = vector;
}

RCLCPP_COMPONENTS_REGISTER_NODE(AevaRosWrapper)
}  // namespace nebula::ros
