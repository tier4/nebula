// Copyright 2024 TIER IV, Inc.

#include "hesai_ros_decoder_test.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_ros/common/rclcpp_logger.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <pandar_msgs/msg/pandar_scan.hpp>

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <vector>

namespace nebula::ros
{
HesaiRosDecoderTest::HesaiRosDecoderTest(
  const rclcpp::NodeOptions & options, const std::string & node_name,
  const HesaiRosDecoderTestParams & params)
: rclcpp::Node(node_name, options), params_(params)
{
  drivers::HesaiCalibrationConfiguration calibration_configuration;
  drivers::HesaiSensorConfiguration sensor_configuration;
  drivers::HesaiCorrection correction_configuration;

  wrapper_status_ =
    GetParameters(sensor_configuration, calibration_configuration, correction_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error: " << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    correction_cfg_ptr_ = std::make_shared<drivers::HesaiCorrection>(correction_configuration);
    wrapper_status_ = InitializeDriver(
      std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
      correction_cfg_ptr_);
  } else {
    wrapper_status_ = InitializeDriver(
      std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
      calibration_cfg_ptr_);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
}

Status HesaiRosDecoderTest::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calibration_configuration)
{
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    calibration_configuration, std::make_shared<drivers::loggers::RclcppLogger>(get_logger()));
  return driver_ptr_->get_status();
}

Status HesaiRosDecoderTest::get_status()
{
  return wrapper_status_;
}

Status HesaiRosDecoderTest::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCorrection & correction_configuration)
{
  std::filesystem::path calibration_dir = _SRC_CALIBRATION_DIR_PATH;
  calibration_dir /= "hesai";
  std::filesystem::path bag_root_dir = _SRC_RESOURCES_DIR_PATH;
  bag_root_dir /= "hesai";

  sensor_configuration.sensor_model =
    nebula::drivers::sensor_model_from_string(params_.sensor_model);
  sensor_configuration.return_mode = nebula::drivers::return_mode_from_string_hesai(
    params_.return_mode, sensor_configuration.sensor_model);
  sensor_configuration.frame_id = params_.frame_id;
  sensor_configuration.sync_angle = params_.sync_angle;
  sensor_configuration.cut_angle = params_.cut_angle;
  sensor_configuration.cloud_min_angle = params_.fov_start;
  sensor_configuration.cloud_max_angle = params_.fov_end;
  sensor_configuration.min_range = params_.min_range;
  sensor_configuration.max_range = params_.max_range;

  sensor_configuration.calibration_path = (calibration_dir / params_.calibration_file).string();
  params_.bag_path = (bag_root_dir / params_.bag_path).string();
  RCLCPP_DEBUG_STREAM(get_logger(), "Bag path relative to test root: " << params_.bag_path);
  params_.storage_id = params_.storage_id;
  params_.format = params_.format;
  params_.target_topic = params_.target_topic;
  sensor_configuration.dual_return_distance_threshold = params_.dual_return_distance_threshold;

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (sensor_configuration.calibration_path.empty()) {
    return Status::INVALID_CALIBRATION_FILE;
  }

  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    auto cal_status =
      correction_configuration.load_from_file(sensor_configuration.calibration_path);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Invalid correction file: '" << sensor_configuration.calibration_path << "'");
      return cal_status;
    }
  } else {
    auto cal_status =
      calibration_configuration.load_from_file(sensor_configuration.calibration_path);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Invalid calibration file: '" << sensor_configuration.calibration_path << "'");
      return cal_status;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Sensor configuration: " << sensor_configuration);
  return Status::OK;
}

void HesaiRosDecoderTest::read_bag(
  std::function<void(uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr)> scan_callback)
{
  rosbag2_storage::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  RCLCPP_DEBUG_STREAM(get_logger(), params_.bag_path);
  RCLCPP_DEBUG_STREAM(get_logger(), params_.storage_id);
  RCLCPP_DEBUG_STREAM(get_logger(), params_.format);
  RCLCPP_DEBUG_STREAM(get_logger(), params_.target_topic);

  auto target_topic_name = params_.target_topic;
  if (target_topic_name.substr(0, 1) == "/") {
    target_topic_name = target_topic_name.substr(1);
  }
  target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");

  rcpputils::fs::path bag_dir(params_.bag_path);

  storage_options.uri = params_.bag_path;
  storage_options.storage_id = params_.storage_id;
  converter_options.output_serialization_format = params_.format;
  rclcpp::Serialization<pandar_msgs::msg::PandarScan> serialization;

  rosbag2_cpp::Reader bag_reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  bag_reader.open(storage_options, converter_options);
  while (bag_reader.has_next()) {
    auto bag_message = bag_reader.read_next();

    RCLCPP_DEBUG_STREAM(get_logger(), "Found topic name " << bag_message->topic_name);

    if (bag_message->topic_name == params_.target_topic) {
      pandar_msgs::msg::PandarScan extracted_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

      RCLCPP_DEBUG_STREAM(
        get_logger(),
        "Found data in topic " << bag_message->topic_name << ": " << bag_message->time_stamp);

      auto extracted_msg_ptr = std::make_shared<pandar_msgs::msg::PandarScan>(extracted_msg);

      for (auto & pkt : extracted_msg_ptr->packets) {
        auto pointcloud_ts = driver_ptr_->parse_cloud_packet(
          std::vector<uint8_t>(pkt.data.begin(), std::next(pkt.data.begin(), pkt.size)));
        auto pointcloud = std::get<0>(pointcloud_ts);
        auto scan_timestamp = std::get<1>(pointcloud_ts);

        if (!pointcloud) {
          continue;
        }

        scan_callback(bag_message->time_stamp, scan_timestamp, pointcloud);
      }
    }
  }
}

}  // namespace nebula::ros
