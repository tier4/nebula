// Copyright 2023 Map IV, Inc.
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

#include "velodyne/velodyne_ros_offline_extract_bag_pcd.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <vector>

namespace nebula::ros
{
VelodyneRosOfflineExtractBag::VelodyneRosOfflineExtractBag(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::VelodyneCalibrationConfiguration calibration_configuration;
  drivers::VelodyneSensorConfiguration sensor_configuration;

  wrapper_status_ = get_parameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error: " << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<const drivers::VelodyneCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ =
    std::make_shared<const drivers::VelodyneSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = initialize_driver(sensor_cfg_ptr_, calibration_cfg_ptr_);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
}

Status VelodyneRosOfflineExtractBag::initialize_driver(
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_configuration,
  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ =
    std::make_shared<drivers::VelodyneDriver>(sensor_configuration, calibration_configuration);
  return driver_ptr_->get_status();
}

Status VelodyneRosOfflineExtractBag::get_status()
{
  return wrapper_status_;
}

Status VelodyneRosOfflineExtractBag::get_parameters(
  drivers::VelodyneSensorConfiguration & sensor_configuration,
  drivers::VelodyneCalibrationConfiguration & calibration_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::sensor_model_from_string(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::return_mode_from_string(this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "velodyne", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("min_range", 0.3, descriptor);
    sensor_configuration.min_range = this->get_parameter("min_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("max_range", 300., descriptor);
    sensor_configuration.max_range = this->get_parameter("max_range").as_double();
  }
  double view_direction = sensor_configuration.scan_phase * M_PI / 180;
  double view_width = 360 * M_PI / 180;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("view_width", 300., descriptor);
    view_width = this->get_parameter("view_width").as_double() * M_PI / 180;
  }

  if (sensor_configuration.sensor_model != nebula::drivers::SensorModel::VELODYNE_HDL64) {
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.type = 2;
      descriptor.read_only = false;
      descriptor.dynamic_typing = false;
      descriptor.additional_constraints = "";
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(0).set__to_value(359).set__step(1);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
      sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
    }
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.type = 2;
      descriptor.read_only = false;
      descriptor.dynamic_typing = false;
      descriptor.additional_constraints = "";
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(0).set__to_value(359).set__step(1);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("cloud_max_angle", 359, descriptor);
      sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
    }
  } else {
    double min_angle = fmod(fmod(view_direction + view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    double max_angle = fmod(fmod(view_direction - view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    sensor_configuration.cloud_min_angle =
      static_cast<int>(std::lround(100 * (2 * M_PI - min_angle) * 180 / M_PI));
    sensor_configuration.cloud_max_angle =
      static_cast<int>(std::lround(100 * (2 * M_PI - max_angle) * 180 / M_PI));
    if (sensor_configuration.cloud_min_angle == sensor_configuration.cloud_max_angle) {
      // avoid returning empty cloud if min_angle = max_angle
      sensor_configuration.cloud_min_angle = 0;
      sensor_configuration.cloud_max_angle = 36000;
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("bag_path", "", descriptor);
    bag_path_ = this->get_parameter("bag_path").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("storage_id", "sqlite3", descriptor);
    storage_id_ = this->get_parameter("storage_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("out_path", "", descriptor);
    out_path_ = this->get_parameter("out_path").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("format", "cdr", descriptor);
    format_ = this->get_parameter("format").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("out_num", 3, descriptor);
    out_num_ = this->get_parameter("out_num").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("skip_num", 3, descriptor);
    skip_num_ = this->get_parameter("skip_num").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 1;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("only_xyz", false, descriptor);
    only_xyz_ = this->get_parameter("only_xyz").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("target_topic", "", descriptor);
    target_topic_ = this->get_parameter("target_topic").as_string();
  }

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
      calibration_configuration.load_from_file(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    }
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model
                                         << ", Return mode: " << sensor_configuration.return_mode
                                         << ", Scan Phase: " << sensor_configuration.scan_phase);
  return Status::OK;
}

Status VelodyneRosOfflineExtractBag::read_bag()
{
  rosbag2_storage::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  std::cout << bag_path_ << std::endl;
  std::cout << storage_id_ << std::endl;
  std::cout << out_path_ << std::endl;
  std::cout << format_ << std::endl;
  std::cout << target_topic_ << std::endl;
  std::cout << out_num_ << std::endl;
  std::cout << skip_num_ << std::endl;
  std::cout << only_xyz_ << std::endl;

  rcpputils::fs::path o_dir(out_path_);
  auto target_topic_name = target_topic_;
  if (target_topic_name.substr(0, 1) == "/") {
    target_topic_name = target_topic_name.substr(1);
  }
  target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");
  o_dir = o_dir / rcpputils::fs::path(target_topic_name);
  if (rcpputils::fs::create_directories(o_dir)) {
    std::cout << "created: " << o_dir << std::endl;
  }
  //  return Status::OK;

  pcl::PCDWriter pcd_writer;

  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> bag_writer;
  bool needs_open = true;
  storage_options.uri = bag_path_;
  storage_options.storage_id = storage_id_;
  converter_options.output_serialization_format = format_;  // "cdr";
  {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);
    int cnt = 0;
    int out_cnt = 0;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      std::cout << "Found topic name " << bag_message->topic_name << std::endl;

      if (bag_message->topic_name == target_topic_) {
        std::cout << (cnt + 1) << ", " << (out_cnt + 1) << "/" << out_num_ << std::endl;
        velodyne_msgs::msg::VelodyneScan extracted_msg;
        rclcpp::Serialization<velodyne_msgs::msg::VelodyneScan> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

        std::cout << "Found data in topic " << bag_message->topic_name << ": "
                  << bag_message->time_stamp << std::endl;

        //        nebula::drivers::NebulaPointCloudPtr pointcloud =
        //        driver_ptr_->ConvertScanToPointcloud(
        //          std::make_shared<velodyne_msgs::msg::VelodyneScan>(extracted_msg));

        for (auto & pkt : extracted_msg.packets) {
          auto pointcloud_ts = driver_ptr_->parse_cloud_packet(
            std::vector<uint8_t>(pkt.data.begin(), std::next(pkt.data.begin(), pkt.data.size())),
            pkt.stamp.sec);
          auto pointcloud = std::get<0>(pointcloud_ts);

          if (!pointcloud) {
            continue;
          }

          auto fn = std::to_string(bag_message->time_stamp) + ".pcd";

          if (needs_open) {
            const rosbag2_storage::StorageOptions storage_options_w(
              {(o_dir / std::to_string(bag_message->time_stamp)).string(), "sqlite3"});
            const rosbag2_cpp::ConverterOptions converter_options_w(
              {rmw_get_serialization_format(), rmw_get_serialization_format()});
            bag_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
            bag_writer->open(storage_options_w, converter_options_w);
            bag_writer->create_topic(
              {bag_message->topic_name, "velodyne_msgs/msg/VelodyneScan",
               rmw_get_serialization_format(), ""});
            needs_open = false;
          }
          bag_writer->write(bag_message);
          cnt++;
          if (skip_num_ < cnt) {
            out_cnt++;
            if (only_xyz_) {
              pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
              pcl::copyPointCloud(*pointcloud, cloud_xyz);
              pcd_writer.writeBinary((o_dir / fn).string(), cloud_xyz);
            } else {
              pcd_writer.writeBinary((o_dir / fn).string(), *pointcloud);
            }
          }
          if (out_num_ <= out_cnt) {
            break;
          }
        }
      }
    }
    // close on scope exit
  }
  return Status::OK;
}

}  // namespace nebula::ros
