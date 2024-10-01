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

#include "hesai/hesai_ros_offline_extract_bag_pcd.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <regex>

namespace nebula
{
namespace ros
{
HesaiRosOfflineExtractBag::HesaiRosOfflineExtractBag(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
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

Status HesaiRosOfflineExtractBag::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calibration_configuration)
{
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    calibration_configuration);
  return driver_ptr_->GetStatus();
}

Status HesaiRosOfflineExtractBag::GetStatus()
{
  return wrapper_status_;
}

Status HesaiRosOfflineExtractBag::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCorrection & correction_configuration)
{
  auto sensor_model_ = this->declare_parameter<std::string>("sensor_model", "", param_read_only());
  sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model_);
  auto return_mode_ = this->declare_parameter<std::string>("return_mode", "", param_read_only());
  sensor_configuration.return_mode =
    nebula::drivers::ReturnModeFromStringHesai(return_mode_, sensor_configuration.sensor_model);
  this->declare_parameter<std::string>("frame_id", "pandar", param_read_only());
  sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    descriptor.integer_range = int_range(0, 360, 1);
    sensor_configuration.sync_angle =
      declare_parameter<uint16_t>("sync_angle", 0., param_read_only());
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor = param_read_only();
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    descriptor.floating_point_range = float_range(0, 360, 0.01);
    sensor_configuration.cut_angle = declare_parameter<double>("cut_angle", 0., param_read_only());
  }

  calibration_configuration.calibration_file =
    this->declare_parameter<std::string>("calibration_file", "", param_read_only());
  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    correction_file_path =
      this->declare_parameter<std::string>("correction_file", "", param_read_only());
  }

  bag_path = this->declare_parameter<std::string>("bag_path", "", param_read_only());
  storage_id = this->declare_parameter<std::string>("storage_id", "sqlite3", param_read_only());
  out_path = this->declare_parameter<std::string>("out_path", "", param_read_only());
  format = this->declare_parameter<std::string>("format", "cdr", param_read_only());
  out_num = this->declare_parameter<uint16_t>("out_num", 3, param_read_only());
  skip_num = this->declare_parameter<uint16_t>("skip_num", 3, param_read_only());
  only_xyz = this->declare_parameter<bool>("only_xyz", false, param_read_only());
  target_topic = this->declare_parameter<std::string>("target_topic", "", param_read_only());

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty()) {
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
  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    if (correction_file_path.empty()) {
      return Status::INVALID_CALIBRATION_FILE;
    } else {
      auto cal_status = correction_configuration.LoadFromFile(correction_file_path);
      if (cal_status != Status::OK) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "Given Correction File: '" << correction_file_path << "'");
        return cal_status;
      }
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Sensor Configuration: " << sensor_configuration);
  return Status::OK;
}

Status HesaiRosOfflineExtractBag::ReadBag()
{
  rosbag2_storage::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  std::cout << bag_path << std::endl;
  std::cout << storage_id << std::endl;
  std::cout << out_path << std::endl;
  std::cout << format << std::endl;
  std::cout << target_topic << std::endl;
  std::cout << out_num << std::endl;
  std::cout << skip_num << std::endl;
  std::cout << only_xyz << std::endl;

  rcpputils::fs::path o_dir(out_path);
  auto target_topic_name = target_topic;
  if (target_topic_name.substr(0, 1) == "/") {
    target_topic_name = target_topic_name.substr(1);
  }
  target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");
  o_dir = o_dir / rcpputils::fs::path(target_topic_name);
  if (rcpputils::fs::create_directories(o_dir)) {
    std::cout << "created: " << o_dir << std::endl;
  }

  pcl::PCDWriter pcd_writer;

  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> bag_writer{};
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id;
  converter_options.output_serialization_format = format;

  rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  reader.open(storage_options, converter_options);
  int cnt = 0;
  int out_cnt = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();

    std::cout << "Found topic name " << bag_message->topic_name << std::endl;

    if (bag_message->topic_name != target_topic) {
      continue;
    }

    std::cout << (cnt + 1) << ", " << (out_cnt + 1) << "/" << out_num << std::endl;
    pandar_msgs::msg::PandarScan extracted_msg;
    rclcpp::Serialization<pandar_msgs::msg::PandarScan> serialization;
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

    std::cout << "Found data in topic " << bag_message->topic_name << ": "
              << bag_message->time_stamp << std::endl;

    nebula_msgs::msg::NebulaPackets nebula_msg;
    nebula_msg.header = extracted_msg.header;
    for (auto & pkt : extracted_msg.packets) {
      std::vector<uint8_t> pkt_data(pkt.data.begin(), std::next(pkt.data.begin(), pkt.size));
      auto pointcloud_ts = driver_ptr_->ParseCloudPacket(pkt_data);
      auto pointcloud = std::get<0>(pointcloud_ts);

      nebula_msgs::msg::NebulaPacket nebula_pkt;
      nebula_pkt.stamp = pkt.stamp;
      nebula_pkt.data.swap(pkt_data);  // move storage from `pkt_data` to `data`
      nebula_msg.packets.push_back(nebula_pkt);

      if (!pointcloud) {
        continue;
      }

      auto fn = std::to_string(bag_message->time_stamp) + ".pcd";

      if (!bag_writer) {
        const rosbag2_storage::StorageOptions storage_options_w(
          {(o_dir / std::to_string(bag_message->time_stamp)).string(), "sqlite3"});
        const rosbag2_cpp::ConverterOptions converter_options_w(
          {rmw_get_serialization_format(), rmw_get_serialization_format()});
        bag_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        bag_writer->open(storage_options_w, converter_options_w);
        bag_writer->create_topic(
          {bag_message->topic_name, "nebula_msgs/msg/NebulaPackets", rmw_get_serialization_format(),
           ""});
      }

      bag_writer->write(bag_message);
      cnt++;
      if (skip_num < cnt) {
        out_cnt++;
        if (only_xyz) {
          pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
          pcl::copyPointCloud(*pointcloud, cloud_xyz);
          pcd_writer.writeBinary((o_dir / fn).string(), cloud_xyz);
        } else {
          pcd_writer.writeBinary((o_dir / fn).string(), *pointcloud);
        }
      }

      if (out_num <= out_cnt) {
        break;
      }
    }
  }
  return Status::OK;
}

}  // namespace ros
}  // namespace nebula
