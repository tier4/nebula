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

#include "innovusion/innovusion_ros_offline_extract_bag_pcd.hpp"

namespace nebula
{
namespace ros
{
InnovusionRosOfflineExtractBag::InnovusionRosOfflineExtractBag(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::InnovusionCalibrationConfiguration calibration_configuration;
  drivers::InnovusionSensorConfiguration sensor_configuration;

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::InnovusionCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::InnovusionSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
}

Status InnovusionRosOfflineExtractBag::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::InnovusionDriver>(
    std::static_pointer_cast<drivers::InnovusionSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::InnovusionCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status InnovusionRosOfflineExtractBag::GetStatus() {return wrapper_status_;}

Status InnovusionRosOfflineExtractBag::GetParameters(
  drivers::InnovusionSensorConfiguration & sensor_configuration,
  drivers::InnovusionCalibrationConfiguration & calibration_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "Falcon");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "Dual", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "innovusion", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "/home/demo/github2/nebula/src/nebula_decoders/calibration/innovusion/Falcon.yaml", descriptor);
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
    this->declare_parameter<double>("max_range", 500., descriptor);
    sensor_configuration.max_range = this->get_parameter("max_range").as_double();
  }
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
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("bag_path", "/home/demo/github2/nebula/bag_inno/", descriptor);
    bag_path = this->get_parameter("bag_path").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("storage_id", "sqlite3", descriptor);
    storage_id = this->get_parameter("storage_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("out_path", "/home/demo/github2/nebula/output", descriptor);
    out_path = this->get_parameter("out_path").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("format", "cdr", descriptor);
    format = this->get_parameter("format").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("out_num", 3, descriptor);
    out_num = this->get_parameter("out_num").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 2;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("skip_num", 3, descriptor);
    skip_num = this->get_parameter("skip_num").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 1;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("only_xyz", false, descriptor);
    only_xyz = this->get_parameter("only_xyz").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("target_topic", "/innovusion_packets", descriptor);
    target_topic = this->get_parameter("target_topic").as_string();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
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

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model
                                         << ", Return mode: " << sensor_configuration.return_mode);
  return Status::OK;
}

Status InnovusionRosOfflineExtractBag::ReadBag()
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
  //  return Status::OK;

  pcl::PCDWriter writer;

  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  bool needs_open = true;
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id;
  converter_options.output_serialization_format = format;  // "cdr";
  {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);
    int cnt = 0;
    int out_cnt = 0;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      std::cout << "Found topic name " << bag_message->topic_name << std::endl;

      if (bag_message->topic_name == target_topic) {
        std::cout << (cnt + 1) << ", " << (out_cnt + 1) << "/" << out_num << std::endl;
        innovusion_msgs::msg::InnovusionScan extracted_msg;
        rclcpp::Serialization<innovusion_msgs::msg::InnovusionScan> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

        std::cout << "Found data in topic " << bag_message->topic_name << ": "
                  << bag_message->time_stamp << std::endl;

        //        nebula::drivers::NebulaPointCloudPtr pointcloud =
        //        driver_ptr_->ConvertScanToPointcloud(
        //          std::make_shared<innovusion_msgs::msg::InnovusionScan>(extracted_msg));
        auto pointcloud_ts = driver_ptr_->ConvertScanToPointcloud(
          std::make_shared<innovusion_msgs::msg::InnovusionScan>(extracted_msg));
        auto pointcloud = std::get<0>(pointcloud_ts);
        auto fn = std::to_string(bag_message->time_stamp) + ".pcd";

        if (needs_open) {
          const rosbag2_storage::StorageOptions storage_options_w(
            {(o_dir / std::to_string(bag_message->time_stamp)).string(), "sqlite3"});
          const rosbag2_cpp::ConverterOptions converter_options_w(
            {rmw_get_serialization_format(), rmw_get_serialization_format()});
          writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
          writer_->open(storage_options_w, converter_options_w);
          writer_->create_topic(
            {bag_message->topic_name, "innovusion_msgs/msg/InnovusionScan",
              rmw_get_serialization_format(), ""});
          needs_open = false;
        }
        writer_->write(bag_message);
        cnt++;
        if (skip_num < cnt) {
          out_cnt++;
          if (only_xyz) {
            pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
            pcl::copyPointCloud(*pointcloud, cloud_xyz);
            writer.writeBinary((o_dir / fn).string(), cloud_xyz);
          } else {
            writer.writeBinary((o_dir / fn).string(), *pointcloud);
          }
        }
        if (out_num <= out_cnt) {
          break;
        }
      }
    }
    // close on scope exit
  }
  return Status::OK;
}

}  // namespace ros
}  // namespace nebula
