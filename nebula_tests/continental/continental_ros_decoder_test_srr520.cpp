// Copyright 2024 TIER IV, Inc.
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

#include "continental_ros_decoder_test_srr520.hpp"

#include "parameter_descriptors.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <gtest/gtest.h>
#include <rcutils/time.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <utility>

namespace nebula::ros
{
ContinentalRosDecoderTest::ContinentalRosDecoderTest(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::continental_srr520::ContinentalSRR520SensorConfiguration sensor_configuration;

  wrapper_status_ = get_parameters(sensor_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error: " << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  sensor_cfg_ptr_ =
    std::make_shared<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(
      sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = initialize_driver(
    std::const_pointer_cast<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(
      sensor_cfg_ptr_));

  driver_ptr_->register_hrr_detection_list_callback(
    std::bind(
      &ContinentalRosDecoderTest::hrr_detection_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_near_detection_list_callback(
    std::bind(
      &ContinentalRosDecoderTest::near_detection_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_object_list_callback(
    std::bind(&ContinentalRosDecoderTest::object_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_status_callback(
    std::bind(&ContinentalRosDecoderTest::status_callback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
}

Status ContinentalRosDecoderTest::initialize_driver(
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    sensor_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::continental_srr520::ContinentalSRR520Decoder>(
    std::static_pointer_cast<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(
      sensor_configuration));
  return Status::OK;
}

Status ContinentalRosDecoderTest::get_status()
{
  return wrapper_status_;
}

Status ContinentalRosDecoderTest::get_parameters(
  drivers::continental_srr520::ContinentalSRR520SensorConfiguration & sensor_configuration)
{
  std::filesystem::path bag_root_dir =
    _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt;
  bag_root_dir /= "continental";

  sensor_configuration.sensor_model = nebula::drivers::sensor_model_from_string(
    declare_parameter<std::string>("sensor_model", "SRR520", param_read_only()));
  sensor_configuration.frame_id =
    declare_parameter<std::string>("frame_id", "some_sensor_frame", param_read_only());
  sensor_configuration.base_frame =
    declare_parameter<std::string>("base_frame", "some_base_frame", param_read_only());

  bag_path_ = declare_parameter<std::string>(
    "bag_path", (bag_root_dir / "srr520" / "1708578209").string(), param_read_only());
  storage_id_ = declare_parameter<std::string>("storage_id", "sqlite3", param_read_only());
  format_ = declare_parameter<std::string>("format", "cdr", param_read_only());
  target_topic_ = declare_parameter<std::string>(
    "target_topic", "/sensing/radar/front_left/nebula_packets", param_read_only());

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Sensor Configuration: " << sensor_configuration);
  return Status::OK;
}

void ContinentalRosDecoderTest::compare_nodes(const YAML::Node & node1, const YAML::Node & node2)
{
  ASSERT_EQ(node1.IsDefined(), node2.IsDefined());
  ASSERT_EQ(node1.IsMap(), node2.IsMap());
  ASSERT_EQ(node1.IsNull(), node2.IsNull());
  ASSERT_EQ(node1.IsScalar(), node2.IsScalar());
  ASSERT_EQ(node1.IsSequence(), node2.IsSequence());

  if (node1.IsMap()) {
    for (YAML::const_iterator it = node1.begin(); it != node1.end(); ++it) {
      compare_nodes(it->second, node2[it->first.as<std::string>()]);
    }
  } else if (node1.IsScalar()) {
    ASSERT_EQ(node1.as<std::string>(), node2.as<std::string>());
  } else if (node1.IsSequence()) {
    ASSERT_EQ(node1.size(), node2.size());
    for (std::size_t i = 0; i < node1.size(); i++) {
      compare_nodes(node1[i], node2[i]);
    }
  }
}

void ContinentalRosDecoderTest::check_result(
  const std::string msg_as_string, const std::string & gt_path)
{
  YAML::Node current_node = YAML::Load(msg_as_string);
  YAML::Node gt_node = YAML::LoadFile(gt_path);
  compare_nodes(gt_node, current_node);

  // To generate the gt
  // std::cout << gt_path << std::endl;
  // std::ofstream ostream(gt_path);
  // ostream << msg_as_string;
  // ostream.close();
}

void ContinentalRosDecoderTest::hrr_detection_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)
{
  hrr_detection_list_count_++;
  EXPECT_EQ(sensor_cfg_ptr_->frame_id, msg->header.frame_id);
  std::string msg_as_string = continental_msgs::msg::to_yaml(*msg);

  std::stringstream detection_path;
  detection_path << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
                 << "_hrr_detection.yaml";

  auto gt_path = rcpputils::fs::path(bag_path_).parent_path() / detection_path.str();
  ASSERT_TRUE(gt_path.exists());

  check_result(msg_as_string, gt_path.string());
}

void ContinentalRosDecoderTest::near_detection_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)
{
  near_detection_list_count_++;
  EXPECT_EQ(sensor_cfg_ptr_->frame_id, msg->header.frame_id);
  std::string msg_as_string = continental_msgs::msg::to_yaml(*msg);

  std::stringstream detection_path;
  detection_path << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
                 << "_near_detection.yaml";

  auto gt_path = rcpputils::fs::path(bag_path_).parent_path() / detection_path.str();
  ASSERT_TRUE(gt_path.exists());

  check_result(msg_as_string, gt_path.string());
}

void ContinentalRosDecoderTest::object_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg)
{
  object_list_count_++;
  EXPECT_EQ(sensor_cfg_ptr_->base_frame, msg->header.frame_id);
  std::string msg_as_string = continental_msgs::msg::to_yaml(*msg);

  std::stringstream detection_path;
  detection_path << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << "_object.yaml";

  auto gt_path = rcpputils::fs::path(bag_path_).parent_path() / detection_path.str();
  ASSERT_TRUE(gt_path.exists());

  check_result(msg_as_string, gt_path.string());
}

void ContinentalRosDecoderTest::read_bag()
{
  rosbag2_storage::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  std::cout << bag_path_ << std::endl;
  std::cout << storage_id_ << std::endl;
  std::cout << format_ << std::endl;
  std::cout << target_topic_ << std::endl;

  auto target_topic_name = target_topic_;
  if (target_topic_name.substr(0, 1) == "/") {
    target_topic_name = target_topic_name.substr(1);
  }
  target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");

  rcpputils::fs::path bag_dir(bag_path_);

  storage_options.uri = bag_path_;
  storage_options.storage_id = storage_id_;
  converter_options.output_serialization_format = format_;  // "cdr";
  rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> serialization;

  {
    rosbag2_cpp::Reader bag_reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    bag_reader.open(storage_options, converter_options);
    while (bag_reader.has_next()) {
      auto bag_message = bag_reader.read_next();

      std::cout << "Found topic name " << bag_message->topic_name << std::endl;

      if (bag_message->topic_name == target_topic_) {
        nebula_msgs::msg::NebulaPackets extracted_msg;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

        std::cout << "Found data in topic " << bag_message->topic_name << ": "
                  << bag_message->time_stamp << std::endl;

        for (auto & packet_msg : extracted_msg.packets) {
          auto extracted_msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>(packet_msg);
          driver_ptr_->process_packet(std::move(extracted_msg_ptr));
        }
      }
    }
  }

  EXPECT_EQ(1, near_detection_list_count_);
  EXPECT_EQ(1, hrr_detection_list_count_);
  EXPECT_EQ(1, object_list_count_);
}

}  // namespace nebula::ros
