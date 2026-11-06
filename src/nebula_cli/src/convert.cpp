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

#include "convert.hpp"

#include "converters.hpp"
#include "model_detect.hpp"
#include "params.hpp"
#include "unpack.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_ros/compatibility/serialized_bag_message.hpp>
#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>


namespace nebula::cli
{
namespace
{
/// A sensor's decoder plus the bookkeeping to write its clouds
struct Sensor
{
  const SensorArgs * args;
  std::unique_ptr<nebula::cli::Converter> converter;
  uint64_t packets{0};
  uint64_t clouds{0};
};

}  // namespace

int convert(const Args & args)
{
  rosbag2_cpp::Writer writer;
  int64_t bag_time_ns = 0;  // record time of the bag message currently being decoded

  std::vector<Sensor> sensors(args.sensors.size());
  std::vector<nebula::cli::Params> all_params(sensors.size());
  for (size_t n = 0; n < sensors.size(); ++n) {
    const auto & sa = args.sensors[n];
    auto & params = all_params[n];
    if (!sa.params_file.empty()) params.load_yaml(sa.params_file);
    for (const auto & [k, v] : sa.flag_params) params.set(k, v);
    for (const auto & a : sa.assignments) params.set_assignment(a);
    sensors[n].args = &sa;
    if (sa.vendor == "robosense" && params.has("return_mode")) {
      std::fprintf(
        stderr, "warning: sensor %zu: return mode is read from the Robosense info packets; "
                "ignoring return_mode\n",
        n + 1);
    }
    if (sa.vendor == "robosense" && sa.info_topic.empty()) {
      throw std::runtime_error(
        "sensor " + std::to_string(n + 1) +
        ": Robosense needs --info-topic (calibration comes from info packets)");
    }
  }

  rosbag2_storage::StorageOptions in_options;
  in_options.uri = args.input;
  rosbag2_cpp::Reader reader;
  reader.open(in_options);

  std::map<std::string, std::string> types;
  const auto input_topics = reader.get_all_topics_and_types();
  for (const auto & t : input_topics) types[t.name] = t.type;

  // Input topic -> (sensor, is_info). Also checks that every topic exists in the bag.
  std::map<std::string, std::pair<Sensor *, bool>> routes;
  for (auto & sensor : sensors) {
    for (const bool is_info : {false, true}) {
      const auto & topic = is_info ? sensor.args->info_topic : sensor.args->topic;
      if (topic.empty()) continue;
      if (!types.count(topic)) {
        throw std::runtime_error("Topic " + topic + " not found in " + args.input);
      }
      routes[topic] = {&sensor, is_info};
    }
    if (args.copy_other && types.count(sensor.args->output_topic)) {
      throw std::runtime_error(
        "--output-topic " + sensor.args->output_topic +
        " already exists in the input bag; pick another or use --drop-other-topics");
    }
  }

  // Pre-pass: detect model / return mode from the first scan packet of each sensor that needs it
  {
    std::map<std::string, size_t> pending;  // scan topic -> sensor index
    std::vector<std::string> topics;
    for (size_t n = 0; n < sensors.size(); ++n) {
      const auto & params = all_params[n];
      const bool auto_model = params.get_string("sensor_model", "auto") == "auto";
      // Robosense takes its return mode from the info packets, and its model cannot be detected
      const bool auto_mode = sensors[n].args->vendor != "robosense" &&
                             params.get_string("return_mode", "auto") == "auto";
      if (auto_model && sensors[n].args->vendor == "robosense") {
        throw std::runtime_error(
          "sensor " + std::to_string(n + 1) +
          ": Robosense models cannot be detected; pass --model");
      }
      if (auto_model || auto_mode) {
        pending[sensors[n].args->topic] = n;
        topics.push_back(sensors[n].args->topic);
      }
    }
    if (!pending.empty()) {
      rosbag2_cpp::Reader probe;
      probe.open(in_options);
      rosbag2_storage::StorageFilter filter;
      filter.topics = topics;
      probe.set_filter(filter);
      std::vector<Packet> first;
      while (!pending.empty() && probe.has_next()) {
        const auto msg = probe.read_next();
        const auto it = pending.find(msg->topic_name);
        if (it == pending.end()) continue;
        first.clear();
        if (!unpack(types.at(msg->topic_name), *msg, first)) {
          throw std::runtime_error("Unsupported message type " + types.at(msg->topic_name));
        }
        if (first.empty()) continue;
        const size_t n = it->second;
        pending.erase(it);
        const auto & vendor = sensors[n].args->vendor;
        auto & params = all_params[n];
        try {
          // Explicit --model / --return-mode win over detection
          if (params.get_string("sensor_model", "auto") == "auto") {
            params.set("sensor_model", nebula::cli::detect_model(vendor, first.front().data));
          }
          if (vendor != "robosense" && params.get_string("return_mode", "auto") == "auto") {
            params.set(
              "return_mode", nebula::cli::detect_return_mode(
                               vendor, params.get_string("sensor_model", ""), first.front().data));
          }
        } catch (const std::exception & e) {
          throw std::runtime_error(
            "sensor " + std::to_string(n + 1) + " (" + sensors[n].args->topic + "): " + e.what());
        }
        std::fprintf(
          stderr, "%s: model %s, return mode %s\n", vendor.c_str(),
          params.get_string("sensor_model", "").c_str(),
          params.get_string("return_mode", "").c_str());
      }
      if (!pending.empty()) {
        throw std::runtime_error(
          "No packets on topic " + pending.begin()->first + " to detect the model; pass --model");
      }
    }
  }

  for (size_t n = 0; n < sensors.size(); ++n) {
    Sensor & sensor = sensors[n];
    const auto & params = all_params[n];
    const std::string frame_id = params.get_string("frame_id", "lidar");
    auto write_cloud = [&writer, &bag_time_ns, &sensor, frame_id](
                         const nebula::drivers::NebulaPointCloudPtr & cloud, double timestamp_s) {
      sensor_msgs::msg::PointCloud2 msg;
      if (sensor.args->point_format == "xyzir") {
        msg = nebula::ros::to_ros_msg(
          nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(*cloud));
      } else if (sensor.args->point_format == "xyziradt") {
        msg = nebula::ros::to_ros_msg(
          nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(*cloud, timestamp_s));
      } else {
        msg = nebula::ros::to_ros_msg(*cloud);
      }
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_s * 1e9));
      msg.header.frame_id = frame_id;
      writer.write(msg, sensor.args->output_topic, rclcpp::Time(bag_time_ns));
      ++sensor.clouds;
    };
    try {
      sensor.converter = nebula::cli::make_converter(sensor.args->vendor, params, write_cloud);
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "sensor " + std::to_string(n + 1) + " (" + sensor.args->topic + "): " + e.what());
    }
  }

  // Open the output only after the config and topics proved valid, to not leave an empty bag
  rosbag2_storage::StorageOptions out_options;
  out_options.uri = args.output;
  out_options.storage_id = args.output_storage;
  writer.open(out_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  for (const auto & sensor : sensors) {
    rosbag2_storage::TopicMetadata topic;
    topic.name = sensor.args->output_topic;
    topic.type = "sensor_msgs/msg/PointCloud2";
    topic.serialization_format = "cdr";
    writer.create_topic(topic);
  }
  if (args.copy_other) {
    for (const auto & t : input_topics) writer.create_topic(t);  // keeps QoS metadata
  }

  auto done = [&]() {
    if (args.max_clouds == 0) return false;
    for (const auto & s : sensors) {
      if (s.clouds < args.max_clouds) return false;
    }
    return true;
  };

  std::vector<Packet> batch;
  while (reader.has_next() && !done()) {
    const auto bag_msg = reader.read_next();
    if (args.copy_other) writer.write(bag_msg);  // verbatim, original timestamps
    const auto route = routes.find(bag_msg->topic_name);
    if (route == routes.end()) continue;
    auto [sensor, is_info] = route->second;

    batch.clear();
    if (!unpack(types.at(bag_msg->topic_name), *bag_msg, batch)) {
      throw std::runtime_error("Unsupported message type " + types.at(bag_msg->topic_name));
    }
    bag_time_ns = static_cast<int64_t>(nebula::ros::get_timestamp_ns(*bag_msg));
    for (const auto & p : batch) {
      if (is_info) {
        sensor->converter->feed_info(p);
      } else {
        sensor->converter->feed_scan(p);
        ++sensor->packets;
      }
    }
  }

  bool all_ok = true;
  for (const auto & s : sensors) {
    std::printf(
      "%s: decoded %" PRIu64 " packets (%" PRIu64 " dropped) into %" PRIu64 " clouds on %s\n",
      s.args->topic.c_str(), s.packets, s.converter->dropped_packets(), s.clouds,
      s.args->output_topic.c_str());
    all_ok = all_ok && s.clouds > 0;
  }
  std::printf("Output bag: %s\n", args.output.c_str());
  return all_ok ? 0 : 1;
}

}  // namespace nebula::cli
