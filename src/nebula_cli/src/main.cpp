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
#include "converters.hpp"
#include "params.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_ros/compatibility/serialized_bag_message.hpp>
#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace
{
using nebula::cli::Packet;

constexpr const char * kUsage = R"(Usage: nebula_cli convert --input <bag> --output <new bag> [global options]
         [--sensor] --vendor <hesai|velodyne|robosense> --topic <packets topic>
                    [--model <model>] [--return-mode <mode>] [sensor options]
         [--sensor ... ]   (repeat to convert several topics, each with its own vendor/config)

Reads packets from a rosbag, decodes them with the Nebula drivers and writes PointCloud2 to a new bag.
Supported packet topic types: pandar_msgs/msg/PandarScan, velodyne_msgs/msg/VelodyneScan,
robosense_msgs/msg/RobosenseScan, nebula_msgs/msg/NebulaPackets.

Global options:
  --input PATH          input bag (directory or file)
  --output PATH         output bag directory (must not exist)
  --output-storage ID   sqlite3 (default) | mcap
  --max-clouds N        stop once every sensor produced N clouds (other topics are then copied
                        only up to that point)
  --drop-other-topics   do not copy the input bag's other topics (by default all are copied as-is)

Sensor options (apply to the latest --sensor; without --sensor there is one implicit sensor):
  --sensor              start a new sensor
  --vendor V            hesai | velodyne | robosense
  --topic T             packets topic to decode
  --info-topic T        Robosense only: RobosenseInfoPacket topic (provides calibration, required)
  --output-topic T      PointCloud2 topic to write (default /points; required with several sensors)
  --model M             sensor model, e.g. Pandar64, VLP16, Helios (default auto: Hesai by packet
                        size, Velodyne by product ID; Robosense must be given)
  --return-mode R       e.g. Dual, Strongest, Last (default auto: read from the packets; Robosense
                        always takes it from the info packets)
  --calibration-file F  calibration file (default: the one shipped with the decoder package;
                        for PandarAT128 set the correction file with -p correction_file:=F)
  --frame-id ID         frame_id of the output clouds (default: lidar)
  --params-file F       ROS param YAML (e.g. src/nebula_velodyne/nebula_velodyne/config/VLP16.param.yaml)
  -p name:=value        any other driver parameter (cut_angle, sync_angle, scan_phase,
                        rotation_speed, cloud_min_angle, cloud_max_angle, min_range, max_range, ...)
  --point-format F      full (default, XYZIRCAEDT) | xyzir | xyziradt
Precedence: defaults < --params-file < flags < -p)";

/// Stamp conversion shared by all packet message types
int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return rclcpp::Time(t).nanoseconds();
}

template <typename Msg>
Msg deserialize(const rosbag2_storage::SerializedBagMessage & bag_msg)
{
  Msg msg;
  rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
  rclcpp::Serialization<Msg>().deserialize_message(&serialized, &msg);
  return msg;
}

/// Packets of a message of a supported type. Returns false if the type is unsupported.
bool unpack(
  const std::string & type, const rosbag2_storage::SerializedBagMessage & m,
  std::vector<Packet> & out)
{
  if (type == "pandar_msgs/msg/PandarScan") {
    for (const auto & p : deserialize<pandar_msgs::msg::PandarScan>(m).packets) {
      // The array is fixed-size; `size` is the valid length
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.begin() + p.size}});
    }
  } else if (type == "velodyne_msgs/msg/VelodyneScan") {
    for (const auto & p : deserialize<velodyne_msgs::msg::VelodyneScan>(m).packets) {
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
    }
  } else if (type == "robosense_msgs/msg/RobosenseScan") {
    for (const auto & p : deserialize<robosense_msgs::msg::RobosenseScan>(m).packets) {
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
    }
  } else if (type == "robosense_msgs/msg/RobosenseInfoPacket") {
    const auto p = deserialize<robosense_msgs::msg::RobosenseInfoPacket>(m).packet;
    out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
  } else if (type == "nebula_msgs/msg/NebulaPackets") {
    for (const auto & p : deserialize<nebula_msgs::msg::NebulaPackets>(m).packets) {
      out.push_back({to_ns(p.stamp), p.data});
    }
  } else {
    return false;
  }
  return true;
}

/// Settings of one sensor (one packets topic -> one PointCloud2 topic)
struct SensorArgs
{
  std::string vendor, topic, info_topic, output_topic, point_format{"full"}, params_file;
  std::vector<std::pair<std::string, std::string>> flag_params;  // applied over the params file
  std::vector<std::string> assignments;                          // -p, applied last
};

struct Args
{
  std::string input, output, output_storage{"sqlite3"};
  std::vector<SensorArgs> sensors;
  uint64_t max_clouds{0};
  bool copy_other{true};
};

Args parse_args(int argc, char ** argv)
{
  const std::map<std::string, std::string> param_flags{
    {"--model", "sensor_model"},
    {"--return-mode", "return_mode"},
    {"--calibration-file", "calibration_file"},
    {"--frame-id", "frame_id"}};
  Args a;
  // Per-sensor flags apply to the latest `--sensor`; without any, they form one implicit sensor.
  auto sensor = [&a]() -> SensorArgs & {
    if (a.sensors.empty()) a.sensors.emplace_back();
    return a.sensors.back();
  };
  for (int i = 2; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--drop-other-topics") {
      a.copy_other = false;
      continue;
    }
    if (flag == "--sensor") {
      a.sensors.emplace_back();
      continue;
    }
    if (i + 1 >= argc) throw std::runtime_error("Missing value for " + flag);
    const std::string value = argv[++i];
    if (flag == "--input") a.input = value;
    else if (flag == "--output") a.output = value;
    else if (flag == "--output-storage") a.output_storage = value;
    else if (flag == "--max-clouds") a.max_clouds = std::stoull(value);
    else if (flag == "--vendor") sensor().vendor = value;
    else if (flag == "--topic") sensor().topic = value;
    else if (flag == "--info-topic") sensor().info_topic = value;
    else if (flag == "--output-topic") sensor().output_topic = value;
    else if (flag == "--point-format") sensor().point_format = value;
    else if (flag == "--params-file") sensor().params_file = value;
    else if (flag == "-p") sensor().assignments.push_back(value);
    else if (param_flags.count(flag)) sensor().flag_params.emplace_back(param_flags.at(flag), value);
    else throw std::runtime_error("Unknown option " + flag);
  }

  if (a.input.empty()) throw std::runtime_error("Missing required option --input");
  if (a.output.empty()) throw std::runtime_error("Missing required option --output");
  if (a.sensors.empty()) throw std::runtime_error("No sensor given (use --vendor and --topic)");
  std::set<std::string> used_topics;
  for (size_t n = 0; n < a.sensors.size(); ++n) {
    auto & s = a.sensors[n];
    const std::string id = "sensor " + std::to_string(n + 1) + ": ";
    if (s.vendor.empty()) throw std::runtime_error(id + "missing --vendor");
    if (s.topic.empty()) throw std::runtime_error(id + "missing --topic");
    if (s.output_topic.empty()) {
      if (a.sensors.size() > 1) throw std::runtime_error(id + "missing --output-topic");
      s.output_topic = "/points";
    }
    if (s.point_format != "full" && s.point_format != "xyzir" && s.point_format != "xyziradt") {
      throw std::runtime_error(id + "unknown --point-format " + s.point_format);
    }
    // A topic may feed one sensor only, and an output must not collide with any other topic
    for (const auto & t : {s.topic, s.info_topic}) {
      if (!t.empty() && !used_topics.insert(t).second) {
        throw std::runtime_error(id + "topic " + t + " is used by more than one sensor");
      }
    }
  }
  for (const auto & s : a.sensors) {
    if (!used_topics.insert(s.output_topic).second) {
      throw std::runtime_error("output topic " + s.output_topic + " collides with another topic");
    }
  }
  return a;
}

/// A sensor's decoder plus the bookkeeping to write its clouds
struct Sensor
{
  const SensorArgs * args;
  std::unique_ptr<nebula::cli::Converter> converter;
  uint64_t packets{0};
  uint64_t clouds{0};
};

int convert(const Args & args)
{
  rosbag2_cpp::Writer writer;
  int64_t bag_time_ns = 0;  // record time of the bag message currently being decoded

  std::vector<Sensor> sensors(args.sensors.size());
  for (size_t n = 0; n < sensors.size(); ++n) {
    const auto & sa = args.sensors[n];
    nebula::cli::Params params;
    if (!sa.params_file.empty()) params.load_yaml(sa.params_file);
    for (const auto & [k, v] : sa.flag_params) params.set(k, v);
    for (const auto & a : sa.assignments) params.set_assignment(a);

    Sensor & sensor = sensors[n];
    sensor.args = &sa;
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
      sensor.converter = nebula::cli::make_converter(sa.vendor, params, write_cloud);
    } catch (const std::exception & e) {
      throw std::runtime_error("sensor " + std::to_string(n + 1) + " (" + sa.topic + "): " + e.what());
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
      if (!types.count(topic)) throw std::runtime_error("Topic " + topic + " not found in " + args.input);
      routes[topic] = {&sensor, is_info};
    }
    if (args.copy_other && types.count(sensor.args->output_topic)) {
      throw std::runtime_error(
        "--output-topic " + sensor.args->output_topic +
        " already exists in the input bag; pick another or use --drop-other-topics");
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
      "%s: decoded %lu packets (%lu dropped) into %lu clouds on %s\n", s.args->topic.c_str(),
      s.packets, s.converter->dropped_packets(), s.clouds, s.args->output_topic.c_str());
    all_ok = all_ok && s.clouds > 0;
  }
  std::printf("Output bag: %s\n", args.output.c_str());
  return all_ok ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    std::puts(kUsage);
    return argc < 2 ? 2 : 0;
  }
  if (std::string(argv[1]) != "convert") {
    std::fprintf(stderr, "Unknown command '%s'\n%s\n", argv[1], kUsage);
    return 2;
  }
  try {
    return convert(parse_args(argc, argv));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}
