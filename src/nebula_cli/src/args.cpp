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

#include "args.hpp"

#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace nebula::cli
{

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
    else if (flag == "--threads") {
      a.threads = std::stoull(value);
      if (a.threads == 0) throw std::runtime_error("--threads must be at least 1");
    } else if (flag == "--vendor") sensor().vendor = value;
    else if (flag == "--topic") sensor().topic = value;
    else if (flag == "--info-topic") sensor().info_topic = value;
    else if (flag == "--output-topic") sensor().output_topic = value;
    else if (flag == "--point-format") sensor().point_format = value;
    else if (flag == "--params-file") sensor().params_file = value;
    else if (flag == "-p") sensor().assignments.push_back(value);
    else if (param_flags.count(flag))
      sensor().flag_params.emplace_back(param_flags.at(flag), value);
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
    if (!s.info_topic.empty() && s.vendor != "robosense") {
      throw std::runtime_error(id + "--info-topic is only used by Robosense");
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
}  // namespace nebula::cli
