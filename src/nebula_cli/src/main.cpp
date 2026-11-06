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
#include "convert.hpp"

#include <cstdio>
#include <exception>
#include <string>

namespace
{
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
    return nebula::cli::convert(nebula::cli::parse_args(argc, argv));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}
