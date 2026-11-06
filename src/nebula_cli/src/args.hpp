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

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace nebula::cli
{

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

/// Parse `nebula_cli convert ...` (argv[0] is the program, argv[1] the command).
/// Throws std::runtime_error on invalid input.
Args parse_args(int argc, char ** argv);

}  // namespace nebula::cli
