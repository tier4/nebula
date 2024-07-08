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

#pragma once

#include "nebula_common/nebula_common.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <ostream>
#include <string>

namespace nebula::drivers::aeva
{

using nlohmann::json;

struct Aeries2Config : public SensorConfigurationBase
{
  std::string sensor_ip;
  json tree;
};

inline std::ostream & operator<<(std::ostream & os, const Aeries2Config & arg)
{
  os << "Aeva Aeries2 Sensor Configuration:\n";
  os << "Sensor Model: " << arg.sensor_model << '\n';
  os << "Frame ID: " << arg.frame_id << '\n';
  os << "Sensor IP: " << arg.sensor_ip;

  for (const auto & category : arg.tree.items()) {
    os << '\n' << category.key() << ":";
    auto category_settings = category.value();
    for (const auto & setting : category_settings.items()) {
      os << "\n  " << setting.key() << ": " << setting.value();
    }
  }

  return os;
}

}  // namespace nebula::drivers::aeva
