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

#include "nebula_decoders/nebula_decoders_common/point_filters/ring_section_filter.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_filters/point_filter.hpp>
#include <nebula_common/util/expected.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{

using nlohmann::json;
using namespace std::string_literals;  // NOLINT

inline nebula::util::expected<std::vector<std::shared_ptr<PointFilter>>, std::string>
parse_point_filters(const std::string & s, SensorModel sensor_model)
{
  if (s.empty()) {
    return std::vector<std::shared_ptr<PointFilter>>{};
  }

  std::vector<std::shared_ptr<PointFilter>> parsed_filters;

  json j;
  try {
    j = json::parse(s);
  } catch (json::parse_error & e) {
    return "Could not parse JSON: "s + e.what();
  }

  if (!j.is_object()) {
    return "expected a JSON object"s;
  }

  for (const auto & [key, value] : j.items()) {
    if (key == "ring_section_filter") {
      auto parsed = RingSectionFilter::fromJson(value, sensor_model);
      if (!parsed.has_value()) {
        return "Could not parse " + key + ": " + parsed.error();
      }

      auto filter = std::make_shared<RingSectionFilter>(parsed.value());
      parsed_filters.emplace_back(filter);
    } else {
      return "unknown filter type: " + key;
    }
  }

  return parsed_filters;
}

}  // namespace nebula::drivers
