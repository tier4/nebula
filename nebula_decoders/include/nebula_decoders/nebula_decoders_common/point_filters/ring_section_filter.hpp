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

#include "nebula_decoders/nebula_decoders_common/angles.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_filters/point_filter.hpp>
#include <nebula_common/util/expected.hpp>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::drivers
{

using namespace std::string_literals;  // NOLINT

class RingSectionFilter : public PointFilter
{
public:
  explicit RingSectionFilter(
    const std::vector<std::tuple<uint32_t, float, float>> & excluded_sections_deg)
  {
    uint32_t max_channel = 0;
    for (const auto & [channel, _1, _2] : excluded_sections_deg) {
      max_channel = std::max(max_channel, channel);
    }

    excluded_sections_rad_.resize(max_channel + 1);

    for (auto [channel, section_start_deg, section_end_deg] : excluded_sections_deg) {
      section_start_deg = normalize_angle(section_start_deg, 360.f);
      section_end_deg = normalize_angle(section_end_deg, 360.f);
      if (section_end_deg == section_start_deg) {
        section_end_deg += 360;
      }

      excluded_sections_rad_.at(channel).emplace_back(
        deg2rad(section_start_deg), deg2rad(section_end_deg));
    }
  }

  bool excluded(const NebulaPoint & point) override
  {
    if (point.channel >= excluded_sections_rad_.size()) return false;
    const auto & ring_sections = excluded_sections_rad_[point.channel];
    for (const auto & [section_start_rad, section_end_rad] : ring_sections) {
      bool in_excluded_region = angle_is_between(section_start_rad, section_end_rad, point.azimuth);
      if (in_excluded_region) return true;
    }

    return false;
  }

  /**
   * @brief Generate a ring section filter from a given JSON array. The array is expected to be in
   * the format [[channel_id, start_deg, end_deg], [channel_id, start_deg, end_deg], ...]. Not all
   * channels need to be specified, and channels can be specified multiple times. In that case,
   * multiple sections in the channel can be filtered.
   *
   * @param s The string to be parsed.
   * @return nebula::util::expected<RingSectionFilter<ChannelN>, std::string> A ring section
   * filter if parsed successfully, an error message otherwise.
   */
  static nebula::util::expected<RingSectionFilter, std::string> fromJson(
    const nlohmann::json & json)
  {
    std::vector<std::tuple<uint32_t, float, float>> parsed_sections;

    if (!json.is_array()) {
      return "expected JSON string to represent an array"s;
    }

    for (const auto & sector : json) {
      if (!sector.is_array()) {
        return "expected sector to be an array"s;
      }

      if (sector.size() != 3) {
        return "expected sector to be of length 3 (channel_id, start_deg, end_deg)"s;
      }

      if (!sector[0].is_number_unsigned()) {
        return "expected unsigned integer as channel ID"s;
      }

      if (!sector[1].is_number() || !sector[2].is_number()) {
        return "expected section start and end to be numbers"s;
      }

      uint32_t channel_id = sector[0];
      float start_deg = sector[1];
      float end_deg = sector[2];

      parsed_sections.emplace_back(channel_id, start_deg, end_deg);
    }

    if (parsed_sections.empty()) {
      return "expected at least one filtered-out section"s;
    }

    return RingSectionFilter(parsed_sections);
  }

private:
  std::vector<std::vector<std::pair<float, float>>> excluded_sections_rad_;
};

}  // namespace nebula::drivers
