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

#include "nebula_ouster_decoders/ouster_metadata.hpp"

#include "nebula_ouster_decoders/ouster_packet.hpp"

#include <nebula_core_hw_interfaces/connections/http_client.hpp>
#include <nebula_core_hw_interfaces/connections/socket_utils.hpp>
#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers
{

namespace
{
ouster_packet::UdpProfileLidar parse_profile(const std::string & s)
{
  if (s == "LEGACY") return ouster_packet::UdpProfileLidar::LEGACY;
  if (s == "RNG19_RFL8_SIG16_NIR16") return ouster_packet::UdpProfileLidar::RNG19_RFL8_SIG16_NIR16;
  if (s == "RNG19_RFL8_SIG16_NIR16_DUAL") {
    return ouster_packet::UdpProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;
  }
  if (s == "RNG15_RFL8_NIR8") return ouster_packet::UdpProfileLidar::RNG15_RFL8_NIR8;
  throw std::runtime_error("Unsupported Ouster UDP profile: " + s);
}

uint32_t columns_per_frame_from_mode(const std::string & mode)
{
  const auto x = mode.find('x');
  if (x == std::string::npos) {
    throw std::runtime_error("Invalid lidar_mode: " + mode);
  }
  return static_cast<uint32_t>(std::stoul(mode.substr(0, x)));
}

/// @brief Fetch a sub-object from the JSON root or a nested sensor_info wrapper.
/// @details Ouster's /api/v1/sensor/metadata/sensor_info returns a flat object; some firmware
/// versions or exported metadata files wrap everything under a "sensor_info" key. Handle both.
const nlohmann::json & section(const nlohmann::json & root, const std::string & key)
{
  if (root.contains(key)) return root.at(key);
  if (root.contains("sensor_info") && root.at("sensor_info").contains(key)) {
    return root.at("sensor_info").at(key);
  }
  throw std::runtime_error("Ouster metadata: missing required section '" + key + "'");
}
}  // namespace

OusterMetadata parse_ouster_metadata(const std::string & sensor_info_json)
{
  using nlohmann::json;
  const json doc = json::parse(sensor_info_json);

  OusterMetadata md{};

  // Beam geometry — always present.
  const json & bi = section(doc, "beam_intrinsics");
  md.beam_altitude_angles_deg = bi.at("beam_altitude_angles").get<std::vector<double>>();
  md.beam_azimuth_angles_deg = bi.at("beam_azimuth_angles").get<std::vector<double>>();
  if (bi.contains("lidar_origin_to_beam_origin_mm")) {
    md.lidar_origin_to_beam_origin_mm = bi.at("lidar_origin_to_beam_origin_mm").get<double>();
  }

  // Scan geometry. Modern firmware exposes `lidar_data_format`; legacy sensors do not.
  const bool has_ldf =
    doc.contains("lidar_data_format") ||
    (doc.contains("sensor_info") && doc.at("sensor_info").contains("lidar_data_format"));

  if (has_ldf) {
    const json & ldf = section(doc, "lidar_data_format");
    md.columns_per_frame = ldf.at("columns_per_frame").get<uint32_t>();
    md.columns_per_packet = ldf.at("columns_per_packet").get<uint32_t>();
    md.pixels_per_column = ldf.at("pixels_per_column").get<uint32_t>();
    if (ldf.contains("pixel_shift_by_row")) {
      md.pixel_shift_by_row = ldf.at("pixel_shift_by_row").get<std::vector<int32_t>>();
    }
    if (ldf.contains("udp_profile_lidar")) {
      md.udp_profile_lidar = parse_profile(ldf.at("udp_profile_lidar").get<std::string>());
    }
  } else {
    md.pixels_per_column = static_cast<uint32_t>(md.beam_altitude_angles_deg.size());
    md.udp_profile_lidar = ouster_packet::UdpProfileLidar::LEGACY;
    md.columns_per_packet = 16;  // legacy default
  }

  // Optional informational fields. Also provides columns_per_frame fallback for legacy sensors.
  const json * cfg = nullptr;
  if (doc.contains("config_params")) {
    cfg = &doc.at("config_params");
  } else if (doc.contains("sensor_info") && doc.at("sensor_info").contains("config_params")) {
    cfg = &doc.at("sensor_info").at("config_params");
  }
  if (cfg != nullptr && cfg->contains("lidar_mode")) {
    md.lidar_mode = cfg->at("lidar_mode").get<std::string>();
    if (!has_ldf) md.columns_per_frame = columns_per_frame_from_mode(md.lidar_mode);
  }

  // Prod line / firmware rev live at different levels depending on firmware version.
  if (doc.contains("prod_line")) md.sensor_prod_line = doc.at("prod_line").get<std::string>();
  else if (doc.contains("sensor_info") && doc.at("sensor_info").contains("prod_line")) {
    md.sensor_prod_line = doc.at("sensor_info").at("prod_line").get<std::string>();
  }
  if (doc.contains("firmware_rev")) md.firmware_version = doc.at("firmware_rev").get<std::string>();

  // Sanity checks.
  if (md.beam_altitude_angles_deg.size() != md.pixels_per_column ||
      md.beam_azimuth_angles_deg.size() != md.pixels_per_column) {
    throw std::runtime_error(
      "Ouster metadata: beam angle arrays length does not match pixels_per_column");
  }
  if (!md.pixel_shift_by_row.empty() && md.pixel_shift_by_row.size() != md.pixels_per_column) {
    throw std::runtime_error(
      "Ouster metadata: pixel_shift_by_row length does not match pixels_per_column");
  }
  if (md.pixel_shift_by_row.empty()) {
    md.pixel_shift_by_row.assign(md.pixels_per_column, 0);
  }

  md.update_derived_sizes();
  return md;
}

std::string fetch_ouster_metadata_http(const std::string & sensor_ip, int timeout_ms)
{
  connections::HttpClient client(sensor_ip, 80);
  // Modern firmware: /api/v1/sensor/metadata returns the full document (all sections).
  // The /api/v1/sensor/metadata/sensor_info sub-endpoint only returns the sensor_info section —
  // do NOT use that one. Older firmware exposes /metadata as a legacy alias.
  try {
    return client.get("/api/v1/sensor/metadata", timeout_ms);
  } catch (const connections::SocketError &) {
    return client.get("/metadata", timeout_ms);
  }
}

}  // namespace nebula::drivers
