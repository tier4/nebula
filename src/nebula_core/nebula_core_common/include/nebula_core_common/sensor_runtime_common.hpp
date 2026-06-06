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

#ifndef NEBULA_SENSOR_RUNTIME_COMMON_HPP
#define NEBULA_SENSOR_RUNTIME_COMMON_HPP

#include <nebula_core_common/nebula_common.hpp>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace nebula::drivers
{
// Plugin-identity fields are owned by the plugin's metadata() method:
// vendor, package_name, library_path, factory_symbol, supported_models.
// Filesystem-path fields (descriptor_path, package_share_path, schema_path,
// config_defaults_path, calibration_assets_path) are populated by SensorRegistry
// from the plugin descriptor JSON and are empty when obtained directly via
// SensorPlugin::metadata().
struct SensorPluginMetadata
{
  std::string vendor;
  std::string package_name;
  std::string library_path;
  std::string factory_symbol;
  std::string descriptor_path;
  std::string package_share_path;
  std::string schema_path;
  std::string config_defaults_path;
  std::string calibration_assets_path;
  std::vector<SensorModel> supported_models;
};

struct SensorProgress
{
  uint64_t processed_packets = 0;
  uint64_t matched_packets = 0;
  uint64_t dropped_packets = 0;
  uint64_t decoded_packets = 0;
  uint64_t output_count = 0;
  uint64_t error_count = 0;
};

enum class SensorErrorType {
  None,
  ConfigError,
  TransportError,
  ProtocolError,
  DecoderError,
  InternalError,
};

struct SensorError
{
  SensorErrorType type{SensorErrorType::None};
  std::string message;
  uint64_t timestamp_ns{0};
};

using SensorErrorCallback = std::function<void(const SensorError &)>;

enum class SensorPacketResult {
  Success,
  Buffered,
  Ignored,
  Error,
};

struct SensorConfiguration : public LidarConfigurationBase
{
  uint16_t gnss_port{0};

  std::string calibration_file;

  struct
  {
    struct
    {
      float start{0};
      float end{360};
    } azimuth;
    struct
    {
      float start{-90};
      float end{90};
    } elevation;
  } fov;

  double rotation_speed{600};

  // Generic key-value store for vendor-specific parameters
  std::map<std::string, std::string> extra_params;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_RUNTIME_COMMON_HPP
