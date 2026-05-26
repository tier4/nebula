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

#ifndef NEBULA_SENSOR_PLUGIN_HPP
#define NEBULA_SENSOR_PLUGIN_HPP

#include <nebula_core_decoders/sensor_decoder_runtime.hpp>
#include <nebula_core_decoders/sensor_requirements.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebula::drivers
{
// Increment whenever the SensorPlugin or SensorDecoderRuntime vtable layout changes.
// Plugins must export nebula_plugin_abi_version() returning this value; the registry
// rejects plugins whose version does not match.
constexpr uint32_t kNebulaPluginAbiVersion = 1;

class SensorPlugin
{
public:
  virtual ~SensorPlugin() = default;

  virtual SensorPluginMetadata metadata() const = 0;
  virtual std::vector<SensorModelInfo> supported_models() const = 0;

  virtual std::vector<PacketChannelRequirement> packet_requirements(
    const SensorConfiguration & config) const = 0;

  virtual std::vector<LiveTransportRequirement> live_transport_requirements(
    const SensorConfiguration & config) const = 0;

  virtual std::unique_ptr<SensorDecoderRuntime> create_decoder_runtime() const = 0;
};

}  // namespace nebula::drivers

// Factory and ABI version symbols exported by every plugin shared library.
extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin();
void destroy_nebula_sensor_plugin(nebula::drivers::SensorPlugin * plugin);
uint32_t nebula_plugin_abi_version();
}

#endif  // NEBULA_SENSOR_PLUGIN_HPP
