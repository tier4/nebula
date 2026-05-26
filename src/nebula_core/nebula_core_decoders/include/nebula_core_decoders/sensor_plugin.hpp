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

#ifndef NEBULA_SENSOR_PLUGIN_HPP
#define NEBULA_SENSOR_PLUGIN_HPP

#include <nebula_core_decoders/sensor_decoder_runtime.hpp>
#include <nebula_core_decoders/sensor_requirements.hpp>

#include <memory>
#include <vector>

namespace nebula::drivers
{
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

#endif  // NEBULA_SENSOR_PLUGIN_HPP
