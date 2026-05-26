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

#ifndef NEBULA_SENSOR_PLUGIN_EXPORT_HPP
#define NEBULA_SENSOR_PLUGIN_EXPORT_HPP

#include <nebula_core_decoders/sensor_plugin.hpp>

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin();
void destroy_nebula_sensor_plugin(nebula::drivers::SensorPlugin * plugin);
uint32_t nebula_plugin_abi_version();
}

#endif  // NEBULA_SENSOR_PLUGIN_EXPORT_HPP
