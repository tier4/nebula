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

#ifndef NEBULA_SENSOR_DECODER_RUNTIME_HPP
#define NEBULA_SENSOR_DECODER_RUNTIME_HPP

#include <nebula_core_common/sensor_output.hpp>
#include <nebula_core_common/sensor_packet.hpp>
#include <nebula_core_common/sensor_runtime_common.hpp>

#include <functional>
#include <memory>

namespace nebula::drivers
{
using SensorOutputCallback = std::function<void(const SensorDecodedOutput &)>;
using SensorProgressCallback = std::function<void(const SensorProgress &)>;

class SensorDecoderRuntime
{
public:
  virtual ~SensorDecoderRuntime() = default;

  virtual void configure(const SensorConfiguration & config) = 0;

  virtual void set_output_callback(SensorOutputCallback callback) = 0;
  virtual void set_error_callback(SensorErrorCallback callback) = 0;
  virtual void set_progress_callback(SensorProgressCallback callback) = 0;

  virtual SensorPacketResult process_packet(const SensorPacket & packet) = 0;
  virtual void flush() = 0;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_DECODER_RUNTIME_HPP
