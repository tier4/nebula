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

#ifndef NEBULA_REPLAY_SESSION_RUNNER_HPP
#define NEBULA_REPLAY_SESSION_RUNNER_HPP

#include <nebula_core_hw_interfaces/pcap_packet_source.hpp>
#include <nebula_core_runtime/packet_router.hpp>
#include <nebula_core_runtime/sensor_registry.hpp>

#include <memory>
#include <string>

namespace nebula::drivers
{
struct ReplaySessionConfig
{
  SensorModel model;
  std::string pcap_file;
  SensorConfiguration sensor_config;
};

// Thread-safety: all set_*_callback() and configure() calls must complete before
// start() is called. Callbacks registered after start() may race with on_packet().
class ReplaySessionRunner
{
public:
  explicit ReplaySessionRunner(std::shared_ptr<SensorRegistry> registry);
  // Stops and joins the source thread while router_ and runtime_ are still alive.
  ~ReplaySessionRunner();

  void configure(const ReplaySessionConfig & config);
  void set_output_callback(SensorOutputCallback callback);
  void set_error_callback(SensorErrorCallback callback);
  void set_progress_callback(SensorProgressCallback callback);

  void start();
  void stop();
  void wait_until_finished();

private:
  void on_packet(const SensorPacket & packet);

  std::shared_ptr<SensorRegistry> registry_;
  std::unique_ptr<SensorDecoderRuntime> runtime_;
  std::unique_ptr<PcapPacketSource> source_;
  std::unique_ptr<PacketRouter> router_;

  SensorOutputCallback output_callback_;
  SensorErrorCallback error_callback_;
  SensorProgressCallback progress_callback_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_REPLAY_SESSION_RUNNER_HPP
