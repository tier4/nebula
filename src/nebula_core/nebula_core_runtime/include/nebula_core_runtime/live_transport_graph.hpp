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

#ifndef NEBULA_LIVE_TRANSPORT_GRAPH_HPP
#define NEBULA_LIVE_TRANSPORT_GRAPH_HPP

#include <nebula_core_hw_interfaces/can_packet_source.hpp>
#include <nebula_core_hw_interfaces/http_control_endpoint.hpp>
#include <nebula_core_hw_interfaces/tcp_packet_source.hpp>
#include <nebula_core_hw_interfaces/udp_packet_source.hpp>
#include <nebula_core_runtime/packet_router.hpp>
#include <nebula_core_runtime/sensor_registry.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nebula::drivers
{
struct LiveSessionConfig
{
  SensorModel model;
  SensorConfiguration sensor_config;
  std::map<std::string, std::string> extra_params;
};

class LiveTransportGraph
{
public:
  explicit LiveTransportGraph(std::shared_ptr<SensorRegistry> registry);

  // After configure() the graph is stopped. Call start() to resume packet processing.
  void configure(const LiveSessionConfig & config);
  void set_output_callback(SensorOutputCallback callback);
  void set_error_callback(SensorErrorCallback callback);
  void set_progress_callback(SensorProgressCallback callback);

  void start();
  void stop();
  std::string http_get(const std::string & endpoint_name, int timeout_ms = 500) const;
  std::string http_post(
    const std::string & endpoint_name, const std::string & body, int timeout_ms = 500) const;

private:
  void on_packet(const SensorPacket & packet);
  void on_output(const SensorDecodedOutput & output);
  void on_error(const SensorError & error);
  void on_progress(const SensorProgress & progress);

  std::shared_ptr<SensorRegistry> registry_;
  std::shared_ptr<SensorDecoderRuntime> runtime_;
  std::vector<std::shared_ptr<PacketSource>> sources_;
  std::map<std::string, std::shared_ptr<HttpControlEndpoint>> http_controls_;
  std::shared_ptr<PacketRouter> router_;

  SensorOutputCallback output_callback_;
  SensorErrorCallback error_callback_;
  SensorProgressCallback progress_callback_;
  std::mutex lifecycle_mutex_;
  mutable std::mutex mutex_;
  std::mutex processing_mutex_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_LIVE_TRANSPORT_GRAPH_HPP
