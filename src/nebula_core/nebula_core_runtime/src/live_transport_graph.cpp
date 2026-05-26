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

#include <nebula_core_runtime/live_transport_graph.hpp>

#include <iostream>
#include <stdexcept>

namespace nebula::drivers
{
namespace
{
uint16_t require_port(const LiveTransportRequirement & requirement)
{
  if (!requirement.port.has_value()) {
    throw std::runtime_error(
      "Live transport requirement '" + requirement.name + "' is missing a port");
  }
  return *requirement.port;
}
}  // namespace

LiveTransportGraph::LiveTransportGraph(std::shared_ptr<SensorRegistry> registry)
: registry_(registry)
{
}

void LiveTransportGraph::configure(const LiveSessionConfig & config)
{
  auto metadata = registry_->find_plugin_for_model(config.model);
  if (!metadata) {
    throw std::runtime_error("No plugin found for model");
  }

  auto plugin = registry_->load_plugin(*metadata);
  if (!plugin) {
    throw std::runtime_error("Failed to load plugin");
  }

  auto runtime_unique = plugin->create_decoder_runtime();
  auto runtime = std::shared_ptr<SensorDecoderRuntime>(std::move(runtime_unique));
  runtime->set_output_callback(
    std::bind(&LiveTransportGraph::on_output, this, std::placeholders::_1));
  runtime->set_error_callback(
    std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
  runtime->set_progress_callback(
    std::bind(&LiveTransportGraph::on_progress, this, std::placeholders::_1));
  runtime->configure(config.sensor_config);

  auto router = std::make_shared<PacketRouter>();
  router->configure(plugin->packet_requirements(config.sensor_config));

  std::vector<std::unique_ptr<PacketSource>> sources;
  std::map<std::string, std::unique_ptr<HttpControlEndpoint>> http_controls;
  auto requirements = plugin->live_transport_requirements(config.sensor_config);
  for (const auto & req : requirements) {
    if (req.transport == SensorTransportKind::UDP) {
      const auto port = require_port(req);
      auto source = std::make_unique<UdpPacketSource>();
      source->configure(config.sensor_config.host_ip, port);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::CAN) {
      auto source = std::make_unique<CanPacketSource>();
      std::string interface = "can0";
      if (config.extra_params.count("can_interface")) {
        interface = config.extra_params.at("can_interface");
      }
      source->configure(interface);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::TCP) {
      const auto port = require_port(req);
      auto source = std::make_unique<TcpPacketSource>();
      source->configure(config.sensor_config.sensor_ip, port);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::HTTP) {
      const auto port = require_port(req);
      auto endpoint = std::make_unique<HttpControlEndpoint>();
      const std::string path = req.http_path.value_or("/");
      endpoint->configure(req.name, config.sensor_config.sensor_ip, port, path);
      http_controls[req.name] = std::move(endpoint);
    } else if (req.required) {
      throw std::runtime_error(
        "Unsupported required live transport requirement '" + req.name + "'");
    }
  }

  std::vector<std::unique_ptr<PacketSource>> old_sources;
  std::map<std::string, std::unique_ptr<HttpControlEndpoint>> old_http_controls;
  std::lock_guard<std::mutex> processing_lock(processing_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    old_sources = std::move(sources_);
    old_http_controls = std::move(http_controls_);
    runtime_ = std::move(runtime);
    router_ = std::move(router);
    sources_ = std::move(sources);
    http_controls_ = std::move(http_controls);
  }
}

void LiveTransportGraph::set_output_callback(SensorOutputCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  output_callback_ = callback;
}

void LiveTransportGraph::set_error_callback(SensorErrorCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_callback_ = callback;
}

void LiveTransportGraph::set_progress_callback(SensorProgressCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  progress_callback_ = callback;
}

void LiveTransportGraph::start()
{
  for (auto & source : sources_) {
    source->start();
  }
}

void LiveTransportGraph::stop()
{
  for (auto & source : sources_) {
    source->stop();
  }
}

std::string LiveTransportGraph::http_get(const std::string & endpoint_name, int timeout_ms) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto endpoint = http_controls_.find(endpoint_name);
  if (endpoint == http_controls_.end()) {
    throw std::invalid_argument("No HTTP control endpoint named '" + endpoint_name + "'");
  }
  return endpoint->second->get(timeout_ms);
}

std::string LiveTransportGraph::http_post(
  const std::string & endpoint_name, const std::string & body, int timeout_ms) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto endpoint = http_controls_.find(endpoint_name);
  if (endpoint == http_controls_.end()) {
    throw std::invalid_argument("No HTTP control endpoint named '" + endpoint_name + "'");
  }
  return endpoint->second->post(body, timeout_ms);
}

void LiveTransportGraph::on_packet(const SensorPacket & packet)
{
  std::shared_ptr<SensorDecoderRuntime> runtime;
  std::shared_ptr<PacketRouter> router;
  SensorErrorCallback error_callback;
  SensorError error;
  bool emit_error = false;
  SensorPacket mutable_packet = packet;

  {
    std::lock_guard<std::mutex> processing_lock(processing_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      runtime = runtime_;
      router = router_;
      error_callback = error_callback_;
    }

    if (!router || !runtime) {
      return;
    }

    if (router->route(mutable_packet)) {
      const auto result = runtime->process_packet(mutable_packet);
      emit_error = result == SensorPacketResult::Error && static_cast<bool>(error_callback);
    }

    if (emit_error) {
      error.type = SensorErrorType::DecoderError;
      error.timestamp_ns = mutable_packet.timestamp_ns;
      error.message = "Decoder runtime returned SensorPacketResult::Error";
    }
  }

  if (emit_error && error_callback) {
    error_callback(error);
  }
}

void LiveTransportGraph::on_output(const SensorDecodedOutput & output)
{
  SensorOutputCallback output_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    output_callback = output_callback_;
  }

  if (output_callback) {
    output_callback(output);
  }
}

void LiveTransportGraph::on_error(const SensorError & error)
{
  SensorErrorCallback error_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_callback = error_callback_;
  }

  if (error_callback) {
    error_callback(error);
  }
}

void LiveTransportGraph::on_progress(const SensorProgress & progress)
{
  SensorProgressCallback progress_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_callback = progress_callback_;
  }

  if (progress_callback) {
    progress_callback(progress);
  }
}

}  // namespace nebula::drivers
