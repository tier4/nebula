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

#include <nebula_core_runtime/live_transport_graph.hpp>

#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

LiveTransportGraph::~LiveTransportGraph()
{
  // Acquire lifecycle_mutex_ defensively to drain any orderly lifecycle call
  // (e.g. a stop() finishing on another thread). It does NOT make destruction
  // safe under truly concurrent configure()/start()/stop(): once the destructor
  // is invoked, the caller has already committed to teardown and the other
  // thread's stack frames may still reference *this after this function returns.
  // Callers must serialize destruction with lifecycle calls externally (see
  // docs/vendor_neutral_runtime_interface.md threading model).
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  // Stop sources while all graph state is still alive. C++ destroys members in reverse
  // declaration order: processing_mutex_ and mutex_ would be destroyed before sources_,
  // so source threads calling on_packet() could access dangling synchronization state.
  std::vector<std::shared_ptr<PacketSource>> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = std::move(sources_);
  }
  for (auto & src : snapshot) {
    src->stop();
  }
}

void LiveTransportGraph::configure(const LiveSessionConfig & config)
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

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

  std::vector<std::shared_ptr<PacketSource>> new_sources;
  std::map<std::string, std::shared_ptr<HttpControlEndpoint>> new_http_controls;
  auto requirements = plugin->live_transport_requirements(config.sensor_config);
  for (const auto & req : requirements) {
    if (req.transport == SensorTransportKind::UDP) {
      const auto port = require_port(req);
      auto source = std::make_shared<UdpPacketSource>();
      source->configure(config.sensor_config.host_ip, port);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      new_sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::CAN) {
      auto source = std::make_shared<CanPacketSource>();
      std::string interface = "can0";
      if (config.extra_params.count("can_interface")) {
        interface = config.extra_params.at("can_interface");
      }
      source->configure(interface);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      new_sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::TCP) {
      const auto port = require_port(req);
      auto source = std::make_shared<TcpPacketSource>();
      source->configure(config.sensor_config.sensor_ip, port);
      source->set_error_callback(
        std::bind(&LiveTransportGraph::on_error, this, std::placeholders::_1));
      source->set_packet_callback(
        std::bind(&LiveTransportGraph::on_packet, this, std::placeholders::_1));
      new_sources.push_back(std::move(source));
    } else if (req.transport == SensorTransportKind::HTTP) {
      const auto port = require_port(req);
      auto endpoint = std::make_shared<HttpControlEndpoint>();
      const std::string path = req.http_path.value_or("/");
      endpoint->configure(req.name, config.sensor_config.sensor_ip, port, path);
      new_http_controls[req.name] = std::move(endpoint);
    } else if (req.required) {
      throw std::runtime_error(
        "Unsupported required live transport requirement '" + req.name + "'");
    }
  }

  // Drain existing sources before installing new state. Must NOT hold
  // processing_mutex_ here: source threads invoke on_packet() which acquires
  // processing_mutex_, so joining them while holding it would deadlock.
  {
    std::vector<std::shared_ptr<PacketSource>> old_sources;
    std::map<std::string, std::shared_ptr<HttpControlEndpoint>> old_http_controls;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      old_sources = std::move(sources_);
      old_http_controls = std::move(http_controls_);
    }
    for (auto & src : old_sources) {
      src->stop();
    }
  }  // old sources fully stopped and destroyed before new state is installed

  // Install new state under mutex_ only. on_packet() snapshots runtime_ and router_
  // atomically under the same mutex, so the swap is invisible to in-progress calls.
  // processing_mutex_ is intentionally NOT held here: holding it would deadlock if a
  // user callback (firing from within process_packet() while processing_mutex_ is held)
  // calls configure().
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    runtime_ = std::move(runtime);
    router_ = std::move(router);
    sources_ = std::move(new_sources);
    http_controls_ = std::move(new_http_controls);
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
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  std::vector<std::shared_ptr<PacketSource>> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & src : sources_) {
      snapshot.push_back(src);
    }
  }
  for (auto & src : snapshot) {
    src->start();
  }
}

void LiveTransportGraph::stop()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  std::vector<std::shared_ptr<PacketSource>> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & src : sources_) {
      snapshot.push_back(src);
    }
  }
  for (auto & src : snapshot) {
    src->stop();
  }
}

std::string LiveTransportGraph::http_get(const std::string & endpoint_name, int timeout_ms) const
{
  // Copy the shared_ptr under the lock, then do I/O outside it. Holding mutex_
  // across a blocking network call would stall on_packet() for the full timeout.
  std::shared_ptr<HttpControlEndpoint> ep;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = http_controls_.find(endpoint_name);
    if (it == http_controls_.end()) {
      throw std::invalid_argument("No HTTP control endpoint named '" + endpoint_name + "'");
    }
    ep = it->second;
  }
  return ep->get(timeout_ms);
}

std::string LiveTransportGraph::http_post(
  const std::string & endpoint_name, const std::string & body, int timeout_ms) const
{
  std::shared_ptr<HttpControlEndpoint> ep;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = http_controls_.find(endpoint_name);
    if (it == http_controls_.end()) {
      throw std::invalid_argument("No HTTP control endpoint named '" + endpoint_name + "'");
    }
    ep = it->second;
  }
  return ep->post(body, timeout_ms);
}

void LiveTransportGraph::on_packet(const SensorPacket & packet)
{
  std::shared_ptr<SensorDecoderRuntime> runtime;
  std::shared_ptr<PacketRouter> router;
  SensorPacketView view = SensorPacketView::from(packet);

  {
    std::lock_guard<std::mutex> processing_lock(processing_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      runtime = runtime_;
      router = router_;
    }

    if (!router || !runtime) {
      return;
    }

    if (router->route(view)) {
      // Error reporting is the runtime's responsibility via set_error_callback().
      // Synthesizing a second error here would double-report for runtimes that
      // already call error_callback_ with full context before returning Error.
      runtime->process_packet(view);
    }
  }
}

void LiveTransportGraph::on_output(const SensorDecodedOutput & output)
{
  SensorOutputCallback output_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    output_callback = output_callback_;
  }

  if (!output_callback) return;

  // User-callback exceptions are non-fatal here. The graph keeps running and
  // logs to stderr; see the threading contract in
  // docs/vendor_neutral_runtime_interface.md.
  try {
    output_callback(output);
  } catch (const std::exception & e) {
    std::cerr << "LiveTransportGraph: output callback threw: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "LiveTransportGraph: output callback threw a non-std::exception" << std::endl;
  }
}

void LiveTransportGraph::on_error(const SensorError & error)
{
  SensorErrorCallback error_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_callback = error_callback_;
  }

  if (!error_callback) return;

  try {
    error_callback(error);
  } catch (const std::exception & e) {
    std::cerr << "LiveTransportGraph: error callback threw: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "LiveTransportGraph: error callback threw a non-std::exception" << std::endl;
  }
}

void LiveTransportGraph::on_progress(const SensorProgress & progress)
{
  SensorProgressCallback progress_callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_callback = progress_callback_;
  }

  if (!progress_callback) return;

  try {
    progress_callback(progress);
  } catch (const std::exception & e) {
    std::cerr << "LiveTransportGraph: progress callback threw: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "LiveTransportGraph: progress callback threw a non-std::exception" << std::endl;
  }
}

}  // namespace nebula::drivers
