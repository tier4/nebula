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

#include <nebula_core_runtime/replay_session_runner.hpp>

#include <iostream>
#include <memory>
#include <utility>

namespace nebula::drivers
{

ReplaySessionRunner::ReplaySessionRunner(std::shared_ptr<SensorRegistry> registry)
: registry_(registry)
{
}

ReplaySessionRunner::~ReplaySessionRunner()
{
  // Stop and join the source thread while router_ and runtime_ are still alive.
  // Member destruction order would destroy router_ before source_ joins its thread,
  // leaving on_packet() with a dangling router_ pointer.
  if (source_) {
    source_->stop();
    source_.reset();
  }
}

void ReplaySessionRunner::configure(const ReplaySessionConfig & config)
{
  // Stop and release any in-flight source before replacing runtime_ and router_:
  // on_packet() reads both pointers without a lock. Clear the old member state
  // before constructing replacements so a later exception leaves the runner
  // unconfigured rather than half-configured.
  if (source_) {
    source_->stop();
  }
  source_.reset();
  router_.reset();
  runtime_.reset();

  auto metadata = registry_->find_plugin_for_model(config.model);
  if (!metadata) {
    throw std::runtime_error("No plugin found for model");
  }

  auto plugin = registry_->load_plugin(*metadata);
  if (!plugin) {
    throw std::runtime_error("Failed to load plugin");
  }

  auto runtime = plugin->create_decoder_runtime();
  if (!runtime) {
    throw std::runtime_error("Plugin returned null decoder runtime");
  }
  if (output_callback_) {
    runtime->set_output_callback(output_callback_);
  }
  if (error_callback_) {
    runtime->set_error_callback(error_callback_);
  }
  if (progress_callback_) {
    runtime->set_progress_callback(progress_callback_);
  }
  runtime->configure(config.sensor_config);

  auto router = std::make_unique<PacketRouter>();
  router->configure(plugin->packet_requirements(config.sensor_config));

  auto source = std::make_unique<PcapPacketSource>();
  source->set_error_callback(error_callback_);
  source->open(config.pcap_file);
  source->set_packet_callback(
    std::bind(&ReplaySessionRunner::on_packet, this, std::placeholders::_1));

  runtime_ = std::move(runtime);
  router_ = std::move(router);
  source_ = std::move(source);
}

void ReplaySessionRunner::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
  if (runtime_) {
    runtime_->set_output_callback(output_callback_);
  }
}

void ReplaySessionRunner::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
  if (runtime_) {
    runtime_->set_error_callback(error_callback_);
  }
  if (source_) {
    source_->set_error_callback(error_callback_);
  }
}

void ReplaySessionRunner::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
  if (runtime_) {
    runtime_->set_progress_callback(progress_callback_);
  }
}

void ReplaySessionRunner::start()
{
  if (source_) {
    source_->start();
  }
}

void ReplaySessionRunner::stop()
{
  if (source_) {
    source_->stop();
  }
}

void ReplaySessionRunner::wait_until_finished()
{
  if (source_) {
    source_->wait_until_finished();
  }
}

void ReplaySessionRunner::on_packet(const SensorPacket & packet)
{
  SensorPacketView view = SensorPacketView::from(packet);
  if (router_->route(view)) {
    // Error reporting is the runtime's responsibility via set_error_callback().
    runtime_->process_packet(view);
  }
}

}  // namespace nebula::drivers
