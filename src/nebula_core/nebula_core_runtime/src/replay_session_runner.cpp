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

namespace nebula::drivers
{

ReplaySessionRunner::ReplaySessionRunner(std::shared_ptr<SensorRegistry> registry)
: registry_(registry)
{
}

void ReplaySessionRunner::configure(const ReplaySessionConfig & config)
{
  auto metadata = registry_->find_plugin_for_model(config.model);
  if (!metadata) {
    throw std::runtime_error("No plugin found for model");
  }

  auto plugin = registry_->load_plugin(*metadata);
  if (!plugin) {
    throw std::runtime_error("Failed to load plugin");
  }

  runtime_ = plugin->create_decoder_runtime();
  if (output_callback_) {
    runtime_->set_output_callback(output_callback_);
  }
  if (error_callback_) {
    runtime_->set_error_callback(error_callback_);
  }
  if (progress_callback_) {
    runtime_->set_progress_callback(progress_callback_);
  }
  runtime_->configure(config.sensor_config);

  router_ = std::make_unique<PacketRouter>();
  router_->configure(plugin->packet_requirements(config.sensor_config));

  source_ = std::make_unique<PcapPacketSource>();
  source_->set_error_callback(error_callback_);
  source_->open(config.pcap_file);
  source_->set_packet_callback(
    std::bind(&ReplaySessionRunner::on_packet, this, std::placeholders::_1));
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
    const auto result = runtime_->process_packet(view);
    if (result == SensorPacketResult::Error && error_callback_) {
      SensorError error;
      error.type = SensorErrorType::DecoderError;
      error.timestamp_ns = view.timestamp_ns;
      error.message = "Decoder runtime returned SensorPacketResult::Error";
      error_callback_(error);
    }
  }
}

}  // namespace nebula::drivers
