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

#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/aeva_hw_interface.hpp"

#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/aeva_api.hpp"

namespace nebula::drivers
{

void AevaHwInterface::on_config_change(const std::shared_ptr<const aeva::Aeries2Config> & config)
{
  if (!reconfig_api_ || !setup_sensor_) return;

  json manifest{};
  for (uint32_t i = 0; i < g_manifest_retries && manifest.is_null(); ++i) {
    try {
      if (i > 0) {
        logger_->info("Re-trying to fetch manifest");
      }
      manifest = reconfig_api_->get_manifest();
      if (i > 0) {
        logger_->info("Manifest OK");
      }
    } catch (const std::runtime_error & e) {
      logger_->error(std::string("Could not fetch sensor manifest: ") + e.what());
      reconfig_api_ = make_reconfig_api(*config, logger_);
    }
  }

  if (manifest.is_null()) {
    throw std::runtime_error("Reached maximum retries while trying to fetch manifest");
  }

  for (const auto & category : config->tree.items()) {
    for (const auto & setting : category.value().items()) {
      try_reconfig(manifest, category.key(), setting.key(), setting.value());
    }
  }
}

void AevaHwInterface::register_cloud_packet_callback(PointcloudParser::callback_t callback)
{
  pointcloud_api_->register_callback(std::move(callback));
}

void AevaHwInterface::register_raw_cloud_packet_callback(
  connections::ObservableByteStream::callback_t callback)
{
  pointcloud_api_->register_bytes_callback(std::move(callback));
}

void AevaHwInterface::register_health_callback(HealthParser::callback_t callback)
{
  health_api_->register_callback(std::move(callback));
}

void AevaHwInterface::register_telemetry_callback(TelemetryParser::callback_t callback)
{
  telemetry_api_->register_callback(std::move(callback));
}

void AevaHwInterface::try_reconfig(
  const json & manifest, const std::string & node_name, const std::string & key, const json & value)
{
  auto old_value_opt = util::get_if_exists<json>(manifest, {node_name, key, "value"});
  if (old_value_opt && old_value_opt.value() == value) return;

  try {
    reconfig_api_->set_parameter(node_name, key, value);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Could not set " + node_name + "." + key + ": " + e.what());
  }

  // ////////////////////////////////////////
  // Value was successfully updated
  // ////////////////////////////////////////

  logger_->info("Set " + node_name + "." + key + " to " + value.dump());
}

std::shared_ptr<ReconfigParser> AevaHwInterface::make_reconfig_api(
  const aeva::Aeries2Config & config, const std::shared_ptr<loggers::Logger> & logger)
{
  return std::make_shared<ReconfigParser>(
    std::make_shared<TcpStream>(config.sensor_ip, connections::aeva::g_port_reconfig_response),
    std::make_shared<TcpSender>(config.sensor_ip, connections::aeva::g_port_reconfig_request),
    logger->child("ReconfigApi"));
}

std::shared_ptr<PointcloudParser> AevaHwInterface::make_pointcloud_api(
  const aeva::Aeries2Config & config)
{
  return std::make_shared<PointcloudParser>(
    std::make_shared<TcpStream>(config.sensor_ip, connections::aeva::g_port_spherical_point_cloud));
}

std::shared_ptr<HealthParser> AevaHwInterface::make_health_api(const aeva::Aeries2Config & config)
{
  return std::make_shared<HealthParser>(
    std::make_shared<TcpStream>(config.sensor_ip, connections::aeva::g_port_health));
}

std::shared_ptr<TelemetryParser> AevaHwInterface::make_telemetry_api(
  const aeva::Aeries2Config & config)
{
  return std::make_shared<TelemetryParser>(
    std::make_shared<TcpStream>(config.sensor_ip, connections::aeva::g_port_telemetry));
}

}  // namespace nebula::drivers
