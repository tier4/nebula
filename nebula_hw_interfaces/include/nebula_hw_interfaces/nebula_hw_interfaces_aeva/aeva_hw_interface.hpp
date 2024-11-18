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

#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/health.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/pointcloud.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/reconfig.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/telemetry.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/tcp_receiver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/tcp_sender.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <utility>

namespace nebula::drivers
{

using connections::HealthParser;
using connections::PointcloudParser;
using connections::ReconfigParser;
using connections::TcpSender;
using connections::TcpStream;
using connections::TelemetryParser;
using nlohmann::json;

class AevaHwInterface
{
public:
  /**
   * @brief Construct a new AevaHwInterface which tries to connect to the sensor specified in
   * `config`. Throws on connection failure.
   *
   * @param logger the logger the HW interface will log to
   * @param config The configuration containing the sensor IP, whether to not to setup the sensor,
   * and the sensor settings
   */
  AevaHwInterface(
    const std::shared_ptr<loggers::Logger> & logger, bool setup_sensor,
    const std::shared_ptr<const aeva::Aeries2Config> & config)
  : AevaHwInterface(
      logger, setup_sensor, config, make_pointcloud_api(*config), make_telemetry_api(*config),
      make_reconfig_api(*config, logger), make_health_api(*config))
  {
  }

  /**
   * @brief Construct a new AevaHwInterface which takes in given instances of the API endpoints it
   * should use. This is useful for testing or offline playback.
   *
   * @param logger The logger the HW interface uses
   * @param config The sensor configuration
   * @param pointcloud_api The pointcloud endpoint. Can be null.
   * @param telemetry_api  The telemetry endpoint. Can be null.
   * @param reconfig_api  The reconfig endpoint. Can be null.
   * @param health_api  The health endpoint. Can be null.
   */
  AevaHwInterface(
    const std::shared_ptr<loggers::Logger> & logger, bool setup_sensor,
    const std::shared_ptr<const aeva::Aeries2Config> & config,
    std::shared_ptr<PointcloudParser> pointcloud_api,
    std::shared_ptr<TelemetryParser> telemetry_api, std::shared_ptr<ReconfigParser> reconfig_api,
    std::shared_ptr<HealthParser> health_api)
  : setup_sensor_(setup_sensor),
    logger_(logger),
    pointcloud_api_(std::move(pointcloud_api)),
    telemetry_api_(std::move(telemetry_api)),
    reconfig_api_(std::move(reconfig_api)),
    health_api_(std::move(health_api))
  {
    if (setup_sensor_ && reconfig_api_) {
      logger_->info("Configuring sensor...");
      on_config_change(config);
      logger_->info("Config OK");
    }
  }

  void on_config_change(const std::shared_ptr<const aeva::Aeries2Config> & config);

  void register_cloud_packet_callback(PointcloudParser::callback_t callback);

  void register_raw_cloud_packet_callback(connections::ObservableByteStream::callback_t callback);

  void register_health_callback(HealthParser::callback_t callback);

  void register_telemetry_callback(TelemetryParser::callback_t callback);

private:
  void try_reconfig(
    const json & manifest, const std::string & node_name, const std::string & key,
    const json & value);

  static std::shared_ptr<ReconfigParser> make_reconfig_api(
    const aeva::Aeries2Config & config, const std::shared_ptr<loggers::Logger> & logger);

  static std::shared_ptr<PointcloudParser> make_pointcloud_api(const aeva::Aeries2Config & config);

  static std::shared_ptr<HealthParser> make_health_api(const aeva::Aeries2Config & config);

  static std::shared_ptr<TelemetryParser> make_telemetry_api(const aeva::Aeries2Config & config);

  const bool setup_sensor_;
  std::shared_ptr<loggers::Logger> logger_;
  std::shared_ptr<PointcloudParser> pointcloud_api_;
  std::shared_ptr<TelemetryParser> telemetry_api_;
  std::shared_ptr<ReconfigParser> reconfig_api_;
  std::shared_ptr<HealthParser> health_api_;

  /// Fetching the sensor manifest sometimes times out. In those cases, retry the below number of
  /// times.
  static const uint8_t g_manifest_retries = 3;
};

}  // namespace nebula::drivers
