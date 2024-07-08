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

#include "nebula_common/util/parsing.hpp"
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
#include <stdexcept>
#include <string>
#include <type_traits>
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
    std::shared_ptr<loggers::Logger> logger, bool setup_sensor,
    const std::shared_ptr<const aeva::Aeries2Config> & config)
  : AevaHwInterface(
      logger, setup_sensor, config, makepointcloudApi(*config), makeTelemetryApi(*config),
      makeReconfigApi(*config, logger), makeHealthApi(*config))
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
    std::shared_ptr<loggers::Logger> logger, bool setup_sensor,
    const std::shared_ptr<const aeva::Aeries2Config> & config,
    std::shared_ptr<PointcloudParser> pointcloud_api,
    std::shared_ptr<TelemetryParser> telemetry_api, std::shared_ptr<ReconfigParser> reconfig_api,
    std::shared_ptr<HealthParser> health_api)
  : setup_sensor_(setup_sensor),
    logger_(std::move(logger)),
    pointcloud_api_(std::move(pointcloud_api)),
    telemetry_api_(std::move(telemetry_api)),
    reconfig_api_(std::move(reconfig_api)),
    health_api_(std::move(health_api))
  {
    if (setup_sensor_ && reconfig_api_) {
      logger_->info("Configuring sensor...");
      onConfigChange(config);
      logger_->info("Config OK");
    }
  }

  void onConfigChange(const std::shared_ptr<const aeva::Aeries2Config> & config)
  {
    if (!reconfig_api_ || !setup_sensor_) return;

    json manifest{};
    for (uint32_t i = 0; i < MANIFEST_RETRIES && manifest.is_null(); ++i) {
      try {
        if (i > 0) {
          logger_->info("Re-trying to fetch manifest");
        }
        manifest = reconfig_api_->getManifest();
        if (i > 0) {
          logger_->info("Manifest OK");
        }
      } catch (const std::runtime_error & e) {
        logger_->error(std::string("Could not fetch sensor manifest: ") + e.what());
        reconfig_api_ = makeReconfigApi(*config, logger_);
      }
    }

    if (manifest.is_null()) {
      throw std::runtime_error("Reached maximum retries while trying to fetch manifest");
    }

    tryReconfig(
      manifest, "scanner", "dithering_enable_ego_speed", config->dithering_enable_ego_speed);
    tryReconfig(manifest, "scanner", "dithering_pattern_option", config->dithering_pattern_option);
    tryReconfig(manifest, "scanner", "ele_offset_rad", config->ele_offset_rad);
    tryReconfig(
      manifest, "scanner", "elevation_auto_adjustment", config->elevation_auto_adjustment);
    tryReconfig(manifest, "scanner", "enable_frame_dithering", config->enable_frame_dithering);
    tryReconfig(manifest, "scanner", "enable_frame_sync", config->enable_frame_sync);
    tryReconfig(manifest, "scanner", "flip_pattern_vertically", config->flip_pattern_vertically);
    tryReconfig(manifest, "scanner", "frame_sync_offset_in_ms", config->frame_sync_offset_in_ms);
    tryReconfig(manifest, "scanner", "frame_sync_type", config->frame_sync_type);
    tryReconfig(
      manifest, "scanner", "frame_synchronization_on_rising_edge",
      config->frame_synchronization_on_rising_edge);
    tryReconfig(manifest, "scanner", "hfov_adjustment_deg", config->hfov_adjustment_deg);
    tryReconfig(manifest, "scanner", "hfov_rotation_deg", config->hfov_rotation_deg);
    tryReconfig(manifest, "scanner", "highlight_ROI", config->highlight_ROI);
    tryReconfig(manifest, "scanner", "horizontal_fov_degrees", config->horizontal_fov_degrees);
    tryReconfig(manifest, "scanner", "roi_az_offset_rad", config->roi_az_offset_rad);
    tryReconfig(manifest, "scanner", "vertical_pattern", config->vertical_pattern);
  }

  void registerCloudPacketCallback(PointcloudParser::callback_t callback)
  {
    pointcloud_api_->registerCallback(std::move(callback));
  }

  void registerRawCloudPacketCallback(connections::ObservableByteStream::callback_t callback)
  {
    pointcloud_api_->registerBytesCallback(std::move(callback));
  }

  void registerHealthCallback(HealthParser::callback_t callback)
  {
    health_api_->registerCallback(std::move(callback));
  }

  void registerTelemetryCallback(TelemetryParser::callback_t callback)
  {
    telemetry_api_->registerCallback(std::move(callback));
  }

private:
  template <typename T>
  void tryReconfig(
    const json & manifest, const std::string & node_name, const std::string & key, const T & value)
  {
    auto old_value_opt = util::get_if_exists<T>(manifest, {node_name, key, "value"});
    if (old_value_opt && old_value_opt.value() == value) return;

    try {
      reconfig_api_->setParameter(node_name, key, value);
    } catch (const std::runtime_error & e) {
      throw std::runtime_error("Could not set " + node_name + "." + key + ": " + e.what());
    }

    // ////////////////////////////////////////
    // Value was successfully updated
    // ////////////////////////////////////////

    std::string value_str;
    if constexpr (std::is_same_v<T, std::string>) {
      value_str = value;
    } else {
      value_str = std::to_string(value);
    }

    logger_->info("Set " + node_name + "." + key + " to " + value_str);
  }

  static std::shared_ptr<ReconfigParser> makeReconfigApi(
    const aeva::Aeries2Config & config, const std::shared_ptr<loggers::Logger> & logger)
  {
    return std::make_shared<ReconfigParser>(
      std::make_shared<TcpStream>(config.sensor_ip, 41007),
      std::make_shared<TcpSender>(config.sensor_ip, 21901), logger->child("ReconfigApi"));
  }

  static std::shared_ptr<PointcloudParser> makepointcloudApi(const aeva::Aeries2Config & config)
  {
    return std::make_shared<PointcloudParser>(std::make_shared<TcpStream>(config.sensor_ip, 41000));
  }

  static std::shared_ptr<HealthParser> makeHealthApi(const aeva::Aeries2Config & config)
  {
    return std::make_shared<HealthParser>(std::make_shared<TcpStream>(config.sensor_ip, 41001));
  }

  static std::shared_ptr<TelemetryParser> makeTelemetryApi(const aeva::Aeries2Config & config)
  {
    return std::make_shared<TelemetryParser>(std::make_shared<TcpStream>(config.sensor_ip, 41003));
  }

  const bool setup_sensor_;
  std::shared_ptr<loggers::Logger> logger_;
  std::shared_ptr<PointcloudParser> pointcloud_api_;
  std::shared_ptr<TelemetryParser> telemetry_api_;
  std::shared_ptr<ReconfigParser> reconfig_api_;
  std::shared_ptr<HealthParser> health_api_;

  // Fetching the sensor manifest sometimes times out. In those cases, retry the below number of
  // times.
  static const uint8_t MANIFEST_RETRIES = 3;
};

}  // namespace nebula::drivers
