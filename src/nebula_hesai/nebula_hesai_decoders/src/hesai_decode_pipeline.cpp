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

#include "nebula_hesai_decoders/hesai_decode_pipeline.hpp"

#include "nebula_hesai_decoders/diagnostics/functional_safety_advanced.hpp"
#include "nebula_hesai_decoders/diagnostics/functional_safety_basic.hpp"

#include <nebula_core_common/util/string_conversions.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nebula::drivers
{

namespace
{

const HesaiSensorConfiguration & require(
  const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration)
{
  if (!sensor_configuration) {
    throw std::runtime_error("HesaiDecodePipeline cannot be created without a sensor config");
  }
  return *sensor_configuration;
}

diagnostics::RateBoundStatus make_pointcloud_rate_monitor(
  const HesaiSensorConfiguration & sensor_configuration,
  const HesaiDecoderDiagnosticsConfiguration::PointcloudPublishRate & config,
  loggers::Logger & logger, const std::shared_ptr<const util::Clock> & clock)
{
  const double nominal_rate_hz =
    get_scan_rate_hz(sensor_configuration.sensor_model, sensor_configuration.rotation_speed);
  const auto & ok = config.frequency_ok;
  const auto & warn = config.frequency_warn;

  // Since this is not a critical error, continue operation.
  if (nominal_rate_hz < ok.min_hz || nominal_rate_hz > ok.max_hz) {
    std::stringstream ss;
    ss << "The configured sensor framerate (" << sensor_configuration.rotation_speed
       << " RPM = " << nominal_rate_hz
       << " Hz) is outside the configured framerate bounds: " << ok.min_hz << " - " << ok.max_hz
       << ". Please check the sensor configuration.";
    logger.warn(ss.str());
  }

  return {
    clock, diagnostics::RateBoundStatusParam(ok.min_hz, ok.max_hz),
    diagnostics::RateBoundStatusParam(warn.min_hz, warn.max_hz), config.num_frame_transition};
}

std::unique_ptr<FunctionalSafetyStatusProcessor> make_functional_safety_status_processor(
  const std::optional<AdvancedFunctionalSafetyConfiguration> & config)
{
  if (!config) {
    return std::make_unique<FunctionalSafetyBasic>();
  }

  return std::make_unique<FunctionalSafetyAdvanced>(
    read_error_definitions_from_csv(config->error_definitions_path), config->ignored_error_codes);
}

}  // namespace

HesaiDecodePipeline::HesaiDecodePipeline(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
  std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration,
  const HesaiDecoderDiagnosticsConfiguration & diagnostics_configuration,
  std::shared_ptr<loggers::Logger> logger, std::shared_ptr<const util::Clock> clock,
  pointcloud_callback_t pointcloud_callback, blockage_mask_callback_t blockage_mask_callback)
: logger_(std::move(logger)),
  clock_(std::move(clock)),
  pointcloud_callback_(std::move(pointcloud_callback)),
  blockage_mask_callback_(std::move(blockage_mask_callback)),
  pointcloud_rate_monitor_(make_pointcloud_rate_monitor(
    require(sensor_configuration), diagnostics_configuration.pointcloud_publish_rate, *logger_,
    clock_)),
  sensor_configuration_(std::move(sensor_configuration)),
  calibration_(std::move(calibration))
{
  if (!calibration_) {
    throw std::runtime_error("HesaiDecodePipeline cannot be created without a calibration");
  }

  const auto sensor_model = sensor_configuration_->sensor_model;

  if (supports_functional_safety(sensor_model)) {
    functional_safety_monitor_.emplace(
      clock_, make_functional_safety_status_processor(sensor_configuration_->functional_safety));
  }

  if (supports_packet_loss_detection(sensor_model)) {
    if (!diagnostics_configuration.packet_loss) {
      throw std::runtime_error(
        "The sensor model supports packet loss detection, but it is not configured");
    }
    packet_loss_monitor_.emplace(diagnostics_configuration.packet_loss->error_threshold, clock_);
  }

  driver_ = make_driver();

  const auto driver_status = driver_->get_status();
  if (driver_status != Status::OK) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(driver_status));
  }
}

PacketDecodeResult HesaiDecodePipeline::process_packet(const std::vector<uint8_t> & packet)
{
  run_scheduled_diagnostics();
  return driver_->parse_cloud_packet(packet);
}

void HesaiDecodePipeline::set_sensor_configuration(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration)
{
  require(sensor_configuration);
  sensor_configuration_ = std::move(sensor_configuration);
  driver_ = make_driver();
}

void HesaiDecodePipeline::set_calibration(
  std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration)
{
  if (!calibration) {
    throw std::runtime_error("HesaiDecodePipeline cannot be used without a calibration");
  }
  calibration_ = std::move(calibration);
  driver_ = make_driver();
}

void HesaiDecodePipeline::reset()
{
  driver_ = make_driver();
}

Status HesaiDecodePipeline::status() const
{
  return driver_->get_status();
}

void HesaiDecodePipeline::set_diagnostics_schedule(
  std::chrono::nanoseconds period, diagnostics_callback_t diagnostics_callback)
{
  diagnostics_period_ = period;
  diagnostics_callback_ = std::move(diagnostics_callback);
  next_diagnostics_time_.reset();
}

std::chrono::nanoseconds HesaiDecodePipeline::default_diagnostics_period() const
{
  const double scan_rate_hz =
    get_scan_rate_hz(sensor_configuration_->sensor_model, sensor_configuration_->rotation_speed);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / scan_rate_hz));
}

HesaiDecoderDiagnostics HesaiDecodePipeline::report_diagnostics()
{
  HesaiDecoderDiagnostics diagnostics;
  diagnostics.pointcloud_rate = report_pointcloud_rate();
  diagnostics.packet_loss = report_packet_loss();
  diagnostics.functional_safety = report_functional_safety();
  return diagnostics;
}

diagnostics::RateBoundStatus::Report HesaiDecodePipeline::report_pointcloud_rate()
{
  return pointcloud_rate_monitor_.report();
}

std::optional<PacketLossMonitor::Report> HesaiDecodePipeline::report_packet_loss()
{
  if (!packet_loss_monitor_) {
    return std::nullopt;
  }
  return packet_loss_monitor_->report();
}

std::optional<FunctionalSafetyMonitor::Report> HesaiDecodePipeline::report_functional_safety()
{
  if (!functional_safety_monitor_) {
    return std::nullopt;
  }
  return functional_safety_monitor_->report();
}

void HesaiDecodePipeline::run_scheduled_diagnostics()
{
  if (!diagnostics_callback_) {
    return;
  }

  const auto now = clock_->now();
  if (!next_diagnostics_time_) {
    next_diagnostics_time_ = now + diagnostics_period_;
    return;
  }

  if (now < next_diagnostics_time_.value()) {
    return;
  }

  next_diagnostics_time_ = now + diagnostics_period_;
  diagnostics_callback_(report_diagnostics());
}

std::shared_ptr<HesaiDriver> HesaiDecodePipeline::make_driver()
{
  auto pointcloud_cb = [this](const NebulaPointCloudPtr & pointcloud, double timestamp_s) {
    if (pointcloud_callback_) {
      pointcloud_callback_(pointcloud, timestamp_s);
    }
    pointcloud_rate_monitor_.tick();
  };

  FunctionalSafetyDecoderBase::alive_cb_t alive_cb;
  FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb;
  FunctionalSafetyDecoderBase::status_cb_t status_cb;

  if (functional_safety_monitor_) {
    alive_cb = [this]() { functional_safety_monitor_->on_alive(); };
    stuck_cb = [this](bool is_stuck) { functional_safety_monitor_->on_stuck(is_stuck); };
    status_cb = [this](
                  FunctionalSafetySeverity severity, const FunctionalSafetyErrorCodes & codes) {
      functional_safety_monitor_->on_status(severity, codes);
    };
  }

  PacketLossDetectorBase::lost_cb_t lost_cb;

  if (packet_loss_monitor_) {
    lost_cb = [this](uint64_t n_lost) { packet_loss_monitor_->on_lost(n_lost); };
  }

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin;
  const auto & bin_size_mdeg = sensor_configuration_->blockage_mask_horizontal_bin_size_mdeg;

  if (bin_size_mdeg && blockage_mask_callback_) {
    blockage_mask_plugin =
      std::make_shared<point_filters::BlockageMaskPlugin>(bin_size_mdeg.value());
    blockage_mask_plugin->set_callback(blockage_mask_callback_);
  }

  return std::make_shared<HesaiDriver>(
    sensor_configuration_, calibration_, logger_, std::move(pointcloud_cb), std::move(alive_cb),
    std::move(stuck_cb), std::move(status_cb), std::move(lost_cb), std::move(blockage_mask_plugin));
}

}  // namespace nebula::drivers
