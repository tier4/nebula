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

#pragma once

#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/diagnostics/functional_safety_monitor.hpp"
#include "nebula_hesai_decoders/diagnostics/packet_loss_monitor.hpp"
#include "nebula_hesai_decoders/hesai_driver.hpp"

#include <nebula_core_common/diagnostics/rate_bound_status.hpp>
#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <nebula_core_decoders/point_filters/blockage_mask.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace nebula::drivers
{

/// @brief Settings of the diagnostics that monitor the decoding of one sensor
struct HesaiDecoderDiagnosticsConfiguration
{
  struct FrequencyRange
  {
    double min_hz{};
    double max_hz{std::numeric_limits<double>::infinity()};
  };

  struct PointcloudPublishRate
  {
    FrequencyRange frequency_ok;
    FrequencyRange frequency_warn;
    size_t num_frame_transition{1};
  };

  struct PacketLoss
  {
    /// Number of packets lost between two diagnostic updates from which on an error is reported
    uint64_t error_threshold{};
  };

  PointcloudPublishRate pointcloud_publish_rate;
  /// Required for sensor models that support packet loss detection, unused for all others
  std::optional<PacketLoss> packet_loss;
};

/// @brief The state of everything that monitors the decoding of one sensor, at one point in time
struct HesaiDecoderDiagnostics
{
  /// The rate at which scans complete
  diagnostics::RateBoundStatus::Report pointcloud_rate;
  /// Unset for sensor models that do not support packet loss detection
  std::optional<PacketLossMonitor::Report> packet_loss;
  /// Unset for sensor models that do not support functional safety
  std::optional<FunctionalSafetyMonitor::Report> functional_safety;
};

/**
 * @brief Everything between a sensor's UDP payloads and its decoded outputs.
 *
 * The pipeline owns the decoder and the monitors that watch it. It does not depend on ROS 2: the
 * logger and clock are injected, outputs are delivered through callbacks, and the monitors return
 * typed reports.
 *
 * There are two ways to get diagnostics. An owner with its own schedule, such as a ROS 2 node with
 * a diagnostic updater, calls the `report_*()` functions whenever it wants to. An owner that
 * replays recorded data calls `set_diagnostics_schedule()`, and gets reports at a fixed period of
 * the injected clock.
 *
 * The pipeline is not thread-safe, apart from the `report_*()` functions, which can be called
 * while packets are processed. The callbacks are invoked from within `process_packet()`.
 */
class HesaiDecodePipeline
{
public:
  using pointcloud_callback_t = HesaiScanDecoder::pointcloud_callback_t;
  using blockage_mask_callback_t = point_filters::BlockageMaskPlugin::callback_t;
  using diagnostics_callback_t = std::function<void(const HesaiDecoderDiagnostics &)>;

  /**
   * @param sensor_configuration The settings of the sensor that sends the packets
   * @param calibration The calibration of that sensor
   * @param diagnostics_configuration The settings of the monitors
   * @param logger The logger that the pipeline and its components log to
   * @param clock The clock that rates and timeouts in diagnostics are measured on
   * @param pointcloud_callback Called for every completed scan
   * @param blockage_mask_callback Called for every completed scan if blockage mask output is
   * configured. Without a callback, no blockage mask is computed.
   * @throws std::runtime_error if a configuration is missing or invalid, or if the decoder cannot
   * be set up
   */
  HesaiDecodePipeline(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
    std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration,
    const HesaiDecoderDiagnosticsConfiguration & diagnostics_configuration,
    std::shared_ptr<loggers::Logger> logger, std::shared_ptr<const util::Clock> clock,
    pointcloud_callback_t pointcloud_callback,
    blockage_mask_callback_t blockage_mask_callback = nullptr);

  HesaiDecodePipeline(const HesaiDecodePipeline &) = delete;
  HesaiDecodePipeline & operator=(const HesaiDecodePipeline &) = delete;
  HesaiDecodePipeline(HesaiDecodePipeline &&) = delete;
  HesaiDecodePipeline & operator=(HesaiDecodePipeline &&) = delete;
  ~HesaiDecodePipeline() = default;

  /// @brief Decode the payload of one UDP packet. Completed scans are delivered via callbacks.
  PacketDecodeResult process_packet(const std::vector<uint8_t> & packet);

  /// @brief Replace the sensor configuration and discard the scan that is being accumulated.
  void set_sensor_configuration(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration);

  /// @brief Replace the calibration and discard the scan that is being accumulated.
  void set_calibration(std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration);

  /// @brief Discard the scan that is being accumulated.
  void reset();

  [[nodiscard]] Status status() const;

  /**
   * @brief Report the state of all monitors every `period` of the injected clock.
   *
   * The clock is read before every packet. Once `period` has passed since the last report, the
   * callback is invoked before that packet is decoded. If the clock skips several periods at
   * once, e.g. over a gap in a recording, the callback is only invoked once.
   */
  void set_diagnostics_schedule(
    std::chrono::nanoseconds period, diagnostics_callback_t diagnostics_callback);

  /// @brief The period at which the ROS 2 node updates its diagnostics: once per scan
  [[nodiscard]] std::chrono::nanoseconds default_diagnostics_period() const;

  /// @brief Report the state of all monitors. This advances the monitors like the individual
  /// `report_*()` functions do.
  [[nodiscard]] HesaiDecoderDiagnostics report_diagnostics();

  /// @brief Classify the rate at which scans complete. This advances the rate's hysteresis.
  [[nodiscard]] diagnostics::RateBoundStatus::Report report_pointcloud_rate();

  /// @brief Report the packets lost since the last call. Unset for sensor models that do not
  /// support packet loss detection.
  [[nodiscard]] std::optional<PacketLossMonitor::Report> report_packet_loss();

  /// @brief Report the most severe functional safety status since the last call. Unset for sensor
  /// models that do not support functional safety.
  [[nodiscard]] std::optional<FunctionalSafetyMonitor::Report> report_functional_safety();

private:
  [[nodiscard]] std::shared_ptr<HesaiDriver> make_driver();
  void run_scheduled_diagnostics();

  std::shared_ptr<loggers::Logger> logger_;
  std::shared_ptr<const util::Clock> clock_;
  pointcloud_callback_t pointcloud_callback_;
  blockage_mask_callback_t blockage_mask_callback_;

  diagnostics::RateBoundStatus pointcloud_rate_monitor_;
  std::optional<FunctionalSafetyMonitor> functional_safety_monitor_;
  std::optional<PacketLossMonitor> packet_loss_monitor_;

  diagnostics_callback_t diagnostics_callback_;
  std::chrono::nanoseconds diagnostics_period_{};
  std::optional<std::chrono::nanoseconds> next_diagnostics_time_;

  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration_;
  std::shared_ptr<HesaiDriver> driver_;
};

}  // namespace nebula::drivers
