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

#ifndef NEBULA_HESAI_DRIVER_H
#define NEBULA_HESAI_DRIVER_H

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_decoders/nebula_decoders_common/point_filters/blockage_mask.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/packet_loss_detector.hpp"

#include <nebula_common/loggers/logger.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace nebula::drivers
{
/// @brief Hesai driver
class HesaiDriver
{
private:
  /// @brief Current driver status
  Status driver_status_;
  std::shared_ptr<loggers::Logger> logger_;
  /// @brief Decoder according to the model
  std::shared_ptr<HesaiScanDecoder> scan_decoder_;

  template <typename SensorT>
  std::shared_ptr<HesaiScanDecoder> initialize_decoder(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration,
    FunctionalSafetyDecoderBase::alive_cb_t alive_cb,
    FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb,
    FunctionalSafetyDecoderBase::status_cb_t status_cb, PacketLossDetectorBase::lost_cb_t lost_cb,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin = nullptr);

  template <typename SensorT>
  std::enable_if_t<
    hesai_packet::HasFunctionalSafety<typename SensorT::packet_t>::value,
    std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>>
  initialize_functional_safety_decoder(
    FunctionalSafetyDecoderBase::alive_cb_t alive_cb,
    FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb,
    FunctionalSafetyDecoderBase::status_cb_t status_cb, uint16_t sensor_rpm)
  {
    auto functional_safety_decoder =
      std::make_shared<FunctionalSafetyDecoder<typename SensorT::packet_t>>(sensor_rpm);
    functional_safety_decoder->set_alive_callback(std::move(alive_cb));
    functional_safety_decoder->set_stuck_callback(std::move(stuck_cb));
    functional_safety_decoder->set_status_callback(std::move(status_cb));
    return functional_safety_decoder;
  }

  template <typename SensorT>
  std::enable_if_t<
    !hesai_packet::HasFunctionalSafety<typename SensorT::packet_t>::value,
    std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>>
  initialize_functional_safety_decoder(
    FunctionalSafetyDecoderBase::alive_cb_t /* alive_cb */,
    FunctionalSafetyDecoderBase::stuck_cb_t /* stuck_cb */,
    FunctionalSafetyDecoderBase::status_cb_t /* status_cb */, uint16_t /* sensor_rpm */)
  {
    return nullptr;
  }

  template <typename SensorT>
  std::enable_if_t<
    hesai_packet::HasPacketLossDetection<typename SensorT::packet_t>::value,
    std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>>>
  initialize_packet_loss_detector(PacketLossDetectorBase::lost_cb_t lost_cb)
  {
    auto packet_loss_detector = std::make_shared<PacketLossDetector<typename SensorT::packet_t>>();
    packet_loss_detector->set_lost_callback(std::move(lost_cb));
    return packet_loss_detector;
  }

  template <typename SensorT>
  std::enable_if_t<
    !hesai_packet::HasPacketLossDetection<typename SensorT::packet_t>::value,
    std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>>>
  initialize_packet_loss_detector(PacketLossDetectorBase::lost_cb_t /* lost_cb */)
  {
    return nullptr;
  }

public:
  HesaiDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver (either
  /// HesaiCalibrationConfiguration for sensors other than AT128 or HesaiCorrection for AT128)
  HesaiDriver(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration,
    const std::shared_ptr<loggers::Logger> & logger,
    HesaiScanDecoder::pointcloud_callback_t pointcloud_cb,
    FunctionalSafetyDecoderBase::alive_cb_t alive_cb = nullptr,
    FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb = nullptr,
    FunctionalSafetyDecoderBase::status_cb_t status_cb = nullptr,
    PacketLossDetectorBase::lost_cb_t lost_cb = nullptr,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin = nullptr);

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status set_calibration_configuration(
    const HesaiCalibrationConfigurationBase & calibration_configuration);

  void set_pointcloud_callback(HesaiScanDecoder::pointcloud_callback_t pointcloud_cb);

  /// @brief Decode a pointcloud packet. If a pointcloud is produced, `pointcloud_cb` is called.
  /// @param packet Packet to decode
  /// @return Expected containing metadata on success, or decode error on failure
  nebula::util::expected<PacketMetadata, DecodeError> parse_cloud_packet(
    const std::vector<uint8_t> & packet);
};

}  // namespace nebula::drivers

#endif  // NEBULA_HESAI_DRIVER_H
