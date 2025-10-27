// Copyright 2025 TIER IV, Inc.
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

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include <nebula_common/imu_types.hpp>

#include <cstdint>
#include <functional>
#include <utility>

namespace nebula::drivers
{

class ImuDecoderBase
{
public:
  using imu_cb_t = HesaiScanDecoder::imu_callback_t;
  virtual ~ImuDecoderBase() = default;
};

template <typename PacketT>
class ImuDecoderTypedBase : public ImuDecoderBase
{
public:
  virtual void update(const PacketT & /*packet*/) {}
  virtual void set_callback(imu_cb_t /*cb*/) {}
};

template <typename PacketT>
class ImuDecoder : public ImuDecoderTypedBase<PacketT>
{
  static constexpr float g = 9.80665f;

public:
  using imu_cb_t = typename ImuDecoderBase::imu_cb_t;

  void set_callback(imu_cb_t cb) override { on_imu_ = std::move(cb); }

  void update(const PacketT & packet) override
  {
    if (!on_imu_) return;
    uint32_t imu_ts = packet.tail.imu_timestamp;
    if (imu_ts == last_imu_timestamp_) return;
    last_imu_timestamp_ = imu_ts;

    // Convert units to SI
    // Acceleration: A * U * 0.001 mg -> mg -> m/s^2
    const float accel_scale =
      static_cast<float>(packet.tail.imu_acceleration_unit) * 0.001f * 1e-3f * g;
    // Angular velocity: W * U * 0.01 mdps -> dps -> rad/s
    const float ang_scale_deg =
      static_cast<float>(packet.tail.imu_angular_velocity_unit) * 0.01f * 1e-3f;
    const float ang_scale = deg2rad(ang_scale_deg);

    ImuReading reading;
    reading.absolute_timestamp_ns = hesai_packet::get_timestamp_ns(packet);
    reading.accel_mps2_x = static_cast<float>(packet.tail.imu_x_axis_acceleration) * accel_scale;
    reading.accel_mps2_y = static_cast<float>(packet.tail.imu_y_axis_acceleration) * accel_scale;
    reading.accel_mps2_z = static_cast<float>(packet.tail.imu_z_axis_acceleration) * accel_scale;
    reading.ang_vel_rps_x = static_cast<float>(packet.tail.imu_x_axis_angular_velocity) * ang_scale;
    reading.ang_vel_rps_y = static_cast<float>(packet.tail.imu_y_axis_angular_velocity) * ang_scale;
    reading.ang_vel_rps_z = static_cast<float>(packet.tail.imu_z_axis_angular_velocity) * ang_scale;

    on_imu_(reading);
  }

private:
  uint32_t last_imu_timestamp_{0};
  imu_cb_t on_imu_;
};

}  // namespace nebula::drivers
