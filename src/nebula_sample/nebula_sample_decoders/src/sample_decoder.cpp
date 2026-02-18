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

#include "nebula_sample_decoders/sample_decoder.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace nebula::drivers
{

const char * to_cstr(const DecodeError error)
{
  switch (error) {
    case DecodeError::PACKET_FORMAT_INVALID:
      return "packet format invalid";
    case DecodeError::CALLBACK_NOT_SET:
      return "pointcloud callback is not set";
    case DecodeError::EMPTY_PACKET:
      return "packet is empty";
    default:
      return "unknown decode error";
  }
}

SampleDecoder::SampleDecoder(
  FieldOfView<float, Degrees> fov /*, other decoder args */, pointcloud_callback_t pointcloud_cb)
: fov_(fov), pointcloud_callback_(std::move(pointcloud_cb))
{
  // Implement: Initialize sensor-specific decode state (buffers, angle tracking, timestamps).
}

PacketDecodeResult SampleDecoder::unpack(const std::vector<uint8_t> & packet)
{
  const auto decode_begin = std::chrono::steady_clock::now();
  PacketDecodeResult result{{}, DecodeError::CALLBACK_NOT_SET};

  if (!pointcloud_callback_) {
    result.metadata_or_error = DecodeError::CALLBACK_NOT_SET;
  } else if (packet.empty()) {
    result.metadata_or_error = DecodeError::EMPTY_PACKET;
  } else {
    ++packet_count_;

    NebulaPoint point{};
    point.x = static_cast<float>(packet_count_);
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = packet.front();
    point.return_type = 0U;
    point.channel = 0U;
    point.azimuth = fov_.azimuth.start;
    point.elevation = fov_.elevation.start;
    point.distance = static_cast<float>(packet.size());
    point.time_stamp = static_cast<uint32_t>(packet_count_);
    current_scan_cloud_->push_back(point);

    PacketMetadata metadata{};
    metadata.packet_timestamp_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count());
    metadata.did_scan_complete = (packet_count_ % k_packets_per_dummy_scan) == 0;

    if (metadata.did_scan_complete) {
      const auto callback_begin = std::chrono::steady_clock::now();
      pointcloud_callback_(
        current_scan_cloud_, static_cast<double>(metadata.packet_timestamp_ns) * 1e-9);
      result.performance_counters.callback_time_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now() - callback_begin)
                                .count());
      current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
    }

    result.metadata_or_error = metadata;
  }

  result.performance_counters.decode_time_ns =
    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - decode_begin)
                            .count());
  return result;
}

void SampleDecoder::set_pointcloud_callback(pointcloud_callback_t pointcloud_cb)
{
  pointcloud_callback_ = std::move(pointcloud_cb);
}

}  // namespace nebula::drivers
