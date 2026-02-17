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
#include <utility>
#include <vector>

namespace nebula::drivers
{

const char * to_cstr(const DecodeError error)
{
  switch (error) {
    case DecodeError::PACKET_PARSE_FAILED:
      return "packet parse failed";
    case DecodeError::DRIVER_NOT_OK:
      return "decoder is not ready";
    case DecodeError::INVALID_PACKET_SIZE:
      return "invalid packet size";
    default:
      return "unknown decode error";
  }
}

SampleDecoder::SampleDecoder(
  FieldOfView<float, Degrees> fov /*, other decoder args */, pointcloud_callback_t pointcloud_cb)
: fov_(std::move(fov)), pointcloud_callback_(std::move(pointcloud_cb))
{
}

PacketDecodeResult SampleDecoder::unpack(const std::vector<uint8_t> & packet)
{
  const auto decode_begin = std::chrono::steady_clock::now();
  PacketDecodeResult result{{}, DecodeError::DRIVER_NOT_OK};

  if (!pointcloud_callback_) {
    result.metadata_or_error = DecodeError::DRIVER_NOT_OK;
  } else if (packet.empty()) {
    result.metadata_or_error = DecodeError::INVALID_PACKET_SIZE;
  } else {
    PacketMetadata metadata{};
    metadata.packet_timestamp_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count());
    metadata.did_scan_complete = false;
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
