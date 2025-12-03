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

#ifndef NEBULA_SAMPLE_SCAN_DECODER_HPP
#define NEBULA_SAMPLE_SCAN_DECODER_HPP

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/expected.hpp>
#include <nebula_sample_common/sample_common.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace nebula::drivers
{
enum class DecodeError : uint8_t {
  PACKET_PARSE_FAILED,
  DRIVER_NOT_OK,
  INVALID_PACKET_SIZE,
};

struct PacketMetadata
{
  uint64_t packet_timestamp_ns{};
  bool did_scan_complete{false};
};

struct PerformanceCounters
{
  uint64_t decode_time_ns{0};
  uint64_t callback_time_ns{0};
};

struct PacketDecodeResult
{
  PerformanceCounters performance_counters;
  util::expected<PacketMetadata, DecodeError> metadata_or_error;
};

class SampleScanDecoder
{
public:
  using pointcloud_callback_t =
    std::function<void(const NebulaPointCloudPtr & pointcloud, double timestamp_s)>;

  SampleScanDecoder(SampleScanDecoder && c) = delete;
  SampleScanDecoder & operator=(SampleScanDecoder && c) = delete;
  SampleScanDecoder(const SampleScanDecoder & c) = delete;
  SampleScanDecoder & operator=(const SampleScanDecoder & c) = delete;

  virtual ~SampleScanDecoder() = default;
  SampleScanDecoder() = default;

  virtual PacketDecodeResult unpack(const std::vector<uint8_t> & packet) = 0;

  virtual void set_pointcloud_callback(pointcloud_callback_t callback) = 0;
};
}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_SCAN_DECODER_HPP
