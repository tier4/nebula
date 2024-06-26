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

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace nebula::drivers::aeva
{
using nlohmann::json;

#pragma pack(push, 1)

template <typename storage_t, size_t n_fractional_bits>
struct FixedPoint
{
  [[nodiscard]] float value() const { return value_ * std::pow(2., -1. * n_fractional_bits); }

private:
  storage_t value_;
};

template <size_t n_bytes>
struct Padding
{
  uint8_t padding[n_bytes];  // NOLINT
};

struct SomeIpHeader
{
  uint16_t service_id;
  uint16_t method_id;
  uint32_t message_length;
  uint16_t client_id;
  uint16_t session_id;
  uint8_t protocol;
  uint8_t interface_version;
  uint8_t message_type;
  uint8_t return_code;
  uint32_t tp_header_offset : 28;
  uint32_t tp_header : 4;
};

struct MessageHeader
{
  uint8_t major_version : 4;
  uint8_t minor_version : 4;
  uint8_t message_type;
  uint16_t sequence_id;
  uint32_t message_length;
  int64_t acquisition_time_ns;
  int64_t publish_time_ns;
};

struct PointcloudMsgSubheaderAndMetadata
{
  uint16_t aeva_marker;
  uint8_t platform;
  uint8_t reserved_1;
  uint16_t ns_per_index;
  uint64_t first_point_timestamp_ns;
  int32_t frame_sync_index;
  uint32_t period_ns;
  uint32_t n_entries;
  uint32_t capacity;
  uint16_t num_beams : 4;
  uint16_t num_peaks : 2;
  uint16_t range_scale : 10;
  uint8_t line_index;
  uint8_t max_line_index;
  uint32_t face_index : 4;
  uint32_t n_faces : 4;
  uint32_t reserved_2 : 3;
  uint32_t sensitivity_mode : 3;
  uint32_t frame_parity : 1;
  uint32_t reserved_3 : 1;
  uint32_t window_measurement : 1;
  uint32_t reserved_5 : 1;
  uint32_t discard_line : 1;
  uint32_t ping_pong : 1;
  uint32_t reserved_6 : 12;
  FixedPoint<int16_t, 9> mirror_rps;
  uint16_t dither_step : 5;
  uint16_t max_dither_step : 5;
  uint16_t reserved_7 : 6;
  uint8_t reserved_8[12];
};

struct PointCloudPoint
{
  FixedPoint<int16_t, 15> azimuth;
  FixedPoint<int16_t, 15> elevation;
  FixedPoint<uint16_t, 7> range;
  FixedPoint<int16_t, 8> velocity;
  uint8_t intensity;
  uint8_t signal_quality;
  uint32_t beam_id : 3;
  uint32_t peak_id : 2;
  uint32_t line_transition : 1;
  uint32_t valid : 1;
  uint32_t dynamic : 1;
  uint32_t reserved_1 : 1;
  uint32_t up_sweep : 1;
  uint32_t in_ambiguity_region : 1;
  uint32_t reserved_2 : 9;
  uint32_t peak_width : 4;
  uint32_t is_single_detection : 1;
  uint32_t reserved_3 : 7;
};

struct PointCloudMessage
{
  PointcloudMsgSubheaderAndMetadata header;
  std::vector<PointCloudPoint> points;
};

enum class TelemetryDataType : uint8_t {
  kUInt8 = 0,
  kInt8 = 1,
  kUInt16 = 2,
  kInt16 = 3,
  kUInt32 = 4,
  kInt32 = 5,
  kUInt64 = 6,
  kInt64 = 7,
  kFloat = 8,
  kDouble = 9,
  kChar = 10
};

enum class ReconfigRequestType : uint8_t {
  kManifestRequest = 0,
  kManifestResponse = 1,
  kChangeRequest = 2,
  kChangeApproved = 3,
  kChangeIgnored = 4,
  kInvalid = 5
};

inline bool is_error_code(uint32_t health_code)
{
  return (health_code & (1u << 31u)) != 0;
}

struct ReconfigMessage
{
  ReconfigRequestType type = ReconfigRequestType::kInvalid;
  std::optional<json> body;
};

#pragma pack(pop)

}  // namespace nebula::drivers::aeva
