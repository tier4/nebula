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

#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/aeva_api.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_view.hpp"

#include <nebula_common/aeva/packet_types.hpp>
#include <nlohmann/json.hpp>

#include <boost/endian/conversion.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections::aeva
{

using nebula::drivers::aeva::TelemetryDataType;
using nlohmann::json;

using namespace boost::endian;  //  NOLINT

namespace telemetry_detail
{

const std::vector<std::string> g_type_overrides = {
  "display_all_points",
  "hfov_adjustment_deg",
  "hfov_rotation_deg",
  "PTP_master_offset_in_ns",
  "target_below_or_near_min_range",
};

template <typename OutT>
inline std::vector<OutT> parse_number_array(
  std::function<OutT(const uint8_t *)> f, const ByteView::Slice & bytes)
{
  ssize_t type_size = sizeof(OutT);
  if (bytes.size() % type_size != 0) {
    throw std::runtime_error("Buffer length is not divisible by requested type's size");
  }

  size_t n_entries = bytes.size() / type_size;
  std::vector<OutT> result{};
  result.reserve(n_entries);

  for (auto it = bytes.cbegin(); it != bytes.cend(); it += type_size) {
    result.emplace_back(f(&*it));
  }

  return result;
}

inline std::string parse_string(const ByteView::Slice & bytes)
{
  return {bytes.cbegin(), bytes.cend()};
}

}  // namespace telemetry_detail

class TelemetryParser : public AevaParser<AevaStreamType::kTelemetry>
{
public:
  using callback_t = std::function<void(json)>;

  explicit TelemetryParser(std::shared_ptr<PullableByteStream> incoming_byte_stream)
  : AevaParser<AevaStreamType::kTelemetry>(std::move(incoming_byte_stream))
  {
  }

  void register_callback(callback_t callback) { callback_ = std::move(callback); }

protected:
  void on_message(const MessageHeader & message_header, ByteView & payload_bytes) override
  {
    auto payload_size = pull_and_parse<uint32_t>(payload_bytes);

    auto node_name_size = pull_and_parse<uint8_t>(payload_bytes);
    payload_bytes.consume(3).value_or_throw();  // reserved

    expect_eq(
      payload_size,
      message_header.message_length - sizeof(MessageHeader) - sizeof(payload_size) -
        sizeof(node_name_size) - 3,
      "Unexpected payload size field");

    auto node_name_raw = payload_bytes.consume(node_name_size).value_or_throw();
    auto node_name = std::string(node_name_raw.cbegin(), node_name_raw.cend());

    payload_size -= node_name_size;

    json entries = json::object();

    while (payload_size > 0) {
      auto type = pull_and_parse<TelemetryDataType>(payload_bytes);
      payload_bytes.consume(8).value_or_throw();
      auto entry_key_size = pull_and_parse<uint8_t>(payload_bytes);
      auto entry_key_raw = payload_bytes.consume(entry_key_size).value_or_throw();
      auto key = std::string(entry_key_raw.cbegin(), entry_key_raw.cend());
      auto entry_data_size = pull_and_parse<uint32_t>(payload_bytes);
      auto entry_data_raw = payload_bytes.consume(entry_data_size).value_or_throw();

      json value;
      switch (type) {
        case TelemetryDataType::kUInt8:
          value = telemetry_detail::parse_number_array<uint8_t>(
            [](const auto * ref) { return *ref; }, entry_data_raw);
          break;
        case TelemetryDataType::kInt8:
          value = telemetry_detail::parse_number_array<int8_t>(
            [](const auto * ref) { return static_cast<int8_t>(*ref); }, entry_data_raw);
          break;
        case TelemetryDataType::kUInt16:
          value = telemetry_detail::parse_number_array<uint16_t>(&load_little_u16, entry_data_raw);
          break;
        case TelemetryDataType::kInt16:
          value = telemetry_detail::parse_number_array<int16_t>(&load_little_s16, entry_data_raw);
          break;
        case TelemetryDataType::kUInt32:
          value = telemetry_detail::parse_number_array<uint32_t>(&load_little_u32, entry_data_raw);
          break;
        case TelemetryDataType::kInt32:
          value = telemetry_detail::parse_number_array<int32_t>(&load_little_s32, entry_data_raw);
          break;
        case TelemetryDataType::kUInt64:
          value = telemetry_detail::parse_number_array<uint64_t>(&load_little_u64, entry_data_raw);
          break;
        case TelemetryDataType::kInt64:
          value = telemetry_detail::parse_number_array<int64_t>(&load_little_s64, entry_data_raw);
          break;
        case TelemetryDataType::kFloat:
          value = telemetry_detail::parse_number_array<float>(
            [](const uint8_t * ref) {
              auto raw_bytes = load_little_u32(ref);
              float result{};
              memcpy(&result, &raw_bytes, 4);
              return result;
            },
            entry_data_raw);
          break;
        case TelemetryDataType::kDouble:
          value = telemetry_detail::parse_number_array<double>(
            [](const uint8_t * ref) {
              auto raw_bytes = load_little_u64(ref);
              double result{};
              memcpy(&result, &raw_bytes, 8);
              return result;
            },
            entry_data_raw);
          break;
        case TelemetryDataType::kChar:
          auto overrides = telemetry_detail::g_type_overrides;
          bool has_override = std::find(overrides.begin(), overrides.end(), key) != overrides.end();
          if (has_override) {
            uint64_t raw_value = 0;
            for (const uint8_t & it : entry_data_raw) {
              raw_value = (raw_value << 8u) | it;
            }

            value = static_cast<int64_t>(raw_value);
          } else {
            value = telemetry_detail::parse_string(entry_data_raw);
          }
          break;
      }

      if (value.is_array() && value.size() == 1) {
        value = value[0];
      }

      entries[key] = value;
      payload_size -= 1 + 8 + 1 + entry_key_size + 4 + entry_data_size;
    }

    expect_eq(payload_size, 0, "Payload and payload size mismatch");

    json result = {{node_name, entries}};

    if (callback_) {
      callback_(result);
    }
  }

private:
  callback_t callback_;
};

}  // namespace nebula::drivers::connections::aeva
