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

#include <nebula_common/aeva/packet_types.hpp>

#include <boost/endian/conversion.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

using namespace boost::endian;  //  NOLINT
using aeva::HealthCode;

class HealthParser : public AevaParser<AevaStreamType::kHealth>
{
public:
  using callback_t = std::function<void(std::vector<HealthCode>)>;

  explicit HealthParser(std::shared_ptr<PullableByteStream> incoming_byte_stream)
  : AevaParser<AevaStreamType::kHealth>(std::move(incoming_byte_stream))
  {
  }

  void registerCallback(callback_t callback) { callback_ = std::move(callback); }

protected:
  void onMessage(const MessageHeader & message_header, ByteView & payload_bytes) override
  {
    auto n_entries = pull_and_parse<uint64_t>(payload_bytes);

    expect_eq(
      n_entries * sizeof(uint32_t),
      message_header.message_length - sizeof(MessageHeader) - sizeof(n_entries),
      "Unexpected size of health code list");

    std::vector<HealthCode> entries;
    entries.reserve(n_entries);

    for (size_t i = 0; i < n_entries; ++i) {
      auto pointer = &*payload_bytes.consumeUnsafe(sizeof(uint32_t)).cbegin();
      auto entry = load_little_u32(pointer);
      entries.emplace_back(entry);
    }

    if (callback_) {
      callback_(entries);
    }
  }

private:
  callback_t callback_;
};

}  // namespace nebula::drivers::connections
