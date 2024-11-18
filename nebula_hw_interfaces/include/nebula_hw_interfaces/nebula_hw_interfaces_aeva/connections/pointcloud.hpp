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

#include <memory>
#include <utility>

namespace nebula::drivers::connections
{

using nebula::drivers::aeva::PointCloudMessage;
using nebula::drivers::aeva::PointcloudMsgSubheaderAndMetadata;
using nebula::drivers::aeva::PointCloudPoint;

class PointcloudParser : public AevaParser<AevaStreamType::kSphericalPointCloud>
{
public:
  using callback_t = std::function<void(const PointCloudMessage &)>;

  explicit PointcloudParser(std::shared_ptr<PullableByteStream> incoming_byte_stream)
  : AevaParser<AevaStreamType::kSphericalPointCloud>(std::move(incoming_byte_stream))
  {
  }

  void register_callback(callback_t callback) { callback_ = std::move(callback); }

protected:
  void on_message(const MessageHeader & message_header, ByteView & payload_bytes) override
  {
    PointCloudMessage message{};
    message.header = pull_and_parse<PointcloudMsgSubheaderAndMetadata>(payload_bytes);
    expect_eq(message.header.aeva_marker, 0xAE5Au, "Pointcloud Aeva marker mismatch");
    expect_eq(
      message.header.n_entries * sizeof(PointCloudPoint),
      message_header.message_length - sizeof(MessageHeader) -
        sizeof(PointcloudMsgSubheaderAndMetadata),
      "Payload bytes and point bytes mismatch");

    message.points.reserve(message.header.n_entries);
    for (size_t i = 0; i < message.header.n_entries; ++i) {
      auto point = pull_and_parse<PointCloudPoint>(payload_bytes);
      message.points.emplace_back(point);
    }

    if (callback_) {
      callback_(message);
    }
  }

private:
  callback_t callback_;
};

}  // namespace nebula::drivers::connections
