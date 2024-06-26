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
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/aeva/packet_types.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

using aeva::ReconfigMessage;
using aeva::ReconfigRequestType;
using nlohmann::json;
using namespace std::chrono_literals;  // NOLINT

class ReconfigParser : public AevaParser<AevaStreamType::kReconfig>,
                       public AevaSender<AevaStreamType::kReconfig>
{
public:
  using callback_t = std::function<void(const ReconfigMessage &)>;

  explicit ReconfigParser(
    std::shared_ptr<PullableByteStream> incoming_byte_stream,
    std::shared_ptr<WritableByteStream> outgoing_byte_stream)
  : AevaParser<AevaStreamType::kReconfig>(std::move(incoming_byte_stream)),
    AevaSender<AevaStreamType::kReconfig>(std::move(outgoing_byte_stream))
  {
  }

  json getManifest()
  {
    ReconfigMessage request{ReconfigRequestType::kManifestRequest, {}};
    ReconfigMessage response = doRequest(request);

    if (!response.body) {
      throw std::runtime_error("Expected manifest body but got empty response");
    }

    return *response.body;
  }

  json setParameter(std::string node_name, std::string key, json value)
  {
    json request_body = {{node_name, {{key, {{"value", value}}}}}};
    std::cerr << "Sending request with body: \n" << request_body.dump(2) << std::endl;
    ReconfigMessage request = {ReconfigRequestType::kChangeRequest, request_body};

    ReconfigMessage response = doRequest(request);
    if (response.type != aeva::ReconfigRequestType::kChangeApproved) {
      std::stringstream ss{};
      ss << "change request failed";
      if (response.body) {
        ss << " with body: " << *response.body << std::endl;
      }

      throw std::runtime_error(ss.str());
    }

    if (!response.body) {
      throw std::runtime_error("Expected change request response body but got empty response");
    }

    return *response.body;
  }

private:
  ReconfigMessage doRequest(ReconfigMessage request)
  {
    std::lock_guard lock(mtx_inflight_);

    // ////////////////////////////////////////
    // Build request headers and serialize
    // ////////////////////////////////////////

    uint64_t request_id = std::rand();
    {
      std::lock_guard lock(mtx_inflight_request_id_);
      inflight_request_id_.emplace(request_id);
    }

    ReconfigRequestType type = request.type;
    uint8_t encoding_type = 0;
    uint16_t reserved = 0;

    std::vector<uint8_t> message_payload{};
    if (request.body) {
      std::string body_string = nlohmann::to_string(*request.body);
      message_payload = std::vector<uint8_t>(body_string.begin(), body_string.end());
    }

    uint32_t data_size = message_payload.size();

    std::vector<uint8_t> bytes;
    bytes.reserve(
      sizeof(request_id) + sizeof(type) + sizeof(encoding_type) + sizeof(reserved) +
      sizeof(data_size) + data_size);
    serialize(bytes, request_id);
    serialize(bytes, type);
    serialize(bytes, encoding_type);
    serialize(bytes, reserved);
    serialize(bytes, data_size);
    bytes.insert(bytes.end(), message_payload.begin(), message_payload.end());

    // ////////////////////////////////////////
    // Send and wait for response with timeout
    // ////////////////////////////////////////

    std::timed_mutex tm;
    tm.lock();

    ReconfigMessage response{};

    callback_ = [this, &tm, request_id, &response](const auto & message) {
      std::lock_guard lock(mtx_inflight_request_id_);
      if (!inflight_request_id_ || inflight_request_id_.value() != request_id) {
        // Spurious message reached this callback, the references to `response` and `tm` might not
        // be valid Exit without trying to access possibly invalid references and let request
        // timeout
        return;
      }

      response = message;
      tm.unlock();
    };

    sendMessage(bytes);
    auto request_timed_out = !tm.try_lock_for(3s);

    {
      std::lock_guard lock(mtx_inflight_request_id_);
      inflight_request_id_.reset();
    }

    if (request_timed_out) {
      throw std::runtime_error("Request timed out");
    }

    // ////////////////////////////////////////
    // Do validation and return response
    // ////////////////////////////////////////

    bool response_type_valid = (request.type == aeva::ReconfigRequestType::kManifestRequest &&
                                response.type == aeva::ReconfigRequestType::kManifestResponse) ||
                               (request.type == aeva::ReconfigRequestType::kChangeRequest &&
                                (response.type == aeva::ReconfigRequestType::kChangeApproved ||
                                 response.type == aeva::ReconfigRequestType::kChangeIgnored));

    if (!response_type_valid) {
      throw std::runtime_error("Invalid response type received");
    }

    return response;
  }

protected:
  void onMessage(const MessageHeader & message_header, ByteView & payload_bytes) override
  {
    ReconfigMessage message{};

    auto request_id = pull_and_parse<uint64_t>(payload_bytes);
    message.type = pull_and_parse<ReconfigRequestType>(payload_bytes);
    auto encoding_type = pull_and_parse<uint8_t>(payload_bytes);
    expect_eq(encoding_type, 0, "Unsupported encoding type");

    payload_bytes.consume(2).value_or_throw();

    auto data_size = pull_and_parse<uint32_t>(payload_bytes);
    auto data_raw = payload_bytes.consume(data_size).value_or_throw();

    expect_eq(
      data_size,
      message_header.message_length - sizeof(MessageHeader) - sizeof(request_id) -
        sizeof(message.type) - sizeof(encoding_type) - 2 - sizeof(data_size),
      "Unexpected data size field");

    message.body = nlohmann::json::parse(data_raw.cbegin(), data_raw.cend());

    if (callback_) {
      callback_(message);
    }
  }

private:
  callback_t callback_{};
  std::mutex mtx_inflight_{};
  std::mutex mtx_inflight_request_id_{};
  std::optional<uint64_t> inflight_request_id_{};
};

}  // namespace nebula::drivers::connections
