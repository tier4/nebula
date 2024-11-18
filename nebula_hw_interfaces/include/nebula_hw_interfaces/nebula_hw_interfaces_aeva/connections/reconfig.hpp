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

#include "nebula_common/loggers/logger.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/aeva_api.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/aeva/packet_types.hpp>
#include <nlohmann/json.hpp>

#include <boost/algorithm/string/join.hpp>

#include <algorithm>
#include <atomic>
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

namespace nebula::drivers::connections::aeva
{

using drivers::aeva::ReconfigMessage;
using drivers::aeva::ReconfigRequestType;
using nlohmann::json;
using namespace std::chrono_literals;  // NOLINT

class ReconfigParser : public AevaParser<AevaStreamType::kReconfig>,
                       public AevaSender<AevaStreamType::kReconfig>
{
public:
  using callback_t = std::function<void(uint64_t, const ReconfigMessage &)>;

  explicit ReconfigParser(
    std::shared_ptr<PullableByteStream> incoming_byte_stream,
    std::shared_ptr<WritableByteStream> outgoing_byte_stream,
    std::shared_ptr<loggers::Logger> logger)
  : AevaParser<AevaStreamType::kReconfig>(std::move(incoming_byte_stream)),
    AevaSender<AevaStreamType::kReconfig>(std::move(outgoing_byte_stream)),
    logger_(std::move(logger))
  {
  }

  json get_manifest()
  {
    ReconfigMessage request{ReconfigRequestType::kManifestRequest, {}};
    auto responses = do_request(request, g_n_manifest_responses_expected);

    json result{};

    for (const auto & response : responses) {
      if (!response.body) {
        throw std::runtime_error("Expected manifest body but got empty response");
      }

      result.update(*response.body);
    }

    return result;
  }

  json set_parameter(std::string node_name, std::string key, json value)
  {
    json request_body = {{node_name, {{key, {{"value", value}}}}}};
    ReconfigMessage request = {ReconfigRequestType::kChangeRequest, request_body};

    auto response = do_request(request);

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
  ReconfigMessage do_request(const ReconfigMessage & request)
  {
    auto responses = do_request(request, 1);
    return responses.at(0);
  }

  std::vector<ReconfigMessage> do_request(
    const ReconfigMessage & request, size_t n_responses_expected)
  {
    std::lock_guard lock(mtx_inflight_);

    // ////////////////////////////////////////
    // Build request headers and serialize
    // ////////////////////////////////////////

    uint64_t request_id = std::rand();

    NEBULA_LOG_STREAM(logger_->debug, "Sent " << request.type << ", id=" << request_id);

    ReconfigRequestType type = request.type;
    uint8_t encoding_type = 0;
    uint16_t reserved = 0;

    std::vector<uint8_t> message_payload{};
    if (request.body) {
      std::string body_string = nlohmann::to_string(*request.body);
      message_payload = std::vector<uint8_t>(body_string.begin(), body_string.end());
    }

    auto data_size = static_cast<uint32_t>(message_payload.size());

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

    std::timed_mutex tm_callback_timeout;
    tm_callback_timeout.lock();

    std::mutex mtx_responses;
    std::vector<ReconfigMessage> responses{};
    responses.reserve(n_responses_expected);

    auto request_valid = std::make_shared<std::atomic_bool>(true);

    callback_ = [this, request_valid, n_responses_expected, request_id, &responses, &mtx_responses,
                 &tm_callback_timeout](uint64_t response_id, const auto & message) {
      if (!*request_valid) {
        NEBULA_LOG_STREAM(
          logger_->error, "Received " << message.type << ", id=" << response_id
                                      << "for no longer valid request id=" << request_id);
        return;
      }

      if (response_id != request_id) {
        // Spurious message reached this callback, the references to `response` and `tm` might not
        // be valid Exit without trying to access possibly invalid references and let request
        // timeout
        NEBULA_LOG_STREAM(
          logger_->error, "Spurious " << message.type << " received, id=" << response_id
                                      << ". Expected id=" << request_id);
        return;
      }

      {
        std::lock_guard lock(mtx_responses);
        responses.push_back(message);

        if (responses.size() == n_responses_expected) {
          tm_callback_timeout.unlock();
        }
      }
    };

    send_message(bytes);
    auto request_timed_out = !tm_callback_timeout.try_lock_for(20s);

    {
      *request_valid = false;
      callback_ = {};
    }

    if (request_timed_out) {
      throw std::runtime_error("Request timed out");
    }

    // ////////////////////////////////////////
    // Do validation and return response
    // ////////////////////////////////////////

    auto response_type_valid = [&](const ReconfigMessage & response) {
      return (request.type == aeva::ReconfigRequestType::kManifestRequest &&
              response.type == aeva::ReconfigRequestType::kManifestResponse) ||
             (request.type == aeva::ReconfigRequestType::kChangeRequest &&
              (response.type == aeva::ReconfigRequestType::kChangeApproved ||
               response.type == aeva::ReconfigRequestType::kChangeIgnored));
    };

    std::vector<std::string> response_types;
    response_types.reserve(responses.size());
    for (const auto & response : responses) {
      response_types.push_back((std::stringstream{} << response.type).str());
    }

    NEBULA_LOG_STREAM(
      logger_->debug, "Got " + boost::join(response_types, ", ") + " for "
                        << request.type << ", id=" << request_id);

    if (!std::all_of(responses.cbegin(), responses.cend(), response_type_valid)) {
      std::stringstream msg{};
      msg << "Invalid responses for " << request.type << ", id=" << request_id << ": ";
      msg << boost::join(response_types, "; ");
      throw std::runtime_error(msg.str());
    }

    return responses;
  }

protected:
  void on_message(const MessageHeader & message_header, ByteView & payload_bytes) override
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
      callback_(request_id, message);
    }
  }

private:
  std::shared_ptr<loggers::Logger> logger_;
  callback_t callback_;
  std::mutex mtx_inflight_;

  // scanner, calibration, system_config, spc_converter, dsp_control, self_test, window_measurement
  static const size_t g_n_manifest_responses_expected = 7;
};

}  // namespace nebula::drivers::connections::aeva
