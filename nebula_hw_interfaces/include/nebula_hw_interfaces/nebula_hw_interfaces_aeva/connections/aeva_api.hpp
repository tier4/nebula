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

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_view.hpp"

#include <nebula_common/aeva/packet_types.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class ParseError : public std::exception
{
};

class MismatchError : public ParseError
{
public:
  MismatchError(const std::string & message, int64_t expected, int64_t actual)
  : expected(expected),
    actual(actual),
    message_(message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual))
  {
  }

  const int64_t expected;
  const int64_t actual;

  [[nodiscard]] const char * what() const noexcept override { return message_.c_str(); }

private:
  const std::string message_;
};

static const uint16_t AEVA_HEADER = 0xAEFA;

template <typename A, typename E>
void expect_eq(A actual, E expected, const std::string & message)
{
  auto cast_actual = static_cast<int64_t>(actual);
  auto cast_expected = static_cast<int64_t>(expected);
  if (cast_actual != cast_expected) throw MismatchError(message, cast_expected, cast_actual);
}

template <typename A, typename E>
void expect_geq(A actual, E expected, const std::string & message)
{
  auto cast_actual = static_cast<int64_t>(actual);
  auto cast_expected = static_cast<int64_t>(expected);
  if (cast_actual < cast_expected) throw MismatchError(message, cast_expected, cast_actual);
}

enum class AevaStreamType : uint16_t {
  kSphericalPointCloud = 0,
  kHealth = 1,
  kConfig = 2,
  kTelemetry = 3,
  kVelocityEstimate = 4,
  kCalibration = 5,
  kImage = 6,
  kReconfig = 7,
  kVehicleStateEstimate = 8,
  kLog = 9,
  kImu = 10,
  kObjectList = 12,
  kEstimatedDetectionRange = 33,
  kUnknown = 0xFFFFu
};

using nebula::drivers::aeva::MessageHeader;
using nebula::drivers::aeva::SomeIpHeader;

template <typename T>
T pull_and_parse(
  const std::vector<uint8_t>::const_iterator & cbegin,
  const std::vector<uint8_t>::const_iterator & cend)
{
  if (std::distance(cbegin, cend) != sizeof(T)) {
    throw std::runtime_error("Number of bytes provided does not match type's size.");
  }

  T result{};
  memcpy(&result, &*cbegin, sizeof(T));
  return result;
}

template <typename T>
T pull_and_parse(const std::vector<uint8_t> & stream)
{
  return pull_and_parse<T>(stream.cbegin(), stream.cend());
}

template <typename T>
T pull_and_parse(PullableByteStream & stream)
{
  std::vector<uint8_t> buffer;
  stream.read(buffer, sizeof(T));
  return pull_and_parse<T>(buffer);
}

template <typename T>
T pull_and_parse(ByteView & stream)
{
  auto consumed = stream.consume(sizeof(T)).value_or_throw();
  return pull_and_parse<T>(consumed.cbegin(), consumed.cend());
}

template <typename T>
void serialize(std::vector<uint8_t> & out_bytes, const T & object)
{
  size_t old_size = out_bytes.size();
  size_t new_size = old_size + sizeof(T);
  out_bytes.resize(new_size);
  memcpy(&out_bytes.at(old_size), &object, sizeof(T));
}

/**
 * @brief Handles a single datastream from an Aeva sensor. While this API can
 * decode any supported stream type, routing several different streams through
 * one API instance is not supported. Instead, create one instance per data
 * stream.
 */
template <AevaStreamType StreamId>
class AevaParser : public ObservableByteStream
{
public:
  explicit AevaParser(std::shared_ptr<PullableByteStream> incoming_byte_stream)
  : incoming_(std::move(incoming_byte_stream))
  {
    if (!incoming_) {
      throw std::runtime_error("Incoming byte stream cannot be null");
    }

    thread_ = std::thread([&]() {
      while (true) onLowLevelMessage();
    });
  }

  void registerBytesCallback(callback_t callback) override
  {
    bytes_callback_ = std::move(callback);
  }

protected:
  /**
   * @brief Attempts to read one message from the stream, blocking. Parses the SomeIP and
   * MessageHeader part of an API message before calling the higher-level `onMessage` parser.
   */
  void onLowLevelMessage()
  {
    std::vector<uint8_t> some_ip_raw;
    incoming_->read(some_ip_raw, sizeof(SomeIpHeader));
    if (bytes_callback_) {
      bytes_callback_(some_ip_raw);
    }

    auto some_ip = pull_and_parse<SomeIpHeader>(some_ip_raw);
    expect_eq(some_ip.service_id, 0xAEFAu, "Aeva service header mismatch");
    expect_eq(some_ip.method_id, StreamId, "Unexpected method ID");
    auto payload_length = some_ip.message_length - 12;

    std::vector<uint8_t> payload_raw;
    incoming_->read(payload_raw, payload_length);
    if (bytes_callback_) {
      bytes_callback_(payload_raw);
    }

    auto payload_view = ByteView(payload_raw);

    auto message_header = pull_and_parse<MessageHeader>(payload_view);
    expect_eq(message_header.message_type, StreamId, "Unexpected message type");
    expect_eq(message_header.message_length, payload_length, "Payload size mismatch");

    onMessage(message_header, payload_view);
  }

  virtual void onMessage(const MessageHeader & message_header, ByteView & payload_bytes) = 0;

private:
  std::thread thread_;
  std::shared_ptr<PullableByteStream> incoming_;
  callback_t bytes_callback_;
};

template <AevaStreamType StreamId>
class AevaSender
{
public:
  explicit AevaSender(std::shared_ptr<WritableByteStream> outgoing_byte_stream)
  : outgoing_(std::move(outgoing_byte_stream))
  {
  }

protected:
  /**
   * @brief Sends the serialized message payload `bytes` over the outgoing byte stream, prepending
   * the SomeIp and Message headers.
   */
  void sendMessage(std::vector<uint8_t> bytes)
  {
    sequence_number_++;

    std::vector<uint8_t> out_bytes;
    out_bytes.reserve(sizeof(SomeIpHeader) + sizeof(MessageHeader) + bytes.size());

    auto some_ip_payload_size = static_cast<uint32_t>(bytes.size() + sizeof(MessageHeader) + 12);
    SomeIpHeader some_ip{
      0xAEFA,
      static_cast<uint16_t>(StreamId),
      some_ip_payload_size,
      0xFFF0,
      sequence_number_,
      2,
      1,
      1,
      0,
      0,
      0};

    serialize(out_bytes, some_ip);

    auto message_length = static_cast<uint32_t>(sizeof(MessageHeader) + bytes.size());
    MessageHeader message_header{1, 1, static_cast<uint8_t>(StreamId), 0, message_length, 0, 0};
    serialize(out_bytes, message_header);
    out_bytes.insert(out_bytes.end(), bytes.begin(), bytes.end());

    outgoing_->write(out_bytes);
  }

private:
  std::shared_ptr<WritableByteStream> outgoing_;
  uint16_t sequence_number_{};
};

}  // namespace nebula::drivers::connections
