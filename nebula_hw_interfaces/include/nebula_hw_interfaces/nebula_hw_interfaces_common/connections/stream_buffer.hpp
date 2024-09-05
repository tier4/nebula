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
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp"

#include <nebula_common/util/mt_queue.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

/**
 * @brief A `PullableByteStream` that buffers incoming bytes from a given `ObservableByteStream`
 * until they are read by the user.
 */
class StreamBuffer : public PullableByteStream
{
  using packet_loss_callback_t = std::function<void()>;

public:
  StreamBuffer(
    std::shared_ptr<ObservableByteStream> underlying, size_t packets_buffered,
    packet_loss_callback_t packet_loss_callback)
  : underlying_(std::move(underlying)),
    buffer_(packets_buffered),
    packet_loss_callback_(std::move(packet_loss_callback))
  {
    underlying_->registerBytesCallback(
      [&](auto bytes) { onBytesFromUnderlying(std::move(bytes)); });
  }

  void read(std::vector<uint8_t> & into, size_t n_bytes) override
  {
    into.clear();

    // If there is a packet left from the last read that has not been read completely read
    // (remainder_), then continue there. Otherwise, pop the next one off the queue.
    auto bytes = remainder_ ? std::move(remainder_) : buffer_.pop();

    // Sizes match perfectly, return popped vector
    if (bytes->size() == n_bytes) {
      into.swap(*bytes);
      return;
    }

    // Too many bytes popped off the queue, put remainder back
    if (bytes->size() > n_bytes) {
      // Cut the remaining bytes off of bytes (will become remainder_ later). `bytes` then contains
      // exactly the `n_bytes` we want as the into, and `into` contains exactly the bytes we
      // want to become the `remainder_`. Thus, we can just swap the two vectors.
      into.reserve(bytes->size() - n_bytes);
      into.insert(into.end(), bytes->end() - static_cast<ssize_t>(n_bytes), bytes->end());
      bytes->resize(bytes->size() - n_bytes);

      into.swap(*bytes);
      remainder_ = std::move(bytes);
      return;
    }

    // Too little bytes popped off the queue, fetch next part
    if (bytes->size() < n_bytes) {
      auto remaining_length = n_bytes - bytes->size();
      std::vector<uint8_t> remaining_bytes;
      read(remaining_bytes, remaining_length);
      into.swap(*bytes);
      into.reserve(n_bytes);
      into.insert(into.end(), remaining_bytes.begin(), remaining_bytes.end());
      return;
    }

    assert(false);  // Unreachable
  }

private:
  void onBytesFromUnderlying(std::vector<uint8_t> bytes)
  {
    auto ptr = std::make_unique<std::vector<uint8_t>>();
    ptr->swap(bytes);
    auto success = buffer_.tryPush(std::move(ptr));
    if (!success) {
      packet_loss_callback_();
    }
  }

  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;
  std::shared_ptr<ObservableByteStream> underlying_;
  std::unique_ptr<std::vector<uint8_t>> remainder_;
  std::mutex mtx_buffer_;
  MtQueue<std::unique_ptr<std::vector<uint8_t>>> buffer_;

  packet_loss_callback_t packet_loss_callback_;
};

}  // namespace nebula::drivers::connections
