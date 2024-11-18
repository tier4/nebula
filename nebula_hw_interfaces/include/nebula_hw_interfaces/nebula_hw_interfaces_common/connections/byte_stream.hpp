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

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace nebula::drivers::connections
{

/**
 * @brief A stream of packets containing bytes. The given callback is called on each arriving
 * packet.
 */
class ObservableByteStream
{
public:
  using callback_t = typename std::function<void(const std::vector<uint8_t> & buffer)>;

  ObservableByteStream() = default;

  ObservableByteStream(const ObservableByteStream &) = default;
  ObservableByteStream(ObservableByteStream &&) = default;
  ObservableByteStream & operator=(const ObservableByteStream &) = default;
  ObservableByteStream & operator=(ObservableByteStream &&) = default;

  virtual ~ObservableByteStream() = default;

  virtual void register_bytes_callback(callback_t callback) = 0;
};

/**
 * @brief A stream of bytes that the user can read N bytes from synchronously.
 */
class PullableByteStream
{
public:
  PullableByteStream() = default;

  PullableByteStream(const PullableByteStream &) = default;
  PullableByteStream(PullableByteStream &&) = default;
  PullableByteStream & operator=(const PullableByteStream &) = default;
  PullableByteStream & operator=(PullableByteStream &&) = default;

  virtual ~PullableByteStream() = default;

  virtual void read(std::vector<uint8_t> & into, size_t n_bytes) = 0;
};

/**
 * @brief A stream of bytes that can be written to by the user.
 */
class WritableByteStream
{
public:
  WritableByteStream() = default;

  WritableByteStream(const WritableByteStream &) = default;
  WritableByteStream(WritableByteStream &&) = default;
  WritableByteStream & operator=(const WritableByteStream &) = default;
  WritableByteStream & operator=(WritableByteStream &&) = default;

  virtual ~WritableByteStream() = default;

  virtual void write(std::vector<uint8_t> & data) = 0;
};

}  // namespace nebula::drivers::connections
