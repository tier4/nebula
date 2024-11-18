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

#include "nebula_common/util/expected.hpp"

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace nebula::drivers::connections
{

/**
 * @brief A non-owning view onto the bytes contained in an underlying vector.
 * Bytes can be consumed from the view, automatically shrinking it by that amount.
 */
class ByteView
{
public:
  class Slice;

private:
  using iter_t = std::vector<uint8_t>::const_iterator;
  using consume_result_t = nebula::util::expected<Slice, std::out_of_range>;

public:
  explicit ByteView(const std::vector<uint8_t> & underlying)
  : cbegin_(underlying.cbegin()), cend_(underlying.cend())
  {
  }

  explicit ByteView(std::vector<uint8_t> &&) = delete;
  explicit ByteView(const std::vector<uint8_t> &&) = delete;

  /**
   * @brief Consumes `n_bytes` bytes from the view's beginning, shrinking it by that amount and
   * returning a slice making those bytes accessible. This function throws in case of out-of-bounds
   * accesses.
   *
   * @param n_bytes The number of bytes to consume
   * @return Slice The consumed bytes
   */
  Slice consume_unsafe(size_t n_bytes)
  {
    auto n = static_cast<int64_t>(n_bytes);
    if (n > size()) {
      throw std::out_of_range("Index out of bounds");
    }

    auto new_cbegin = std::next(cbegin_, n);
    auto result = Slice(cbegin_, new_cbegin);
    cbegin_ = new_cbegin;

    return result;
  }

  /**
   * @brief Tries to consume `n_bytes`, returns a slice in case of success, or returns an exception
   * in case of failure. Does not throw.
   *
   * @param n_bytes The number of bytes to consume
   * @return consume_result_t Either the consumed bytes as a slice, or a caught exception
   */
  [[nodiscard]] consume_result_t consume(size_t n_bytes)
  {
    try {
      return consume_unsafe(n_bytes);
    } catch (const std::out_of_range & e) {
      return e;
    }
  }

  [[nodiscard]] ssize_t size() const { return std::distance(cbegin_, cend_); }

  /**
   * @brief A non-owning slice of an underlying view. This slice's bytes can be iterated over and it
   * cannot be split/consumed further.
   */
  class Slice
  {
    friend ByteView;

  public:
    [[nodiscard]] auto begin() const { return cbegin_; }
    [[nodiscard]] auto cbegin() const { return cbegin_; }

    [[nodiscard]] auto end() const { return cend_; }
    [[nodiscard]] auto cend() const { return cend_; }

    [[nodiscard]] ssize_t size() const { return std::distance(cbegin_, cend_); }

  private:
    Slice(iter_t cbegin, iter_t cend) : cbegin_(cbegin), cend_(cend) {}

    iter_t cbegin_;
    iter_t cend_;
  };

private:
  iter_t cbegin_;
  iter_t cend_;
};

}  // namespace nebula::drivers::connections
