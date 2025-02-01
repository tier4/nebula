// Copyright 2025 TIER IV, Inc.
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

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace nebula::util
{

template <typename T>
class RingBuffer
{
public:
  explicit RingBuffer(std::size_t capacity) : capacity_(capacity) { buffer_.resize(capacity_); }

  void push_back(const T & value)
  {
    if (is_full()) {
      sum_ -= buffer_[index_];
    }

    sum_ += value;
    buffer_[index_] = value;
    index_ = (index_ + 1) % capacity_;
    size_ = std::min(size_ + 1, capacity_);
  }

  T & operator[](std::size_t index)
  {
    if (index >= size_) {
      throw std::out_of_range("Index out of range");
    }
    return buffer_[(index_ + index) % size_];
  }

  const T & operator[](std::size_t index) const
  {
    if (index >= size_) {
      throw std::out_of_range("Index out of range");
    }
    return buffer_[(index_ + index) % size_];
  }

  std::size_t size() const { return size_; }

  bool is_full() const { return size_ == capacity_; }

  T get_average() const { return sum_ / size_; }

private:
  T sum_{};
  std::vector<T> buffer_;
  std::size_t capacity_;
  std::size_t size_{0};
  std::size_t index_{0};
};

}  // namespace nebula::util
