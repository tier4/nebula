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

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

template <typename T>
class MtQueue
{
private:
  std::mutex mutex_;
  std::condition_variable cv_not_empty_, cv_not_full_;
  std::deque<T> queue_;
  size_t capacity_;

public:
  explicit MtQueue(size_t capacity) : capacity_(capacity) {}

  bool try_push(T && value)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    bool can_push = queue_.size() < capacity_;
    if (can_push) {
      queue_.push_front(std::move(value));
    }
    this->cv_not_empty_.notify_all();
    return can_push;
  }

  void push(T && value)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->cv_not_full_.wait(lock, [this] { return this->queue_.size() < this->capacity_; });
    queue_.push_front(std::move(value));
    this->cv_not_empty_.notify_all();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->cv_not_empty_.wait(lock, [this] { return !this->queue_.empty(); });
    T return_value(std::move(this->queue_.back()));
    this->queue_.pop_back();
    this->cv_not_full_.notify_all();

    return return_value;
  }
};
