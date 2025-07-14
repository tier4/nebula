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

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

namespace nebula::ros
{

/// @brief A thread-safe single-consumer queue processor that runs a callback function
/// on items in a separate thread. The queue has a maximum size, so insertions block or
/// are rejected if the consumer is slower than the producer.
/// @tparam T The type of items to process
template <typename T>
class SingleConsumerProcessor
{
public:
  using callback_t = std::function<void(T &&)>;

  /// @brief Constructor
  /// @param callback The callback function to execute on each item
  /// @param max_queue_size The maximum size of the queue
  explicit SingleConsumerProcessor(callback_t callback, size_t max_queue_size)
  : callback_(std::move(callback)), max_queue_size_(max_queue_size)
  {
    if (max_queue_size == 0) {
      throw std::invalid_argument("Max queue size must be greater than 0");
    }

    if (!callback_) {
      throw std::invalid_argument("Callback function must not be null");
    }

    consumer_thread_ = std::thread(&SingleConsumerProcessor::consumer_loop, this);
  }

  /// @brief Destructor - stops the processing thread
  ~SingleConsumerProcessor() { stop(); }

  // Delete copy and move constructors/operators to prevent accidental copying
  SingleConsumerProcessor(const SingleConsumerProcessor &) = delete;
  SingleConsumerProcessor & operator=(const SingleConsumerProcessor &) = delete;
  SingleConsumerProcessor(SingleConsumerProcessor &&) = default;
  SingleConsumerProcessor & operator=(SingleConsumerProcessor &&) = default;

  /// @brief Add an item to the queue for processing. Blocks until the item has been queued.
  /// @param item The item to process
  void push(T && item)
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    cv_can_push_.wait(lock, [this]() { return should_stop_ || (queue_.size() < max_queue_size_); });

    if (should_stop_) {
      return;
    }

    queue_.push(std::move(item));
    lock.unlock();
    cv_can_pop_.notify_one();
  }

  /// @brief Try to add an item to the queue for processing without blocking.
  /// @param item The item to process
  /// @return True if the item was added, false if the queue is full
  bool try_push(T && item)
  {
    std::unique_lock lock(queue_mutex_);

    if (should_stop_) {
      return false;
    }

    if (queue_.size() >= max_queue_size_) {
      return false;
    }

    queue_.push(std::move(item));
    lock.unlock();
    cv_can_pop_.notify_one();
    return true;
  }

  /// @brief Stop the consumer thread
  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      should_stop_ = true;
    }

    cv_can_pop_.notify_all();
    cv_can_push_.notify_all();

    if (consumer_thread_.joinable()) {
      consumer_thread_.join();
    }
  }

private:
  void consumer_loop()
  {
    std::unique_lock lock(queue_mutex_, std::defer_lock);

    while (true) {
      lock.lock();
      // Wait for an item to be available or for stop signal
      cv_can_pop_.wait(lock, [this] { return !queue_.empty() || should_stop_; });

      if (should_stop_) {
        break;
      }

      T item = std::move(queue_.front());
      queue_.pop();
      lock.unlock();
      cv_can_push_.notify_one();
      callback_(std::move(item));
    }

    assert(lock.owns_lock());

    // Process the remaining items when stopping.
    // No need to unlock as the consumer
    while (!queue_.empty()) {
      T item = std::move(queue_.front());
      queue_.pop();
      callback_(std::move(item));
    }
  }

  callback_t callback_;
  std::thread consumer_thread_;

  std::mutex queue_mutex_;
  std::condition_variable cv_can_pop_;
  std::condition_variable cv_can_push_;

  std::queue<T> queue_{};
  size_t max_queue_size_;
  bool should_stop_{false};
};

}  // namespace nebula::ros
