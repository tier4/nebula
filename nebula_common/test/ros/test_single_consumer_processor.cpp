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

#include "nebula/common/ros/single_consumer_processor.hpp"

#include <rcpputils/thread_safety_annotations.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#define EXPECT_VEC_EQ(a, ...)     \
  {                               \
    auto _a = (a);                \
    decltype(_a) _b{__VA_ARGS__}; \
    EXPECT_EQ(_a, _b);            \
  }

using nebula::ros::SingleConsumerProcessor;

class SingleConsumerProcessorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    processed_items_.clear();
    callback_count_.store(0);
  }

  // Helper function to create a callback that records processed items
  auto make_recording_callback()
  {
    return [this](int item) {
      std::lock_guard lock(processed_items_mutex_);
      processed_items_.push_back(item);
      callback_count_.fetch_add(1);
      processed_cv_.notify_all();
    };
  }

  // Helper function to wait for a specific number of items to be processed
  bool wait_for_processed_count(
    int expected_count, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    std::unique_lock lock(processed_items_mutex_);
    return processed_cv_.wait_for(
      lock, timeout, [&] { return callback_count_.load() >= expected_count; });
  }

  // Helper function to get processed items safely
  std::vector<int> get_processed_items()
  {
    std::lock_guard lock(processed_items_mutex_);
    return processed_items_;
  }

  // Helper function to get callback count
  [[nodiscard]] int get_callback_count() const { return callback_count_.load(); }

private:
  std::vector<int> processed_items_ RCPPUTILS_TSA_GUARDED_BY(processed_items_mutex_);
  std::mutex processed_items_mutex_;
  std::condition_variable processed_cv_;
  std::atomic<int> callback_count_{0};
};

TEST_F(SingleConsumerProcessorTest, ConstructorValidation)
{
  // Test with null callback
  EXPECT_THROW(SingleConsumerProcessor<int>(nullptr, 1), std::invalid_argument);

  // Test with zero max queue size
  EXPECT_THROW(SingleConsumerProcessor<int>([](int /*unused*/) {}, 0), std::invalid_argument);

  // Test with valid parameters
  EXPECT_NO_THROW(SingleConsumerProcessor<int>([](int /*unused*/) {}, 1));
}

TEST_F(SingleConsumerProcessorTest, BasicPushProcessing)
{
  constexpr size_t queue_size = 3;
  SingleConsumerProcessor<int> processor(make_recording_callback(), queue_size);

  // Push some items
  processor.push(1);
  processor.push(2);
  processor.push(3);

  // Wait for processing
  ASSERT_TRUE(wait_for_processed_count(3));

  // Verify all items were processed
  EXPECT_VEC_EQ(get_processed_items(), 1, 2, 3);
}

TEST_F(SingleConsumerProcessorTest, TryPushSuccessWithSpace)
{
  SingleConsumerProcessor<int> processor(make_recording_callback(), 3);

  // try_push should succeed when there's space
  EXPECT_TRUE(processor.try_push(1));
  EXPECT_TRUE(processor.try_push(2));
  EXPECT_TRUE(processor.try_push(3));

  // Wait for processing
  ASSERT_TRUE(wait_for_processed_count(3));

  // Verify all items were processed
  EXPECT_VEC_EQ(get_processed_items(), 1, 2, 3);
}

TEST_F(SingleConsumerProcessorTest, TryPushFailsWhenQueueFull)
{
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::atomic<int> callback_count{0};
  bool callback_can_proceed = false;

  // Create a processor with a controlled callback and queue size of 1
  auto controlled_callback = [&](int /*unused*/) {
    std::unique_lock lock(callback_mutex);
    callback_count.fetch_add(1);
    callback_cv.notify_all();

    // Wait until we're allowed to proceed
    callback_cv.wait(lock, [&] { return callback_can_proceed; });
  };

  // Use queue size 1 to make the test more predictable
  SingleConsumerProcessor<int> processor(controlled_callback, 1);

  // Fill the queue with one item
  EXPECT_TRUE(processor.try_push(1));

  // Wait for the consumer to start processing the first item
  {
    std::unique_lock lock(callback_mutex);
    callback_cv.wait(lock, [&] { return callback_count.load() > 0; });
  }

  // Queue is now empty but callback is busy, so try_push should succeed
  EXPECT_TRUE(processor.try_push(2));

  // Now try_push should fail because queue is full and consumer is busy
  EXPECT_FALSE(processor.try_push(3));

  // Allow callbacks to proceed
  {
    std::lock_guard lock(callback_mutex);
    callback_can_proceed = true;
  }
  callback_cv.notify_all();
}

TEST_F(SingleConsumerProcessorTest, StopProcessesRemainingItems)
{
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::vector<int> processed_order;
  bool callback_can_proceed = false;

  auto controlled_callback = [&](int item) {
    std::unique_lock lock(callback_mutex);

    // For the first item, wait for permission to proceed
    if (item == 1) {
      callback_cv.wait(lock, [&] { return callback_can_proceed; });
    }

    processed_order.push_back(item);
    callback_cv.notify_all();
  };

  constexpr size_t queue_size = 10;
  SingleConsumerProcessor<int> processor(controlled_callback, queue_size);

  // Push multiple items
  constexpr int max_item = 5;
  for (int i = 1; i <= max_item; ++i) {
    processor.push(static_cast<int &&>(i));
  }

  // Call stop() while items are still in queue (first callback is blocked)
  std::thread stop_thread([&]() { processor.stop(); });

  // Allow the callbacks to proceed
  {
    std::lock_guard lock(callback_mutex);
    callback_can_proceed = true;
  }
  callback_cv.notify_all();

  // Wait for stop to complete
  stop_thread.join();

  // Verify all items were processed
  std::lock_guard lock(callback_mutex);
  EXPECT_VEC_EQ(processed_order, 1, 2, 3, 4, 5);
}

TEST_F(SingleConsumerProcessorTest, PushBlocksWhenQueueFull)
{
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::atomic<int> callback_count{0};
  bool callback_can_proceed = false;

  auto controlled_callback = [&](int /*unused*/) {
    std::unique_lock lock(callback_mutex);
    callback_count.fetch_add(1);
    callback_cv.notify_all();

    // Wait until we're allowed to proceed
    callback_cv.wait(lock, [&] { return callback_can_proceed; });
  };

  // Use queue size 1 to make behavior more predictable
  SingleConsumerProcessor<int> processor(controlled_callback, 1);

  // Fill the queue with one item
  processor.push(1);

  // Wait for consumer to start processing the first item
  {
    std::unique_lock lock(callback_mutex);
    callback_cv.wait(lock, [&] { return callback_count.load() > 0; });
  }

  // Fill the queue again while callback is busy
  processor.push(2);

  // This push should block because queue is full and consumer is busy
  std::atomic<bool> push_completed{false};

  std::thread push_thread([&]() {
    processor.push(3);
    push_completed.store(true);
  });

  // Verify push is blocked by checking it hasn't completed yet
  // Since we control the callback, it shouldn't complete until we allow it
  EXPECT_FALSE(push_completed.load());

  // Allow callbacks to proceed
  {
    std::lock_guard lock(callback_mutex);
    callback_can_proceed = true;
  }
  callback_cv.notify_all();

  // Wait for push to complete
  push_thread.join();

  // Verify push completed after callbacks were allowed to proceed
  EXPECT_TRUE(push_completed.load());
}

TEST_F(SingleConsumerProcessorTest, StopPreventsNewSubmissions)
{
  constexpr size_t queue_size = 2;
  SingleConsumerProcessor<int> processor(make_recording_callback(), queue_size);

  // Stop the processor
  processor.stop();

  // Verify processor is in stopping state
  EXPECT_TRUE(processor.is_stopping());

  // push should return immediately without processing
  processor.push(1);

  // try_push should return false
  EXPECT_FALSE(processor.try_push(2));

  // Verify no items were processed
  EXPECT_EQ(get_callback_count(), 0);
}

TEST_F(SingleConsumerProcessorTest, MultipleStopCalls)
{
  constexpr size_t queue_size = 2;
  SingleConsumerProcessor<int> processor(make_recording_callback(), queue_size);

  processor.push(1);
  processor.push(2);

  // Multiple stop calls should be safe
  processor.stop();
  processor.stop();
  processor.stop();

  // Should still have processed the items that were submitted before stop
  ASSERT_TRUE(wait_for_processed_count(2));
  EXPECT_VEC_EQ(get_processed_items(), 1, 2);
}
