#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <chrono>

template <typename T>
class mt_queue
{
private:
  std::mutex mutex_;
  std::condition_variable cv_not_empty_, cv_not_full_;
  std::deque<T> queue_;
  size_t capacity_;

public:
  mt_queue(size_t capacity) : capacity_(capacity) {}

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
    this->cv_not_full_.wait(lock, [=] { return this->queue_.size() < this->capacity_; });
    queue_.push_front(std::move(value));
    this->cv_not_empty_.notify_all();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->cv_not_empty_.wait(lock, [=] { return !this->queue_.empty(); });
    T return_value(std::move(this->queue_.back()));
    this->queue_.pop_back();
    this->cv_not_full_.notify_all();

    return return_value;
  }
};