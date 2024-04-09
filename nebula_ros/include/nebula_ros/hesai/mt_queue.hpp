#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

template <typename T>
class mt_queue
{
private:
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::deque<T> queue_;
  size_t capacity_;

public:
  mt_queue(size_t capacity) : capacity_(capacity) {}

  bool try_push(T && value)
  {
    {
      std::unique_lock<std::mutex> lock(this->mutex_);
      if (queue_.size() == capacity_) {
        return false;
      }

      queue_.push_front(std::move(value));
    }
    this->condition_variable_.notify_one();
    return true;
  }

  void push(T && value)
  {
    {
      std::unique_lock<std::mutex> lock(this->mutex_);
      this->condition_variable_.wait(lock, [=] { return this->queue_.size() < this->capacity_; });
      queue_.push_front(std::move(value));
    }
    this->condition_variable_.notify_one();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    this->condition_variable_.wait(lock, [=] { return !this->queue_.empty(); });
    T return_value(std::move(this->queue_.back()));
    this->queue_.pop_back();
    return return_value;
  }
};