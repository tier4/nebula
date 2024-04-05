#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

template <typename T>
class mt_queue
{
private:
  std::mutex d_mutex;
  std::condition_variable d_condition;
  std::deque<T> d_queue;

  size_t capacity_;

public:
  mt_queue(size_t capacity) : capacity_(capacity) {}

  bool push(T && value)
  {
    {
      std::unique_lock<std::mutex> lock(this->d_mutex);
      if (d_queue.size() == capacity_) {
        return false;
      }

      d_queue.push_front(std::move(value));
    }
    this->d_condition.notify_one();
    return true;
  }
  T pop()
  {
    std::unique_lock<std::mutex> lock(this->d_mutex);
    this->d_condition.wait(lock, [=] { return !this->d_queue.empty(); });
    T rc(std::move(this->d_queue.back()));
    this->d_queue.pop_back();
    return rc;
  }
};