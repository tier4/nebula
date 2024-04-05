#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

namespace nebula
{
namespace util
{

class Counter
{
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using duration = std::chrono::nanoseconds;

public:
  Counter()
  : buf_record_(), buf_output_()
  {
    buf_record_.resize(100000);
    buf_output_.resize(100000);
  }

  std::vector<duration> * reset()
  {
    buf_record_.swap(buf_output_);
    buf_record_.clear();
    return &buf_output_;
  }

  void update(duration measurement)
  {
    buf_record_.push_back(measurement);
  }

private:
  std::vector<duration> buf_record_;
  std::vector<duration> buf_output_;
};
}  // namespace util
}  // namespace nebula