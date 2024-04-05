#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include "nebula_common/util/performance_counter.hpp"


#include <chrono>
#include <cstdint>
#include <iostream>

namespace nebula
{
namespace util
{

using namespace std::chrono_literals;

class TopicDelay
{
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using duration = std::chrono::nanoseconds;

public:
  TopicDelay(std::string tag) : tag_(std::move(tag)), last_print_(clock::now()), delays_() {}

  void tick(builtin_interfaces::msg::Time & stamp)
  {
    auto now = clock::now();

    time_point t_msg = time_point(duration(static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL + stamp.nanosec));
    auto delay = now - t_msg;

    delays_.update(delay);

    if (now - last_print_ >= PRINT_INTERVAL) {
      auto del = delays_.reset();

      last_print_ = now;

      std::stringstream ss;

      ss << "{\"type\": \"topic_delay\", \"tag\": \"" << tag_
         << "\", \"del\": [";
      
      for (duration & dt : *del) {
        ss << dt.count() << ',';
      }

      ss <<  "null]}" << std::endl;

      std::cout << ss.str();
    }
  }

private:
  std::string tag_;

  time_point last_print_{time_point::min()};

  Counter delays_;

  const duration PRINT_INTERVAL = 1s;
};

}  // namespace util
}  // namespace nebula