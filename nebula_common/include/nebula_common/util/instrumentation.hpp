#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>

namespace nebula
{
namespace util
{

using namespace std::chrono_literals;

class Instrumentation
{
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using duration = std::chrono::nanoseconds;

public:
  Instrumentation(std::string tag) : tag(tag), last_print(clock::now()) {}

  void tick()
  {
    auto now = clock::now();

    if (last_tick != time_point::min()) {
      auto rtt = now - last_tick;
      rtts_.update(rtt);
    }

    last_tick = now;
  }

  void tock()
  {
    auto now = clock::now();
    auto delay = now - last_tick;
    delays_.update(delay);

    if (now - last_print >= PRINT_INTERVAL) {
      auto del = delays_.reset();
      auto rtt = rtts_.reset();

      last_print = now;

      std::cout << "{\"tag\": \"" << tag << "\", \"del_min\": " << del.min.count() * 1e-9
                << ", \"del_avg\": " << del.avg.count() * 1e-9
                << ", \"del_max\": " << del.max.count() * 1e-9
                << ", \"rtt_min\": " << rtt.min.count() * 1e-9
                << ", \"rtt_avg\": " << rtt.avg.count() * 1e-9
                << ", \"rtt_max\": " << rtt.max.count() * 1e-9 << ", \"window\": " << rtt.count
                << '}' << std::endl;
    }
  }

private:
  class Counter
  {
  public:
    struct Stats
    {
      duration min;
      duration avg;
      duration max;
      uint64_t count;
    };

    Stats reset()
    {
      auto avg_measurement = sum_measurements / num_measurements;

      Stats result{min_measurement, avg_measurement, max_measurement, num_measurements};

      sum_measurements = duration::zero();
      num_measurements = 0;
      min_measurement = duration::zero();
      max_measurement = duration::max();
    }

    void update(duration measurement)
    {
      sum_measurements += measurement;
      num_measurements++;

      min_measurement = std::min(min_measurement, measurement);
      max_measurement = std::max(max_measurement, measurement);
    }

  private:
    duration sum_measurements{duration::zero()};
    uint64_t num_measurements{0};
    duration min_measurement{duration::max()};
    duration max_measurement{duration::zero()};
  };

  std::string tag;

  time_point last_tick{time_point::min()};
  time_point last_print{time_point::min()};

  Counter delays_;
  Counter rtts_;

  const duration PRINT_INTERVAL = 1s;
};

}  // namespace util
}  // namespace nebula