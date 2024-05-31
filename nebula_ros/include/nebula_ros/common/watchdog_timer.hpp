#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>

namespace nebula
{
namespace ros
{

class WatchdogTimer
{
  using watchdog_cb_t = std::function<void(bool)>;

public:
  WatchdogTimer(
    rclcpp::Node & node, const std::chrono::microseconds & expected_update_interval,
    const watchdog_cb_t & callback)
  : node_(node),
    callback_(callback),
    expected_update_interval_ns_(
      std::chrono::duration_cast<std::chrono::nanoseconds>(expected_update_interval).count())
  {
    timer_ =
      node_.create_wall_timer(expected_update_interval, std::bind(&WatchdogTimer::onTimer, this));
  }

  void update() { last_update_ns_ = node_.get_clock()->now().nanoseconds(); }

private:
  void onTimer()
  {
    uint64_t now_ns = node_.get_clock()->now().nanoseconds();

    bool is_late = (last_update_ns_ > now_ns)
                     ? false
                     : (now_ns - last_update_ns_) > expected_update_interval_ns_;

    callback_(!is_late);
  }

  rclcpp::Node & node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> last_update_ns_;
  const watchdog_cb_t callback_;

  const uint64_t expected_update_interval_ns_;
};

}  // namespace ros
}  // namespace nebula
