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
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#define NEBULA_LOG_STREAM(log_func, stream_args) \
  {                                              \
    std::stringstream ss{};                      \
    ss << stream_args;                           \
    log_func(ss.str());                          \
  }

namespace nebula::drivers::loggers
{

class Logger
{
public:
  /// @brief A view of a logger that either forwards or drops messages.
  /// @details Returned by `Logger::throttled` and `Logger::once`. It converts to `true` if messages
  /// are forwarded, so expensive messages can be skipped entirely:
  /// ```c++
  /// if (auto log = logger->throttled(5s)) {
  ///   NEBULA_LOG_STREAM(log.warn, "Expensive to format: " << value);
  /// }
  /// ```
  /// The view must not outlive the logger it was created from.
  class Filtered
  {
  public:
    void debug(const std::string & message) const
    {
      if (target_) target_->debug(message);
    }
    void info(const std::string & message) const
    {
      if (target_) target_->info(message);
    }
    void warn(const std::string & message) const
    {
      if (target_) target_->warn(message);
    }
    void error(const std::string & message) const
    {
      if (target_) target_->error(message);
    }

    /// @brief Whether messages logged through this view are forwarded to the logger.
    explicit operator bool() const { return target_ != nullptr; }

  private:
    friend class Logger;
    explicit Filtered(Logger * target) : target_(target) {}

    Logger * target_;
  };

  virtual ~Logger() = default;

  virtual void debug(const std::string & message) = 0;
  virtual void info(const std::string & message) = 0;
  virtual void warn(const std::string & message) = 0;
  virtual void error(const std::string & message) = 0;

  virtual std::shared_ptr<Logger> child(const std::string & name) = 0;

  /// @brief Log at most once per `period` from the calling line of code.
  /// @details The first call passes. The state is kept per logger instance and per call site
  /// (file and line), so two objects with their own loggers are throttled independently. Do not
  /// pass `file` and `line`, they identify the call site.
  /// ```c++
  /// logger->throttled(5s).warn("Dropping malformed packet");
  /// ```
  /// @param period The minimum time between two messages that are let through
  /// @return A view that forwards messages if this call passed the throttle, and drops them if not
  Filtered throttled(
    std::chrono::nanoseconds period, const char * file = __builtin_FILE(),
    uint32_t line = __builtin_LINE())
  {
    const uint64_t period_ns = static_cast<uint64_t>(period.count());
    const uint64_t now_ns = steady_now_ns();

    std::lock_guard lock(filter_mutex_);
    auto [it, is_first_call] = throttle_last_passed_ns_.try_emplace(CallSite{file, line}, now_ns);
    if (is_first_call) {
      return Filtered(this);
    }

    auto & last_passed_ns = it->second;
    if (now_ns - last_passed_ns < period_ns) {
      return Filtered(nullptr);
    }

    last_passed_ns = now_ns;
    return Filtered(this);
  }

  /// @brief Log only the first time the calling line of code is reached.
  /// @details The state is kept per logger instance and per call site (file and line), so two
  /// objects with their own loggers each log once. Do not pass `file` and `line`, they identify
  /// the call site.
  /// ```c++
  /// logger->once().warn("Unsupported firmware, skipping object lists");
  /// ```
  /// @return A view that forwards messages on the first call, and drops them afterwards
  Filtered once(const char * file = __builtin_FILE(), uint32_t line = __builtin_LINE())
  {
    std::lock_guard lock(filter_mutex_);
    const bool is_first_call = once_passed_.try_emplace(CallSite{file, line}, true).second;
    return Filtered(is_first_call ? this : nullptr);
  }

protected:
  /// @brief The time used for throttling, in nanoseconds since an arbitrary, fixed point in time.
  virtual uint64_t steady_now_ns() const
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
  }

private:
  using CallSite = std::pair<std::string_view, uint32_t>;

  std::mutex filter_mutex_;
  std::map<CallSite, uint64_t> throttle_last_passed_ns_;
  std::map<CallSite, bool> once_passed_;
};

}  // namespace nebula::drivers::loggers
