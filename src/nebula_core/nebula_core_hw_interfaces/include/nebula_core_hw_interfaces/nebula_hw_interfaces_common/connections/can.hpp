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

#ifndef _GNU_SOURCE
// See `man strerror_r`
#define _GNU_SOURCE
#endif

#include <nebula_core_common/util/errno.hpp>
#include <nebula_core_common/util/expected.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class CanSocketError : public std::exception
{
public:
  explicit CanSocketError(int err_no) : what_{util::errno_to_string(err_no)} {}

  explicit CanSocketError(const std::string_view & msg) : what_(msg) {}

  const char * what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

class CanUsageError : public std::runtime_error
{
public:
  explicit CanUsageError(const std::string & msg) : std::runtime_error(msg) {}
};

class CanSocket
{
  class SockFd
  {
    static const int uninitialized = -1;
    int sock_fd_;

  public:
    SockFd() : sock_fd_{uninitialized} {}
    explicit SockFd(int sock_fd) : sock_fd_{sock_fd} {}
    SockFd(SockFd && other) noexcept : sock_fd_{other.sock_fd_} { other.sock_fd_ = uninitialized; }

    SockFd(const SockFd &) = delete;
    SockFd & operator=(const SockFd &) = delete;
    SockFd & operator=(SockFd && other) noexcept
    {
      std::swap(sock_fd_, other.sock_fd_);
      return *this;
    };

    ~SockFd()
    {
      if (sock_fd_ == uninitialized) return;
      close(sock_fd_);
    }

    [[nodiscard]] int get() const { return sock_fd_; }

    template <typename T>
    [[nodiscard]] util::expected<std::monostate, CanSocketError> setsockopt(
      int level, int optname, const T & optval)
    {
      int result = ::setsockopt(sock_fd_, level, optname, &optval, sizeof(T));
      if (result == -1) return CanSocketError(errno);
      return std::monostate{};
    }
  };

  struct SocketConfig
  {
    int32_t polling_interval_ms{10};
    std::string interface_name;
  };

  CanSocket(SockFd sock_fd, SocketConfig config)
  : sock_fd_(std::move(sock_fd)), poll_fd_{sock_fd_.get(), POLLIN, 0}, config_{std::move(config)}
  {
  }

public:
  class Builder
  {
  public:
    /**
     * @brief Build a CAN socket.
     *
     * @param interface_name The name of the CAN interface (e.g., "can0", "vcan0").
     */
    explicit Builder(const std::string & interface_name)
    {
      config_.interface_name = interface_name;

      int sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (sock_fd == -1) throw CanSocketError(errno);
      sock_fd_ = SockFd{sock_fd};
    }

    /**
     * @brief Set the interval at which the socket polls for new data.
     *
     * @param interval_ms The desired polling interval.
     */
    Builder && set_polling_interval(int32_t interval_ms)
    {
      config_.polling_interval_ms = interval_ms;
      return std::move(*this);
    }

    /**
     * @brief Bind the socket to the CAN interface.
     */
    CanSocket bind() &&
    {
      ifreq ifr{};
      std::strncpy(ifr.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1);
      if (ioctl(sock_fd_.get(), SIOCGIFINDEX, &ifr) == -1) throw CanSocketError(errno);

      sockaddr_can addr{};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      int result = ::bind(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      if (result == -1) throw CanSocketError(errno);

      return CanSocket{std::move(sock_fd_), config_};
    }

  private:
    SockFd sock_fd_;
    SocketConfig config_;
  };

  using callback_t = std::function<void(const can_frame & frame)>;

  /**
   * @brief Register a callback for processing received frames and start the receiver thread.
   *
   * @param callback The function to be executed for each received frame.
   */
  CanSocket & subscribe(callback_t && callback)
  {
    unsubscribe();
    callback_ = std::move(callback);
    launch_receiver();
    return *this;
  }

  bool is_subscribed() { return running_; }

  /**
   * @brief Gracefully stops the active receiver thread.
   */
  CanSocket & unsubscribe()
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    return *this;
  }

  /**
   * @brief Send a CAN frame.
   *
   * @param frame The CAN frame to send.
   */
  void send(const can_frame & frame)
  {
    ssize_t result = ::write(sock_fd_.get(), &frame, sizeof(frame));
    if (result == -1) throw CanSocketError(errno);
    if (result != sizeof(frame)) throw CanSocketError("Incomplete CAN frame write");
  }

  CanSocket(const CanSocket &) = delete;
  CanSocket(CanSocket && other)
  : sock_fd_((other.unsubscribe(), std::move(other.sock_fd_))),
    poll_fd_(other.poll_fd_),
    config_(other.config_)
  {
    if (other.callback_) subscribe(std::move(other.callback_));
  };

  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket & operator=(CanSocket &&) = delete;

  ~CanSocket() { unsubscribe(); }

private:
  void launch_receiver()
  {
    assert(callback_);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      while (running_) {
        auto data_available = is_data_available();
        if (!data_available.has_value()) throw CanSocketError(data_available.error());
        if (!data_available.value()) continue;

        can_frame frame{};
        ssize_t recv_result = ::read(sock_fd_.get(), &frame, sizeof(frame));
        if (recv_result < 0) throw CanSocketError(errno);
        if (recv_result < static_cast<ssize_t>(sizeof(frame)))
          continue;  // Ignore incomplete frames

        callback_(frame);
      }
    });
  }

  util::expected<bool, int> is_data_available()
  {
    int status = poll(&poll_fd_, 1, config_.polling_interval_ms);
    if (status == -1) return errno;
    return (poll_fd_.revents & POLLIN) && (status > 0);
  }

  SockFd sock_fd_;
  pollfd poll_fd_;

  SocketConfig config_;

  std::atomic_bool running_{false};
  std::thread receive_thread_;
  callback_t callback_;
};

}  // namespace nebula::drivers::connections
