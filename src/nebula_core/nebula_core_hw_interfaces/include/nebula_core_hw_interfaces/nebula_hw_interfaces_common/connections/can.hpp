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
#include <nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/socket_utils.hpp>

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
      ::close(sock_fd_);
    }

    [[nodiscard]] int get() const { return sock_fd_; }

    template <typename T>
    [[nodiscard]] util::expected<std::monostate, SocketError> setsockopt(
      int level, int optname, const T & optval)
    {
      int result = ::setsockopt(sock_fd_, level, optname, &optval, sizeof(T));
      if (result == -1) return SocketError(errno);
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
      if (sock_fd == -1) throw SocketError(errno);
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
      if (ioctl(sock_fd_.get(), SIOCGIFINDEX, &ifr) == -1) throw SocketError(errno);

      sockaddr_can addr{};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      int result = ::bind(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      if (result == -1) throw SocketError(errno);

      return CanSocket{std::move(sock_fd_), config_};
    }

  private:
    SockFd sock_fd_;
    SocketConfig config_;
  };

  struct RxMetadata
  {
    std::uint64_t timestamp_ns{0};
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

  void close()
  {
    unsubscribe();
    sock_fd_ = SockFd{};
  }

  void send(const can_frame & frame)
  {
    ssize_t result = ::write(sock_fd_.get(), &frame, sizeof(frame));
    if (result == -1) throw SocketError(errno);
    if (result != sizeof(frame)) throw SocketError("Incomplete CAN frame write");
  }

  void send_fd(const canfd_frame & frame)
  {
    ssize_t result = ::write(sock_fd_.get(), &frame, sizeof(frame));
    if (result == -1) throw SocketError(errno);
    if (result != sizeof(frame)) throw SocketError("Incomplete CAN FD frame write");
  }

  void set_fd_mode(bool enable)
  {
    int fd_mode = enable ? 1 : 0;
    auto result = sock_fd_.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, fd_mode);
    if (!result.has_value()) throw SocketError(result.error());
  }

  void set_filters(const std::vector<can_filter> & filters)
  {
    int result = ::setsockopt(
      sock_fd_.get(), SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      filters.size() * sizeof(can_filter));
    if (result == -1) throw SocketError(errno);
  }

  bool receive_fd(canfd_frame & frame, std::chrono::nanoseconds timeout)
  {
    RxMetadata metadata;
    return receive_fd(frame, timeout, metadata);
  }

  bool receive_fd(canfd_frame & frame, std::chrono::nanoseconds timeout, RxMetadata & metadata)
  {
    pollfd pfd{sock_fd_.get(), POLLIN, 0};
    int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
    int status = poll(&pfd, 1, timeout_ms);
    if (status == -1) throw SocketError(errno);
    if (status == 0) return false;

    // To get timestamps, we should use recvmsg, but for now let's just use system time
    // if we want to keep it simple, or implement recvmsg here.
    // For CAN, SO_TIMESTAMP also works.

    struct iovec iov;
    struct msghdr msg;
    char ctrl[CMSG_SPACE(sizeof(struct timeval))];
    struct sockaddr_can addr;

    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);

    msg.msg_name = &addr;
    msg.msg_namelen = sizeof(addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof(ctrl);
    msg.msg_flags = 0;

    ssize_t recv_result = recvmsg(sock_fd_.get(), &msg, 0);
    if (recv_result < 0) throw SocketError(errno);

    metadata.timestamp_ns = 0;
    for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
         cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
        struct timeval * tv = (struct timeval *)CMSG_DATA(cmsg);
        metadata.timestamp_ns =
          (uint64_t)tv->tv_sec * 1000000000ULL + (uint64_t)tv->tv_usec * 1000ULL;
      }
    }

    if (metadata.timestamp_ns == 0) {
      metadata.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
    }

    if (static_cast<size_t>(recv_result) < sizeof(frame)) {
      throw SocketError("Incomplete CAN FD frame received");
    }
    return true;
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
        if (!data_available.has_value()) throw SocketError(data_available.error());
        if (!data_available.value()) continue;

        can_frame frame{};
        ssize_t recv_result = ::read(sock_fd_.get(), &frame, sizeof(frame));
        if (recv_result < 0) throw SocketError(errno);
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
