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

#include <arpa/inet.h>
#include <netinet/in.h>
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

class TcpSocket
{
  struct Endpoint
  {
    in_addr ip;
    /// In host byte order.
    uint16_t port;
  };

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
    size_t buffer_size{4096};
    Endpoint target;
  };

  TcpSocket(SockFd sock_fd, SocketConfig config)
  : sock_fd_(std::move(sock_fd)), poll_fd_{sock_fd_.get(), POLLIN, 0}, config_{std::move(config)}
  {
  }

public:
  TcpSocket() : sock_fd_(), poll_fd_{-1, POLLIN, 0} {}

  void open(const std::string & ip, uint16_t port)
  {
    if (sock_fd_.get() != -1) {
      sock_fd_ = SockFd();
    }

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) throw SocketError(errno);
    sock_fd_ = SockFd(fd);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    in_addr target_in_addr = parse_ip(ip).value_or_throw();
    addr.sin_addr = target_in_addr;

    int result = ::connect(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
    if (result == -1) throw SocketError(errno);

    config_.target = {target_in_addr, port};
    poll_fd_.fd = sock_fd_.get();
  }

  bool isOpen() const { return sock_fd_.get() != -1; }

  void close() { sock_fd_ = SockFd(); }

  std::vector<uint8_t> receive(size_t n)
  {
    std::vector<uint8_t> buffer(n);
    ssize_t result = ::recv(sock_fd_.get(), buffer.data(), n, 0);
    if (result < 0) throw SocketError(errno);
    if (result == 0) return {};
    buffer.resize(result);
    return buffer;
  }
  class Builder
  {
  public:
    /**
     * @brief Build a TCP socket.
     *
     * @param target_ip The address to connect to.
     * @param target_port The port to connect to.
     */
    Builder(const std::string & target_ip, uint16_t target_port)
    {
      in_addr target_in_addr = parse_ip(target_ip).value_or_throw();
      config_.target = {target_in_addr, target_port};

      int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (sock_fd == -1) throw SocketError(errno);
      sock_fd_ = SockFd{sock_fd};
    }

    /**
     * @brief Set the internal socket receive buffer size.
     *
     * @param bytes The desired buffer size in bytes.
     */
    Builder && set_socket_buffer_size(size_t bytes)
    {
      if (bytes > static_cast<size_t>(INT32_MAX))
        throw UsageError("The maximum value supported (0x7FFFFFF) has been exceeded");

      auto buf_size = static_cast<int>(bytes);
      sock_fd_.setsockopt(SOL_SOCKET, SO_RCVBUF, buf_size).value_or_throw();
      return std::move(*this);
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
     * @brief Connect the socket to the target IP and port.
     */
    TcpSocket connect() &&
    {
      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(config_.target.port);
      addr.sin_addr = config_.target.ip;

      int result = ::connect(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      if (result == -1) throw SocketError(errno);

      return TcpSocket{std::move(sock_fd_), config_};
    }

  private:
    SockFd sock_fd_;
    SocketConfig config_;
  };

  using callback_t = std::function<void(const std::vector<uint8_t> & data)>;

  /**
   * @brief Register a callback for processing received data and start the receiver thread.
   *
   * @param callback The function to be executed for received data.
   */
  TcpSocket & subscribe(callback_t && callback)
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
  TcpSocket & unsubscribe()
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    return *this;
  }

  /**
   * @brief Send data to the connected target.
   *
   * @param data The data to send
   */
  void send(const std::vector<uint8_t> & data)
  {
    ssize_t result = ::send(sock_fd_.get(), data.data(), data.size(), 0);
    if (result == -1) throw SocketError(errno);
  }

  TcpSocket(const TcpSocket &) = delete;
  TcpSocket(TcpSocket && other)
  : sock_fd_((other.unsubscribe(), std::move(other.sock_fd_))),
    poll_fd_(other.poll_fd_),
    config_(other.config_)
  {
    if (other.callback_) subscribe(std::move(other.callback_));
  };

  TcpSocket & operator=(const TcpSocket &) = delete;
  TcpSocket & operator=(TcpSocket &&) = delete;

  ~TcpSocket() { unsubscribe(); }

private:
  void launch_receiver()
  {
    assert(callback_);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      std::vector<uint8_t> buffer;
      buffer.resize(config_.buffer_size);

      while (running_) {
        auto data_available = is_data_available();
        if (!data_available.has_value()) throw SocketError(data_available.error());
        if (!data_available.value()) continue;

        ssize_t recv_result = ::recv(sock_fd_.get(), buffer.data(), buffer.size(), 0);
        if (recv_result < 0) throw SocketError(errno);
        if (recv_result == 0) {
          // Connection closed by peer
          running_ = false;
          return;
        }

        std::vector<uint8_t> received_data(buffer.begin(), buffer.begin() + recv_result);
        callback_(received_data);
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
