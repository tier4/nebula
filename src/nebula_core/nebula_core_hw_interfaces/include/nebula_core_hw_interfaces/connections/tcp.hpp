// Copyright 2026 TIER IV, Inc.
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
#include <nebula_core_hw_interfaces/connections/socket_utils.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

/**
 * @brief A wrapper around a TCP socket (AF_INET, SOCK_STREAM).
 *        Supports both client and server functionality (though server part not fully shown here).
 */
class TcpSocket
{
  struct SocketConfig
  {
    int32_t polling_interval_ms{10};
    int32_t connect_timeout_ms{3000};
    size_t buffer_size{4096};
    Endpoint target{};
  };

  TcpSocket(SockFd sock_fd, SocketConfig config) : sock_fd_(std::move(sock_fd)), config_{config} {}

public:
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

      init_socket();
    }

    explicit Builder(const Endpoint & target)
    {
      config_.target = target;
      init_socket();
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
     * @brief Set the timeout duration for the initial TCP connect attempt.
     *
     * @param timeout_ms The desired connection timeout in milliseconds.
     */
    Builder && set_connect_timeout(int32_t timeout_ms)
    {
      config_.connect_timeout_ms = timeout_ms;
      return std::move(*this);
    }

    /**
     * @brief Set the internal buffer size for receiving data.
     *
     * @param bytes The desired buffer size in bytes.
     */
    Builder && set_buffer_size(size_t bytes)
    {
      config_.buffer_size = bytes;
      return std::move(*this);
    }

    /**
     * @brief Connect the socket to the target IP and port.
     */
    TcpSocket connect() &&
    {
      sockaddr_in addr = config_.target.to_sockaddr();

      // Set socket to non-blocking
      int flags = fcntl(sock_fd_.get(), F_GETFL, 0);
      if (flags == -1) throw SocketError(errno);
      if (fcntl(sock_fd_.get(), F_SETFL, flags | O_NONBLOCK) == -1) throw SocketError(errno);

      int result{-1};
      do {
        result = ::connect(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      } while (result == -1 && errno == EINTR);

      if (result == -1) {
        if (errno != EINPROGRESS) {
          throw SocketError(errno);
        }

        // Connection is in progress, poll for completion
        pollfd pfd{sock_fd_.get(), POLLOUT, 0};
        int poll_result = poll(&pfd, 1, config_.connect_timeout_ms);

        if (poll_result == -1) {
          throw SocketError(errno);
        }
        if (poll_result == 0) {
          throw SocketError("Connection timeout");
        }

        // Check socket error status
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (getsockopt(sock_fd_.get(), SOL_SOCKET, SO_ERROR, &so_error, &len) == -1) {
          throw SocketError(errno);
        }
        if (so_error != 0) {
          throw SocketError(so_error);
        }
      }

      // Restore blocking mode
      if (fcntl(sock_fd_.get(), F_SETFL, flags) == -1) throw SocketError(errno);

      return TcpSocket{std::move(sock_fd_), config_};
    }

  private:
    void init_socket()
    {
      int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (sock_fd == -1) throw SocketError(errno);
      sock_fd_ = SockFd{sock_fd};

      // Enable TCP_NODELAY to reduce latency
      int one = 1;
      sock_fd_.setsockopt(IPPROTO_TCP, TCP_NODELAY, one).value_or_throw();
    }

    SockFd sock_fd_;
    SocketConfig config_;
  };

  /**
   * @brief Send data to the connected target.
   *        Handles partial sends by retrying until all data is transmitted.
   *
   * When this function throws, the socket may be in an undefined state and has to be closed and
   * re-created by the caller before the socket can be used again.
   *
   * @param data The data to send
   * @throws SocketError if send fails
   */
  void send(const std::vector<uint8_t> & data)
  {
    size_t total_sent = 0;
    while (total_sent < data.size()) {
      ssize_t result{-1};
      do {
        result =
          ::send(sock_fd_.get(), data.data() + total_sent, data.size() - total_sent, MSG_NOSIGNAL);
      } while (result == -1 && errno == EINTR);

      if (result == -1) throw SocketError(errno);
      total_sent += static_cast<size_t>(result);
    }
  }

  /**
   * @brief Receive up to n bytes from the socket.
   *        This is a blocking call with an optional timeout.
   *
   * When this function throws, the socket may be in an undefined state and has to be closed and
   * re-created by the caller before the socket can be used again.
   *
   * @param n The maximum number of bytes to receive.
   * @param timeout The timeout duration. If 0, blocks indefinitely.
   * @return A vector containing the received bytes. Empty if timeout.
   *         May contain fewer than n bytes if less data is available.
   * @throws SocketError if receive fails or connection is closed.
   */
  std::vector<uint8_t> receive(
    size_t n, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
  {
    return receive_impl(n, timeout);
  }

  /**
   * @brief Receive up to the configured internal buffer size.
   *        This is a blocking call with an optional timeout.
   *
   * @param timeout The timeout duration. If 0, blocks indefinitely.
   * @return A vector containing the received bytes. Empty if timeout.
   * @throws SocketError if receive fails or connection is closed.
   */
  std::vector<uint8_t> receive(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
  {
    return receive_impl(config_.buffer_size, timeout);
  }

  TcpSocket(const TcpSocket &) = delete;
  TcpSocket(TcpSocket && other) noexcept
  : sock_fd_(std::move(other.sock_fd_)), config_(other.config_)
  {
  }

  TcpSocket & operator=(const TcpSocket &) = delete;
  TcpSocket & operator=(TcpSocket &&) = delete;

private:
  std::vector<uint8_t> receive_impl(size_t n, std::chrono::milliseconds timeout)
  {
    if (n == 0) throw UsageError("Receive size must be greater than zero");

    if (timeout.count() > 0) {
      auto ready = is_socket_ready(sock_fd_.get(), static_cast<int>(timeout.count()));
      if (!ready.has_value()) throw SocketError(ready.error());
      if (!ready.value()) return {};
    }

    std::vector<uint8_t> buffer(n);
    ssize_t result{-1};
    do {
      result = ::recv(sock_fd_.get(), buffer.data(), n, 0);
    } while (result == -1 && errno == EINTR);

    if (result < 0) throw SocketError(errno);
    if (result == 0) throw SocketError("Connection closed");
    buffer.resize(result);
    return buffer;
  }

  SockFd sock_fd_;
  SocketConfig config_;
};

}  // namespace nebula::drivers::connections
