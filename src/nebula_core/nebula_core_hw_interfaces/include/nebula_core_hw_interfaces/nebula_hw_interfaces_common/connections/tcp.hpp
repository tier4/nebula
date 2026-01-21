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
#include <netinet/tcp.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
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
    size_t buffer_size{4096};
    Endpoint target;
  };

  TcpSocket(SockFd sock_fd, SocketConfig config)
  : sock_fd_(std::move(sock_fd)), config_{std::move(config)}
  {
  }

public:
  ~TcpSocket() { unsubscribe(); }

  /**
   * @brief Receive up to n bytes from the socket.
   *        This is a blocking call with an optional timeout.
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
    if (is_subscribed()) {
      throw UsageError("Cannot call receive() while subscribed");
    }

    if (timeout.count() > 0) {
      auto ready = is_socket_ready(sock_fd_.get(), timeout.count());
      if (!ready.has_value()) throw SocketError(ready.error());
      if (!ready.value()) return {};
    }

    std::vector<uint8_t> buffer(n);
    ssize_t result;
    do {
      result = ::recv(sock_fd_.get(), buffer.data(), n, 0);
    } while (result == -1 && errno == EINTR);

    if (result < 0) throw SocketError(errno);
    if (result == 0) throw SocketError("Connection closed");
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

      int result;
      do {
        result = ::connect(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      } while (result == -1 && errno == EINTR);

      if (result == -1) throw SocketError(errno);

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

  // Callback receives the buffer and the number of bytes received in it
  using callback_t =
    std::function<void(const std::vector<uint8_t> & buffer, size_t bytes_received)>;

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

  /**
   * @brief Check if the socket is currently subscribed and receiving data.
   * @return True if receiving, false otherwise.
   */
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
   *        Handles partial sends by retrying until all data is transmitted.
   *
   * @param data The data to send
   * @throws SocketError if send fails
   */
  void send(const std::vector<uint8_t> & data)
  {
    size_t total_sent = 0;
    while (total_sent < data.size()) {
      ssize_t result;
      do {
        result =
          ::send(sock_fd_.get(), data.data() + total_sent, data.size() - total_sent, MSG_NOSIGNAL);
      } while (result == -1 && errno == EINTR);

      if (result == -1) throw SocketError(errno);
      total_sent += static_cast<size_t>(result);
    }
  }

  TcpSocket(const TcpSocket &) = delete;
  TcpSocket(TcpSocket && other) : sock_fd_(), config_(other.config_)
  {
    other.unsubscribe();  // Stop the other thread
    // No need to start our own thread; remain unsubscribed until user calls subscribe()
    // We just take ownership of the FD and config.
    sock_fd_ = std::move(other.sock_fd_);
    // callback_ is not transferred to avoid implicit thread restart confusion
  };

  TcpSocket & operator=(const TcpSocket &) = delete;
  TcpSocket & operator=(TcpSocket &&) = delete;

private:
  void launch_receiver()
  {
    assert(callback_);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      std::vector<uint8_t> buffer;
      buffer.resize(config_.buffer_size);

      while (running_) {
        try {
          auto data_available = is_socket_ready(sock_fd_.get(), config_.polling_interval_ms);
          if (!data_available.has_value()) throw SocketError(data_available.error());
          if (!data_available.value()) continue;

          ssize_t recv_result;
          do {
            recv_result = ::recv(sock_fd_.get(), buffer.data(), buffer.size(), 0);
          } while (recv_result == -1 && errno == EINTR);

          if (recv_result < 0) throw SocketError(errno);
          if (recv_result == 0) {
            // Connection closed by peer
            running_ = false;
            return;
          }

          // Pass direct view of buffer
          callback_(buffer, static_cast<size_t>(recv_result));
        } catch (const SocketError & e) {
          // In a real application, we might want to log this properly or have an on_error callback.
          // For now, we print to stderr and stop the thread to prevent the application from
          // crashing.
          std::cerr << "TcpSocket receiver error: " << e.what() << std::endl;
          running_ = false;
        }
      }
    });
  }

  SockFd sock_fd_;

  SocketConfig config_;

  std::atomic_bool running_{false};
  std::thread receive_thread_;
  callback_t callback_;
};

}  // namespace nebula::drivers::connections
