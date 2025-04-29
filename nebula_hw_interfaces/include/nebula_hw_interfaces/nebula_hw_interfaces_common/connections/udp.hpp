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

#ifndef _GNU_SOURCE
// See `man strerror_r`
#define _GNU_SOURCE
#endif

#include <nebula_common/util/expected.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace nebula::drivers::connections
{

class SocketError : public std::exception
{
  static constexpr size_t gnu_max_strerror_length = 1024;

public:
  explicit SocketError(int err_no)
  {
    std::array<char, gnu_max_strerror_length> msg_buf;
    std::string_view msg = strerror_r(err_no, msg_buf.data(), msg_buf.size());
    what_ = std::string{msg};
  }

  explicit SocketError(const std::string_view & msg) : what_(msg) {}

  const char * what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

class UsageError : public std::runtime_error
{
public:
  explicit UsageError(const std::string & msg) : std::runtime_error(msg) {}
};

inline util::expected<in_addr, UsageError> parse_ip(const std::string & ip)
{
  in_addr parsed_addr{};
  bool valid = inet_aton(ip.c_str(), &parsed_addr);
  if (!valid) return UsageError("Invalid IP address given");
  return parsed_addr;
}

class UdpSocket
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
      close(sock_fd_);
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

    size_t buffer_size{1500};
    Endpoint host;
    std::optional<in_addr> multicast_ip;
    std::optional<Endpoint> sender_filter;
    std::optional<Endpoint> send_to;
  };

  struct MsgBuffers
  {
    explicit MsgBuffers(std::vector<uint8_t> & receive_buffer)
    {
      iov.iov_base = receive_buffer.data();
      iov.iov_len = receive_buffer.size();

      msg.msg_iov = &iov;
      msg.msg_iovlen = 1;
      msg.msg_control = control.data();
      msg.msg_controllen = control.size();
      msg.msg_name = &sender_addr;
      msg.msg_namelen = sizeof(sender_addr);
    }

    iovec iov{};
    std::array<std::byte, 1024> control{};
    sockaddr_in sender_addr{};
    msghdr msg{};
  };

  class DropMonitor
  {
    uint32_t last_drop_counter_{0};

  public:
    uint32_t get_drops_since_last_receive(uint32_t current_drop_counter)
    {
      uint32_t last = last_drop_counter_;
      last_drop_counter_ = current_drop_counter;

      bool counter_did_wrap = current_drop_counter < last;
      if (counter_did_wrap) {
        return (UINT32_MAX - last) + current_drop_counter;
      }

      return current_drop_counter - last;
    }
  };

  UdpSocket(SockFd sock_fd, SocketConfig config)
  : sock_fd_(std::move(sock_fd)), poll_fd_{sock_fd_.get(), POLLIN, 0}, config_{std::move(config)}
  {
  }

public:
  class Builder
  {
  public:
    /**
     * @brief Build a UDP socket with timestamp measuring enabled. The minimal way to start
     * receiving on the socket is `UdpSocket::Builder(...).bind().subscribe(...);`.
     *
     * @param host_ip The address to bind to.
     * @param host_port The port to bind to.
     */
    Builder(const std::string & host_ip, uint16_t host_port)
    {
      in_addr host_in_addr = parse_ip(host_ip).value_or_throw();
      if (host_in_addr.s_addr == INADDR_BROADCAST)
        throw UsageError("Do not bind to broadcast IP. Bind to 0.0.0.0 or a specific IP instead.");

      config_.host = {host_in_addr, host_port};

      int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
      if (sock_fd == -1) throw SocketError(errno);
      sock_fd_ = SockFd{sock_fd};

      sock_fd_.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1).value_or_throw();

      // Enable kernel-space receive time measurement
      sock_fd_.setsockopt(SOL_SOCKET, SO_TIMESTAMP, 1).value_or_throw();

      // Enable reporting on packets dropped due to full UDP receive buffer
      sock_fd_.setsockopt(SOL_SOCKET, SO_RXQ_OVFL, 1).value_or_throw();
    }

    /**
     * @brief Set the socket to drop all packets not coming from `sender_ip` and `sender_port`.
     *
     * @param sender_ip The only allowed sender IP. Cannot be a multicast or broadcast address.
     * @param sender_port The only allowed sender port.
     */
    Builder && limit_to_sender(const std::string & sender_ip, uint16_t sender_port)
    {
      config_.sender_filter.emplace(Endpoint{parse_ip(sender_ip).value_or_throw(), sender_port});
      return std::move(*this);
    }

    /**
     * @brief Set the destination to send packets to.
     *
     * @param dest_ip The destination IP address.
     * @param dest_port The destination port.
     */
    Builder && set_send_destination(const std::string & dest_ip, uint16_t dest_port)
    {
      config_.send_to.emplace(Endpoint{parse_ip(dest_ip).value_or_throw(), dest_port});
      return std::move(*this);
    }

    /**
     * @brief Set the MTU this socket supports. While this can be set arbitrarily, it is best set to
     * the MTU of the network interface, or to the maximum expected packet length.
     *
     * @param bytes The MTU size. The default value is 1500.
     */
    Builder && set_mtu(size_t bytes)
    {
      config_.buffer_size = bytes;
      return std::move(*this);
    }

    /**
     * @brief Set the internal socket receive buffer size. See `SO_RCVBUF` in `man 7 socket` for
     * more information.
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
     * @brief Join an IP multicast group. Only one group can be joined by the socket.
     *
     * @param group_ip The multicast IP. It has to be in the multicast range `224.0.0.0/4` (between
     * `224.0.0.0` and `239.255.255.255`).
     */
    Builder && join_multicast_group(const std::string & group_ip)
    {
      if (config_.multicast_ip)
        throw UsageError("Only one multicast group can be joined by this socket");
      ip_mreq mreq{parse_ip(group_ip).value_or_throw(), config_.host.ip};

      sock_fd_.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, mreq).value_or_throw();
      config_.multicast_ip.emplace(mreq.imr_multiaddr);
      return std::move(*this);
    }

    /**
     * @brief Set the interval at which the socket polls for new data. This should be longer than
     * the expected interval of packets arriving in order to not poll unnecessarily often, and
     * should be shorter than the acceptable time delay for `unsubscribe()`. The `unsubscribe()`
     * function blocks up to one full poll interval before returning.
     *
     * @param interval_ms The desired polling interval. See `man poll` for the meanings of 0 and
     * negative values.
     */
    Builder && set_polling_interval(int32_t interval_ms)
    {
      config_.polling_interval_ms = interval_ms;
      return std::move(*this);
    }

    /**
     * @brief Bind the socket to host IP and port given in `init()`. If `join_multicast_group()` was
     * called before this function, the socket will be bound to `group_ip` instead. At least
     * `init()` has to have been called before.
     */
    UdpSocket bind() &&
    {
      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(config_.host.port);
      addr.sin_addr = config_.multicast_ip ? *config_.multicast_ip : config_.host.ip;

      int result = ::bind(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
      if (result == -1) throw SocketError(errno);

      return UdpSocket{std::move(sock_fd_), config_};
    }

  private:
    SockFd sock_fd_;
    SocketConfig config_;
  };

  struct RxMetadata
  {
    std::optional<uint64_t> timestamp_ns;
    uint64_t drops_since_last_receive{0};
    bool truncated;
  };

  using callback_t = std::function<void(const std::vector<uint8_t> &, const RxMetadata &)>;

  /**
   * @brief Register a callback for processing received packets and start the receiver thread. The
   * callback will be called for each received packet, and will be executed in the receive thread.
   * Has to be called on a bound socket (`bind()` has to have been called before).
   *
   * @param callback The function to be executed for each received packet.
   */
  UdpSocket & subscribe(callback_t && callback)
  {
    unsubscribe();
    callback_ = std::move(callback);
    launch_receiver();
    return *this;
  }

  bool is_subscribed() { return running_; }

  /**
   * @brief Gracefully stops the active receiver thread (if any) but keeps the socket alive. The
   * same socket can later be subscribed again.
   */
  UdpSocket & unsubscribe()
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    return *this;
  }

  /**
   * @brief Send a datagram to the destination set in `set_send_destination()`.
   *
   * @param data The data to send
   * @throw UsageError If no destination has been set via `set_send_destination()`
   * @throw SocketError If the send operation fails
   */
  void send(const std::vector<uint8_t> & data)
  {
    if (!config_.send_to) throw UsageError("No destination set");

    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(config_.send_to->port);
    dest_addr.sin_addr = config_.send_to->ip;

    ssize_t result = sendto(
      sock_fd_.get(), data.data(), data.size(), 0, (sockaddr *)&dest_addr, sizeof(dest_addr));

    if (result == -1) throw SocketError(errno);
  }

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket(UdpSocket && other)
  : sock_fd_((other.unsubscribe(), std::move(other.sock_fd_))),
    poll_fd_(other.poll_fd_),
    config_(other.config_),
    drop_monitor_(other.drop_monitor_)
  {
    if (other.callback_) subscribe(std::move(other.callback_));
  };

  UdpSocket & operator=(const UdpSocket &) = delete;
  UdpSocket & operator=(UdpSocket &&) = delete;

  ~UdpSocket() { unsubscribe(); }

private:
  void launch_receiver()
  {
    assert(callback_);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      std::vector<uint8_t> buffer;
      while (running_) {
        auto data_available = is_data_available();
        if (!data_available.has_value()) throw SocketError(data_available.error());
        if (!data_available.value()) continue;

        buffer.resize(config_.buffer_size);
        MsgBuffers msg_header{buffer};

        // As per `man recvmsg`, zero-length datagrams are permitted and valid. Since the socket is
        // blocking, a recv_result of 0 means we received a valid 0-length datagram.
        ssize_t recv_result = recvmsg(sock_fd_.get(), &msg_header.msg, MSG_TRUNC);
        if (recv_result < 0) throw SocketError(errno);
        size_t untruncated_packet_length = recv_result;

        if (!is_accepted_sender(msg_header.sender_addr)) continue;

        RxMetadata metadata;
        get_receive_metadata(msg_header.msg, metadata);
        metadata.truncated = untruncated_packet_length > config_.buffer_size;

        buffer.resize(std::min(config_.buffer_size, untruncated_packet_length));
        callback_(buffer, metadata);
      }
    });
  }

  void get_receive_metadata(msghdr & msg, RxMetadata & inout_metadata)
  {
    for (cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level != SOL_SOCKET) continue;

      switch (cmsg->cmsg_type) {
        case SO_TIMESTAMP: {
          auto tv = (timeval const *)CMSG_DATA(cmsg);
          uint64_t timestamp_ns = tv->tv_sec * 1'000'000'000 + tv->tv_usec * 1000;
          inout_metadata.timestamp_ns.emplace(timestamp_ns);
          break;
        }
        case SO_RXQ_OVFL: {
          auto drops = (uint32_t const *)CMSG_DATA(cmsg);
          inout_metadata.drops_since_last_receive =
            drop_monitor_.get_drops_since_last_receive(*drops);
          break;
        }
        default:
          continue;
      }
    }
  }

  util::expected<bool, int> is_data_available()
  {
    int status = poll(&poll_fd_, 1, config_.polling_interval_ms);
    if (status == -1) return errno;
    return (poll_fd_.revents & POLLIN) && (status > 0);
  }

  bool is_accepted_sender(const sockaddr_in & sender_addr)
  {
    if (!config_.sender_filter) return true;
    return sender_addr.sin_addr.s_addr == config_.sender_filter->ip.s_addr;
  }

  SockFd sock_fd_;
  pollfd poll_fd_;

  SocketConfig config_;

  std::atomic_bool running_{false};
  std::thread receive_thread_;
  callback_t callback_;

  DropMonitor drop_monitor_;
};

}  // namespace nebula::drivers::connections
