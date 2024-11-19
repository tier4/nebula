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

#include <nebula_common/util/expected.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

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
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

class SocketError : public std::exception
{
public:
  explicit SocketError(int err_no) : what_(strerror(err_no)) {}

  explicit SocketError(const char * msg) : what_(msg) {}

  const char * what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

class UsageError : public std::runtime_error
{
public:
  explicit UsageError(const std::string & msg) : std::runtime_error(msg) {}
};

class UdpSocket
{
private:
  struct Endpoint
  {
    std::string ip;
    uint16_t port;
  };

  struct MsgBuffers
  {
    msghdr msg{};
    iovec iov{};
    std::array<std::byte, 1024> control;
    sockaddr_in sender_addr;
  };

  enum class State { UNINITIALIZED, INITIALIZED, BOUND, ACTIVE };

  static const int g_poll_timeout_ms = 10;

public:
  using callback_t = std::function<void(const std::vector<std::byte> &, uint64_t)>;

  /**
   * @brief Construct a UDP socket with timestamp measuring enabled. The minimal way to start
   * receiving on the socket is `UdpSocket().init(...).bind().subscribe(...);`.
   */
  UdpSocket()
  {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1) throw SocketError(errno);

    int enable = 1;
    int result = setsockopt(sock_fd, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable)) < 0;
    if (result < 0) throw SocketError(errno);

    int reuse = 1;
    result = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (result < 0) throw SocketError(errno);

    poll_fd_ = {sock_fd, POLLIN, 0};
    sock_fd_ = sock_fd;
  }

  /**
   * @brief Specify the host address and port for this socket to be bound to. To bind the socket,
   * call the `bind()` function.
   *
   * @param host_ip The address to bind to.
   * @param host_port The port to bind to.
   */
  UdpSocket & init(const std::string & host_ip, uint16_t host_port)
  {
    if (state_ > State::INITIALIZED) throw UsageError("Socket must be initialized before binding");

    host_ = {host_ip, host_port};
    state_ = State::INITIALIZED;
    return *this;
  }

  /**
   * @brief Set the socket to drop all packets not coming from `sender_ip` and `sender_port`.
   *
   * @param sender_ip The only allowed sender IP. Cannot be a multicast or broadcast address.
   * @param sender_port The only allowed sender port.
   */
  UdpSocket & limit_to_sender(const std::string & sender_ip, uint16_t sender_port)
  {
    if (state_ > State::INITIALIZED) throw UsageError("Buffer size has to be set before binding");

    sender_.emplace(Endpoint{sender_ip, sender_port});
    return *this;
  }

  /**
   * @brief Set the MTU this socket supports. While this can be set arbitrarily, it is best set to
   * the MTU of the network interface, or to the maximum expected packet length.
   *
   * @param bytes The MTU size. The default value is 1500.
   */
  UdpSocket & set_mtu(size_t bytes)
  {
    if (state_ > State::INITIALIZED) throw UsageError("Buffer size has to be set before binding");

    buffer_size_ = bytes;
    return *this;
  }

  /**
   * @brief Join an IP multicast group. Only one group can be joined by the socket.
   *
   * @param group_ip The multicast IP. It has to be in the multicast range `224.0.0.0/4` (between
   * `224.0.0.0` and `239.255.255.255`).
   */
  UdpSocket & join_multicast_group(const std::string & group_ip)
  {
    if (state_ < State::INITIALIZED) throw UsageError("Socket has to be initialized first");

    if (state_ >= State::BOUND)
      throw UsageError("Multicast groups have to be joined before binding");

    ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());  // Multicast group address
    mreq.imr_interface.s_addr = inet_addr(host_.ip.c_str());

    int result = setsockopt(sock_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (result < 0) throw SocketError(errno);

    multicast_ip_.emplace(group_ip);
    return *this;
  }

  /**
   * @brief Bind the socket to host IP and port given in `init()`. If `join_multicast_group()` was
   * called before this function, the socket will be bound to `group_ip` instead. At least `init()`
   * has to have been called before.
   */
  UdpSocket & bind()
  {
    if (state_ < State::INITIALIZED) throw UsageError("Socket has to be initialized first");

    if (state_ >= State::BOUND) throw UsageError("Re-binding already bound socket");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(host_.port);
    addr.sin_addr.s_addr = multicast_ip_ ? inet_addr(multicast_ip_->c_str()) : INADDR_ANY;

    int result = ::bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (result == -1) throw SocketError(errno);
    return *this;
  }

  /**
   * @brief Register a callback for processing received packets and start the receiver thread. The
   * callback will be called for each received packet, and will be executed in the receive thread.
   * Has to be called on a bound socket (`bind()` has to have been called before).
   *
   * @param callback The function to be executed for each received packet.
   */
  UdpSocket & subscribe(callback_t && callback)
  {
    if (state_ < State::BOUND) throw UsageError("Socket has to be bound first");

    if (state_ > State::BOUND) throw UsageError("Cannot re-subscribe to socket");

    callback_ = std::move(callback);
    launch_receiver();
    return *this;
  }

  ~UdpSocket()
  {
    if (state_ == State::ACTIVE) state_ = State::BOUND;
    if (receive_thread_.joinable()) receive_thread_.join();
    close(sock_fd_);
  }

private:
  void launch_receiver()
  {
    assert(state_ == State::BOUND);
    assert(callback_);

    state_ = State::ACTIVE;
    receive_thread_ = std::thread([this]() {
      std::vector<std::byte> buffer;
      while (state_ == State::ACTIVE) {
        auto data_available = is_data_available();
        if (!data_available.has_value()) throw SocketError(data_available.error());
        if (!data_available.value()) continue;

        buffer.resize(buffer_size_);
        auto msg_header = make_msg_header(buffer);

        ssize_t received = recvmsg(sock_fd_, &msg_header.msg, 0);
        if (received < 0) throw SocketError(errno);
        if (!is_accepted_sender(msg_header.sender_addr)) continue;

        auto timestamp_ns_opt = get_receive_timestamp(msg_header.msg);
        if (!timestamp_ns_opt) continue;

        uint64_t timestamp_ns = timestamp_ns_opt.value();

        buffer.resize(received);
        callback_(buffer, timestamp_ns);
      }
    });
  }

  std::optional<uint64_t> get_receive_timestamp(msghdr & msg)
  {
    timeval const * tv = nullptr;
    for (cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
        tv = (struct timeval *)CMSG_DATA(cmsg);
        break;
      }
    }

    if (!tv) {
      return {};
    }

    uint64_t timestamp_ns = tv->tv_sec * 1'000'000'000 + tv->tv_usec * 1000;
    return timestamp_ns;
  }

  util::expected<bool, int> is_data_available()
  {
    int status = poll(&poll_fd_, 1, g_poll_timeout_ms);
    if (status < 0) return errno;
    return (poll_fd_.revents & POLLIN) && (status > 0);
  }

  bool is_accepted_sender(const sockaddr_in & sender_addr)
  {
    if (!sender_) return true;

    std::array<char, INET_ADDRSTRLEN> sender_name;
    inet_ntop(AF_INET, &sender_addr.sin_addr, sender_name.data(), INET_ADDRSTRLEN);
    return std::strncmp(sender_->ip.c_str(), sender_name.data(), INET_ADDRSTRLEN) == 0;
  }

  MsgBuffers make_msg_header(std::vector<std::byte> & receive_buffer) const
  {
    msghdr msg{};
    iovec iov{};
    std::array<std::byte, 1024> control;

    sockaddr_in sender_addr;
    socklen_t sender_addr_len = sizeof(sender_addr);

    iov.iov_base = receive_buffer.data();
    iov.iov_len = receive_buffer.size();

    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data();
    msg.msg_controllen = control.size();
    msg.msg_name = &sender_addr;
    msg.msg_namelen = sender_addr_len;

    return MsgBuffers{msg, iov, control, sender_addr};
  }

  std::atomic<State> state_{State::UNINITIALIZED};

  int sock_fd_;
  pollfd poll_fd_;

  size_t buffer_size_ = 1500;
  Endpoint host_;
  std::optional<std::string> multicast_ip_;
  std::optional<Endpoint> sender_;
  std::thread receive_thread_;
  callback_t callback_;
};

}  // namespace nebula::drivers::connections
