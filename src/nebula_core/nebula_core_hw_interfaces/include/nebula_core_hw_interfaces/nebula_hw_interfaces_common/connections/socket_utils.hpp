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

#include <nebula_core_common/util/errno.hpp>
#include <nebula_core_common/util/expected.hpp>

#include <arpa/inet.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace nebula::drivers::connections
{

/**
 * @brief Exception thrown when a socket operation fails (usually check errno).
 */
class SocketError : public std::exception
{
public:
  explicit SocketError(int err_no) : what_{util::errno_to_string(err_no)} {}

  explicit SocketError(const std::string_view & msg) : what_(msg) {}

  const char * what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

/**
 * @brief Exception thrown when the API is used incorrectly.
 */
class UsageError : public std::runtime_error
{
public:
  explicit UsageError(const std::string & msg) : std::runtime_error(msg) {}
};

/**
 * @brief RAII wrapper for a file descriptor, ensuring it is closed when destroyed.
 *        Non-copyable, move-only.
 */
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

  /**
   * @brief Wrapper for setsockopt.
   *
   * @tparam T Type of the option value.
   * @param level Protocol level.
   * @param optname Option name.
   * @param optval The option value.
   * @return Success (std::monostate) or SocketError.
   */
  template <typename T>
  [[nodiscard]] util::expected<std::monostate, SocketError> setsockopt(
    int level, int optname, const T & optval)
  {
    int result = ::setsockopt(sock_fd_, level, optname, &optval, sizeof(T));
    if (result == -1) return SocketError(errno);
    return std::monostate{};
  }
};

/**
 * @brief Represents an IP (v4) and port endpoint.
 */
struct Endpoint
{
  in_addr ip;
  /// In host byte order.
  uint16_t port;
};

/**
 * @brief Parse a string IP address (dotted quad) into an in_addr.
 *
 * @param ip IP address string.
 * @return in_addr on success, UsageError on failure.
 */
inline util::expected<in_addr, UsageError> parse_ip(const std::string & ip)
{
  in_addr parsed_addr{};
  bool valid = inet_aton(ip.c_str(), &parsed_addr);
  if (!valid) return UsageError("Invalid IP address given");
  return parsed_addr;
}

/**
 * @brief Convert an in_addr to its string representation.
 *
 * @param addr The address to convert.
 * @return String representation (e.g., "192.168.1.1").
 */
inline std::string to_string(const in_addr & addr)
{
  char buf[INET_ADDRSTRLEN];
  if (inet_ntop(AF_INET, &addr, buf, INET_ADDRSTRLEN) == nullptr) {
    return "0.0.0.0";
  }
  return std::string(buf);
}

/**
 * @brief Check if a file descriptor has data ready to read (using poll).
 *
 * @param fd The file descriptor.
 * @param timeout_ms Timeout in milliseconds.
 * @param events Events to check for (default POLLIN).
 * @return True if data is ready, False if timeout. Error code (int) on failure.
 */
inline util::expected<bool, int> is_socket_ready(int fd, int timeout_ms, int events = POLLIN)
{
  pollfd pfd{fd, static_cast<std::int16_t>(events), 0};
  int status = poll(&pfd, 1, timeout_ms);
  if (status == -1) return errno;
  return (pfd.revents & events) && (status > 0);
}

}  // namespace nebula::drivers::connections
