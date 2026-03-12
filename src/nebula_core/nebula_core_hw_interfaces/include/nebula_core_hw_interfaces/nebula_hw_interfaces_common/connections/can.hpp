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

// cspell: ignore TIMESTAMPNS

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
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

/**
 * @brief A wrapper around a raw CAN socket (AF_CAN).
 *        Supports both standard CAN and CAN FD frames.
 */
class CanSocket
{
  struct SocketConfig
  {
    int32_t polling_interval_ms{10};
    std::string interface_name;
  };

  CanSocket(SockFd sock_fd, SocketConfig config)
  : sock_fd_(std::move(sock_fd)), config_{std::move(config)}
  {
  }

public:
  ~CanSocket() { unsubscribe(); }
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

      if (interface_name.rfind("mock_fd:", 0) == 0) {
        int fd = std::stoi(interface_name.substr(8));
        int new_fd = dup(fd);
        if (new_fd == -1) throw SocketError(errno);
        sock_fd_ = SockFd{new_fd};
      } else {
        int sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_fd == -1) throw SocketError(errno);
        sock_fd_ = SockFd{sock_fd};
      }
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
      if (config_.interface_name.rfind("mock_fd:", 0) != 0) {
        ifreq ifr{};
        std::strncpy(ifr.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1);
        if (ioctl(sock_fd_.get(), SIOCGIFINDEX, &ifr) == -1) throw SocketError(errno);

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        int result = ::bind(sock_fd_.get(), (sockaddr *)&addr, sizeof(addr));
        if (result == -1) throw SocketError(errno);
      }

      return CanSocket{std::move(sock_fd_), config_};
    }

  private:
    SockFd sock_fd_;
    SocketConfig config_;
  };

  struct RxMetadata
  {
    std::optional<uint64_t> timestamp_ns;
  };

  using callback_t = std::function<void(const canfd_frame & frame, const RxMetadata & metadata)>;

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

  /**
   * @brief Check if the socket is currently subscribed and receiving data.
   * @return True if receiving, false otherwise.
   */
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
   * @brief Close the socket and return the file descriptor to uninitialized state.
   */

  /**
   * @brief Send a standard CAN frame.
   *
   * @param frame The CAN frame to send.
   * @throws SocketError if the write fails or is incomplete.
   */
  void send(const can_frame & frame)
  {
    ssize_t result = ::write(sock_fd_.get(), &frame, sizeof(frame));
    if (result == -1) throw SocketError(errno);
    if (result != sizeof(frame)) throw SocketError("Incomplete CAN frame write");
  }

  /**
   * @brief Send a CAN FD frame.
   *
   * @param frame The CAN FD frame to send.
   * @throws SocketError if the write fails or is incomplete.
   */
  void send_fd(const canfd_frame & frame)
  {
    ssize_t result = ::write(sock_fd_.get(), &frame, sizeof(frame));
    if (result == -1) throw SocketError(errno);
    if (result != sizeof(frame)) throw SocketError("Incomplete CAN FD frame write");
  }

  /**
   * @brief Enable or disable CAN FD frame support on the socket.
   *
   * @param enable True to enable, false to disable.
   * @throws SocketError if setsockopt fails.
   */
  void set_fd_mode(bool enable)
  {
    if (config_.interface_name.rfind("mock_fd:", 0) == 0) return;
    int fd_mode = enable ? 1 : 0;
    auto result = sock_fd_.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, fd_mode);
    if (!result.has_value()) throw SocketError(result.error());
  }

  /**
   * @brief Enable or disable kernel socket timestamping (SO_TIMESTAMP).
   *
   * @param enable True to enable, false to disable.
   * @throws SocketError if setsockopt fails.
   */
  void set_timestamping(bool enable)
  {
    int timestamping = enable ? 1 : 0;
    auto result = sock_fd_.setsockopt(SOL_SOCKET, SO_TIMESTAMPNS, timestamping);
    if (!result.has_value() && config_.interface_name.rfind("mock_fd:", 0) != 0) {
      throw SocketError(result.error());
    }
  }

  /**
   * @brief Set CAN raw filters.
   *
   * @param filters Vector of can_filter structures.
   * @throws SocketError if setsockopt fails.
   */
  void set_filters(const std::vector<can_filter> & filters)
  {
    if (config_.interface_name.rfind("mock_fd:", 0) == 0) return;
    int result = ::setsockopt(
      sock_fd_.get(), SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      filters.size() * sizeof(can_filter));
    if (result == -1) throw SocketError(errno);
  }

  /**
   * @brief Receive a CAN FD frame with a timeout.
   *
   * @param frame Reference to a canfd_frame to store the received data.
   * @param timeout Timeout duration.
   * @return True if a frame was received, false if timeout occurred.
   * @throws SocketError if poll or read fails.
   */
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

    size_t recv_result = receive_frame_with_metadata(&frame, sizeof(frame), metadata);

    if (recv_result < sizeof(frame.can_id) + sizeof(frame.len)) {
      throw SocketError("Corrupted CAN frame received");
    }

    if (frame.len > CANFD_MAX_DLEN) {
      throw SocketError("Frame length is larger than max allowed CAN FD payload length");
    }

    const auto data_length = static_cast<size_t>(frame.len);
    // some CAN FD frames are shorter than 64 bytes
    const auto expected_length = sizeof(frame) - sizeof(frame.data) + data_length;

    if (recv_result < expected_length) {
      throw SocketError("Incomplete CAN FD frame received");
    }
    return true;
  }

  CanSocket(const CanSocket &) = delete;
  CanSocket(CanSocket && other) : sock_fd_(), config_(other.config_)
  {
    other.unsubscribe();
    callback_t cb = std::move(other.callback_);
    sock_fd_ = std::move(other.sock_fd_);
    if (cb) subscribe(std::move(cb));
  };

  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket & operator=(CanSocket &&) = delete;

private:
  /**
   * @brief Receive a frame using recvmsg and extract timestamp from ancillary data.
   *
   * @param frame_ptr Pointer to the frame buffer (can_frame or canfd_frame).
   * @param frame_size Size of the frame buffer.
   * @param metadata Output metadata including timestamp.
   * @return Number of bytes received.
   * @throws SocketError if recvmsg fails.
   */
  size_t receive_frame_with_metadata(void * frame_ptr, size_t frame_size, RxMetadata & metadata)
  {
    struct iovec iov;
    struct msghdr msg;
    char ctrl[CMSG_SPACE(sizeof(struct timespec))];
    struct sockaddr_can addr;

    iov.iov_base = frame_ptr;
    iov.iov_len = frame_size;

    msg.msg_name = &addr;
    msg.msg_namelen = sizeof(addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof(ctrl);
    msg.msg_flags = 0;

    ssize_t recv_result{};
    do {
      recv_result = recvmsg(sock_fd_.get(), &msg, 0);
    } while (recv_result == -1 && errno == EINTR);

    if (recv_result < 0) throw SocketError(errno);

    metadata.timestamp_ns = extract_timestamp(msg);
    return static_cast<size_t>(recv_result);
  }

  /**
   * @brief Extract SO_TIMESTAMPNS from ancillary data.
   *
   * @param msg The msghdr containing ancillary data.
   * @return Timestamp in nanoseconds, or std::nullopt if not available.
   */
  static std::optional<uint64_t> extract_timestamp(const msghdr & msg)
  {
    for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
         cmsg = CMSG_NXTHDR(const_cast<msghdr *>(&msg), cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPNS) {
        struct timespec * ts = reinterpret_cast<struct timespec *>(CMSG_DATA(cmsg));
        return static_cast<uint64_t>(ts->tv_sec) * 1000000000ULL +
               static_cast<uint64_t>(ts->tv_nsec);
      }
    }
    return std::nullopt;
  }

  void launch_receiver()
  {
    assert(callback_);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      while (running_) {
        try {
          // Check for data with timeout
          auto data_available = is_socket_ready(sock_fd_.get(), config_.polling_interval_ms);
          if (!data_available.has_value()) throw SocketError(data_available.error());
          if (!data_available.value()) continue;

          // Buffer large enough for CAN FD
          union {
            can_frame cc;
            canfd_frame cf;
          } frame_buf{};

          RxMetadata metadata{};
          ssize_t nbytes = receive_frame_with_metadata(&frame_buf, sizeof(frame_buf), metadata);

          canfd_frame fd_frame{};
          if (nbytes == CAN_MTU) {
            // Convert standard frame to FD frame format for uniform callback
            fd_frame.can_id = frame_buf.cc.can_id;
            fd_frame.len = frame_buf.cc.can_dlc;  // For standard, len is dlc
            // Note: standard can_frame doesn't rely on strict 0-8 mapping for len/dlc in same way
            // as FD, but standard allows values > 8 (though invalid). Just copy.
            // Actually, can_dlc is uint8_t 0-8. Copy data.
            std::memcpy(fd_frame.data, frame_buf.cc.data, frame_buf.cc.can_dlc);
            fd_frame.flags = 0;  // Not FD
          } else if (nbytes == CANFD_MTU) {
            fd_frame = frame_buf.cf;
          } else {
            // Invalid frame size or partial read
            std::cerr << "Warning: Incomplete or invalid CAN frame received (size: " << nbytes
                      << ")" << std::endl;
            continue;
          }

          callback_(fd_frame, metadata);
        } catch (const SocketError & e) {
          std::cerr << "CanSocket receiver error: " << e.what() << std::endl;
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

/**
 * @brief Parse CAN filters from string format "id:mask,id:mask".
 *
 * For each filter:
 * - If only ID is given, mask is auto-generated based on ID range (SFF vs EFF).
 * - If ID > CAN_SFF_MASK (0x7FF), it's treated as an extended ID.
 *
 * @param filters_str Comma-separated filter definitions (e.g., "0x123:0x7FF,0x456").
 * @return Vector of can_filter structures.
 */
inline std::vector<can_filter> parse_can_filters(const std::string & filters_str)
{
  std::vector<can_filter> filters;
  if (filters_str.empty()) {
    return filters;
  }

  std::istringstream stream(filters_str);
  std::string part;

  while (std::getline(stream, part, ',')) {
    // Trim whitespace
    auto start = part.find_first_not_of(" \t");
    auto end = part.find_last_not_of(" \t");
    if (start == std::string::npos) continue;
    part = part.substr(start, end - start + 1);

    // Find colon separator
    auto colon_pos = part.find(':');

    try {
      can_filter filter{};
      if (colon_pos != std::string::npos) {
        filter.can_id = std::stoul(part.substr(0, colon_pos), nullptr, 0);
        filter.can_mask = std::stoul(part.substr(colon_pos + 1), nullptr, 0);
      } else {
        filter.can_id = std::stoul(part, nullptr, 0);
        // Auto-generate mask based on ID range
        if (filter.can_id > CAN_SFF_MASK) {
          filter.can_id |= CAN_EFF_FLAG;
          filter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
        } else {
          filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
        }
      }
      filters.push_back(filter);
    } catch (const std::exception &) {
      std::cerr << "Warning: Invalid CAN filter entry '" << part << "', skipping" << std::endl;
      continue;
    }
  }

  return filters;
}

}  // namespace nebula::drivers::connections
