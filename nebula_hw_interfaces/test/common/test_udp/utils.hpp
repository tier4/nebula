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

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

std::optional<int> udp_send(
  const char * to_ip, uint16_t to_port, const std::vector<uint8_t> & bytes)
{
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) return errno;

  int enable = 1;
  int result = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  if (result < 0) return errno;

  sockaddr_in receiver_addr{};
  memset(&receiver_addr, 0, sizeof(receiver_addr));
  receiver_addr.sin_family = AF_INET;
  receiver_addr.sin_port = htons(to_port);
  receiver_addr.sin_addr.s_addr = inet_addr(to_ip);

  result = sendto(
    sock_fd, bytes.data(), bytes.size(), 0, reinterpret_cast<const sockaddr *>(&receiver_addr),
    sizeof(receiver_addr));
  if (result < 0) return errno;
  result = close(sock_fd);
  if (result < 0) return errno;
  return {};
}

template <typename _T, typename _R>
std::optional<
  std::pair<std::vector<uint8_t>, nebula::drivers::connections::UdpSocket::ReceiveMetadata>>
receive_once(nebula::drivers::connections::UdpSocket & sock, std::chrono::duration<_T, _R> timeout)
{
  std::condition_variable cv_received_result;
  std::mutex mtx_result;
  std::optional<
    std::pair<std::vector<uint8_t>, nebula::drivers::connections::UdpSocket::ReceiveMetadata>>
    result;

  sock.subscribe([&](const auto & data, const auto & metadata) {
    std::lock_guard lock(mtx_result);
    result.emplace(data, metadata);
    cv_received_result.notify_one();
  });

  std::unique_lock lock(mtx_result);
  cv_received_result.wait_for(lock, timeout, [&result]() { return result.has_value(); });
  sock.unsubscribe();
  return result;
}
