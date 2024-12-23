// Copyright 2024 TIER IV, Inc.

#include "common/test_udp/utils.hpp"
#include "nebula_common/util/expected.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

static const char localhost_ip[] = "127.0.0.1";
static const char broadcast_ip[] = "255.255.255.255";
static const char any_ip[] = "0.0.0.0";
static const char multicast_group[] = "230.1.2.3";
static const char multicast_group2[] = "230.4.5.6";

static const char sender_ip[] = "192.168.201";
static const uint16_t sender_port = 7373;
static const uint16_t host_port = 6262;

static const std::chrono::duration send_receive_timeout = 100ms;

UdpSocket::callback_t empty_cb()
{
  return [](const auto &, const auto &) {};
}

util::expected<uint64_t, std::string> read_sys_param(const std::string & param_fqn)
{
  std::string path = "/proc/sys/" + param_fqn;
  std::replace(path.begin(), path.end(), '.', '/');
  std::ifstream ifs{path};
  if (!ifs) return "could not read " + param_fqn;

  size_t param{};
  if (!(ifs >> param)) return param_fqn + " has unrecognized format";
  return param;
}

TEST(test_udp, test_basic_lifecycle)
{
  ASSERT_NO_THROW(
    UdpSocket::Builder(localhost_ip, host_port).bind().subscribe(empty_cb()).unsubscribe());
}

TEST(test_udp, test_special_addresses_bind)
{
  ASSERT_THROW(UdpSocket::Builder(broadcast_ip, host_port), UsageError);
  ASSERT_NO_THROW(UdpSocket::Builder(any_ip, host_port).bind());
}

TEST(test_udp, test_joining_invalid_multicast_group)
{
  ASSERT_THROW(
    UdpSocket::Builder(localhost_ip, host_port).join_multicast_group(broadcast_ip).bind(),
    SocketError);
}

TEST(test_udp, test_buffer_resize)
{
  auto rmem_max_maybe = read_sys_param("net.core.rmem_max");
  if (!rmem_max_maybe.has_value()) GTEST_SKIP() << rmem_max_maybe.error();
  size_t rmem_max = rmem_max_maybe.value();

  // Setting buffer sizes up to and including rmem_max shall succeed
  ASSERT_NO_THROW(
    UdpSocket::Builder(localhost_ip, host_port).set_socket_buffer_size(rmem_max).bind());

  // Linux only supports sizes up to INT32_MAX
  ASSERT_THROW(
    UdpSocket::Builder(localhost_ip, host_port)
      .set_socket_buffer_size(static_cast<size_t>(INT32_MAX) + 1)
      .bind(),
    UsageError);
}

TEST(test_udp, test_correct_usage_is_enforced)
{
  // The following functions can be called in any order, any number of times
  ASSERT_NO_THROW(UdpSocket::Builder(localhost_ip, host_port)
                    .set_polling_interval(20)
                    .set_socket_buffer_size(3000)
                    .set_mtu(1600)
                    .limit_to_sender(sender_ip, sender_port)
                    .set_polling_interval(20)
                    .set_socket_buffer_size(3000)
                    .set_mtu(1600)
                    .limit_to_sender(sender_ip, sender_port)
                    .bind());

  // Only one multicast group can be joined
  ASSERT_THROW(
    UdpSocket::Builder(localhost_ip, host_port)
      .join_multicast_group(multicast_group)
      .join_multicast_group(multicast_group2),
    UsageError);

  // Pre-existing subscriptions shall be gracefully unsubscribed when a new subscription is created
  ASSERT_NO_THROW(
    UdpSocket::Builder(localhost_ip, host_port).bind().subscribe(empty_cb()).subscribe(empty_cb()));

  // Explicitly unsubscribing shall be supported
  ASSERT_NO_THROW(UdpSocket::Builder(localhost_ip, host_port)
                    .bind()
                    .subscribe(empty_cb())
                    .unsubscribe()
                    .subscribe(empty_cb()));

  // Unsubscribing on a non-subscribed socket shall also be supported
  ASSERT_NO_THROW(UdpSocket::Builder(localhost_ip, host_port).bind().unsubscribe());
}

TEST(test_udp, test_receiving)
{
  const std::vector<uint8_t> payload{1, 2, 3};
  auto sock = UdpSocket::Builder(localhost_ip, host_port).bind();

  auto err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  auto result_opt = receive_once(sock, send_receive_timeout);

  ASSERT_TRUE(result_opt.has_value());
  auto const & [recv_payload, metadata] = result_opt.value();
  ASSERT_EQ(recv_payload, payload);
  ASSERT_FALSE(metadata.truncated);
  ASSERT_EQ(metadata.drops_since_last_receive, 0);

  // TODO(mojomex): currently cannot test timestamping on loopback interface (no timestamp produced)
}

TEST(test_udp, test_receiving_oversized)
{
  const size_t mtu = 1500;
  std::vector<uint8_t> payload;
  payload.resize(mtu + 1, 0x42);
  auto sock = UdpSocket::Builder(localhost_ip, host_port).set_mtu(mtu).bind();

  auto err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  auto result_opt = receive_once(sock, send_receive_timeout);

  ASSERT_TRUE(result_opt.has_value());
  auto const & [recv_payload, metadata] = result_opt.value();
  ASSERT_EQ(recv_payload.size(), mtu);
  ASSERT_TRUE(std::equal(recv_payload.begin(), recv_payload.end(), payload.begin()));
  ASSERT_TRUE(metadata.truncated);
  ASSERT_EQ(metadata.drops_since_last_receive, 0);
}

TEST(test_udp, test_filtering_sender)
{
  std::vector<uint8_t> payload{1, 2, 3};
  auto sock =
    UdpSocket::Builder(localhost_ip, host_port).limit_to_sender(sender_ip, sender_port).bind();

  auto err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  auto result_opt = receive_once(sock, send_receive_timeout);
  ASSERT_FALSE(result_opt.has_value());
}

TEST(test_udp, test_moveable)
{
  std::vector<uint8_t> payload{1, 2, 3};

  size_t n_received = 0;

  auto sock = UdpSocket::Builder(localhost_ip, host_port).bind();
  sock.subscribe([&n_received](const auto &, const auto &) { n_received++; });

  auto err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  // The subscription moves to the new socket object
  UdpSocket sock2{std::move(sock)};
  ASSERT_TRUE(sock2.is_subscribed());

  err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  std::this_thread::sleep_for(100ms);
  ASSERT_EQ(n_received, 2);
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
};
