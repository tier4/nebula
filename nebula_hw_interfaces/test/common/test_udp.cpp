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
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

static const char localhost_ip[] = "127.0.0.1";
static const char broadcast_ip[] = "255.255.255.255";
static const char any_ip[] = "0.0.0.0";
static const char multicast_group[] = "230.1.2.3";

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
    UdpSocket().init(localhost_ip, host_port).bind().subscribe(empty_cb()).unsubscribe());
}

TEST(test_udp, test_special_addresses_bind)
{
  ASSERT_NO_THROW(UdpSocket().init(broadcast_ip, host_port).bind());
  ASSERT_NO_THROW(UdpSocket().init(any_ip, host_port).bind());
}

TEST(test_udp, test_wildcard_multicast_join_error)
{
  ASSERT_THROW(
    UdpSocket().init(broadcast_ip, host_port).join_multicast_group(multicast_group).bind(),
    SocketError);
}

TEST(test_udp, test_buffer_resize)
{
  auto rmem_max_maybe = read_sys_param("net.core.rmem_max");
  if (!rmem_max_maybe.has_value()) GTEST_SKIP() << rmem_max_maybe.error();
  size_t rmem_max = rmem_max_maybe.value();

  // Setting buffer sizes up to and including rmem_max shall succeed
  ASSERT_NO_THROW(
    UdpSocket().init(localhost_ip, host_port).set_socket_buffer_size(rmem_max).bind());

  // Linux only supports sizes up to INT32_MAX
  ASSERT_THROW(
    UdpSocket()
      .init(localhost_ip, host_port)
      .set_socket_buffer_size(static_cast<size_t>(INT32_MAX) + 1)
      .bind(),
    UsageError);
}

TEST(test_udp, test_correct_usage_is_enforced)
{
  // These functions require other functions (e.g. `init()`) to be called beforehand
  ASSERT_THROW(UdpSocket().bind(), UsageError);
  ASSERT_THROW(UdpSocket().join_multicast_group(multicast_group), UsageError);
  ASSERT_THROW(UdpSocket().subscribe(empty_cb()), UsageError);

  // The following functions can be called in any order, any number of times
  ASSERT_NO_THROW(UdpSocket().limit_to_sender(sender_ip, sender_port));
  ASSERT_NO_THROW(UdpSocket().init(localhost_ip, host_port));
  ASSERT_NO_THROW(UdpSocket()
                    .limit_to_sender(sender_ip, sender_port)
                    .init(localhost_ip, host_port)
                    .limit_to_sender(sender_ip, sender_port)
                    .init(localhost_ip, host_port)
                    .bind());

  // Sockets cannot be re-bound
  ASSERT_THROW(UdpSocket().init(localhost_ip, host_port).bind().bind(), UsageError);

  ASSERT_THROW(
    UdpSocket().init(localhost_ip, host_port).bind().init(localhost_ip, host_port), UsageError);
  ASSERT_NO_THROW(
    UdpSocket().init(localhost_ip, host_port).bind().limit_to_sender(sender_ip, sender_port));

  // Only bound sockets can be subscribed
  ASSERT_THROW(UdpSocket().init(localhost_ip, host_port).subscribe(empty_cb()), UsageError);
  ASSERT_NO_THROW(UdpSocket().init(localhost_ip, host_port).bind().subscribe(empty_cb()));

  // Only one callback can exist
  ASSERT_THROW(
    UdpSocket().init(localhost_ip, host_port).bind().subscribe(empty_cb()).subscribe(empty_cb()),
    UsageError);

  // But un- and re-subscribing shall be supported
  ASSERT_NO_THROW(UdpSocket()
                    .init(localhost_ip, host_port)
                    .bind()
                    .subscribe(empty_cb())
                    .unsubscribe()
                    .subscribe(empty_cb()));

  // Unsubscribing on a non-subscribed socket shall also be supported
  ASSERT_NO_THROW(UdpSocket().unsubscribe());
}

TEST(test_udp, test_receiving)
{
  const std::vector<uint8_t> payload{1, 2, 3};
  UdpSocket sock{};
  sock.init(localhost_ip, host_port).bind();

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
  payload.resize(mtu + 1);
  UdpSocket sock{};
  sock.init(localhost_ip, host_port).set_mtu(mtu).bind();

  auto err_no_opt = udp_send(localhost_ip, host_port, payload);
  if (err_no_opt.has_value()) GTEST_SKIP() << strerror(err_no_opt.value());

  auto result_opt = receive_once(sock, send_receive_timeout);

  ASSERT_TRUE(result_opt.has_value());
  auto const & [recv_payload, metadata] = result_opt.value();
  ASSERT_TRUE(std::equal(recv_payload.begin(), recv_payload.end(), payload.begin()));
  ASSERT_TRUE(metadata.truncated);
  ASSERT_EQ(metadata.drops_since_last_receive, 0);
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
};