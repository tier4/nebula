// Copyright 2025 TIER IV, Inc.

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/socket_utils.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <utility>

namespace nebula::drivers::connections
{

TEST(TestSocketUtils, TestParseIpValid)
{
  auto result = parse_ip("192.168.1.100");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(to_string(result.value()), "192.168.1.100");

  result = parse_ip("0.0.0.0");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(to_string(result.value()), "0.0.0.0");

  result = parse_ip("255.255.255.255");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(to_string(result.value()), "255.255.255.255");
}

TEST(TestSocketUtils, TestParseIpInvalid)
{
  EXPECT_THROW(parse_ip("invalid").value_or_throw(), UsageError);
  EXPECT_THROW(parse_ip("256.0.0.1").value_or_throw(), UsageError);
  EXPECT_THROW(parse_ip("").value_or_throw(), UsageError);
}

TEST(TestSocketUtils, TestToString)
{
  in_addr addr{};
  addr.s_addr = htonl(0xC0A80164);  // 192.168.1.100
  EXPECT_EQ(to_string(addr), "192.168.1.100");

  addr.s_addr = 0;
  EXPECT_EQ(to_string(addr), "0.0.0.0");
}

TEST(TestSocketUtils, TestSockFdRaii)
{
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

  int fd_to_check = fds[0];
  {
    SockFd sock_fd(fds[0]);
    EXPECT_EQ(sock_fd.get(), fd_to_check);
    // fds[1] needs to be closed manually
    ::close(fds[1]);
  }
  // After SockFd goes out of scope, fd should be closed
  // Attempting to write to a closed fd should fail
  char buf = 'x';
  EXPECT_EQ(write(fd_to_check, &buf, 1), -1);
}

TEST(TestSocketUtils, TestSockFdMove)
{
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

  SockFd sock1(fds[0]);
  int fd_val = sock1.get();

  SockFd sock2(std::move(sock1));
  EXPECT_EQ(sock2.get(), fd_val);
  EXPECT_EQ(sock1.get(), -1);  // NOLINT: testing moved-from state

  SockFd sock3;
  sock3 = std::move(sock2);
  EXPECT_EQ(sock3.get(), fd_val);
  EXPECT_EQ(sock2.get(), -1);  // NOLINT: testing moved-from state

  ::close(fds[1]);
}

TEST(TestSocketUtils, TestSockFdSetsockopt)
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_NE(fd, -1);

  SockFd sock_fd(fd);
  auto result = sock_fd.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1);
  EXPECT_TRUE(result.has_value());
}

TEST(TestSocketUtils, TestIsSocketReady)
{
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

  // No data available, should timeout
  auto result = is_socket_ready(fds[0], 10);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value());

  // Write data to fds[1], now fds[0] should be ready
  char buf = 'x';
  ASSERT_EQ(write(fds[1], &buf, 1), 1);

  result = is_socket_ready(fds[0], 100);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result.value());

  ::close(fds[0]);
  ::close(fds[1]);
}

TEST(TestSocketUtils, TestSocketError)
{
  SocketError err1(ENOENT);
  EXPECT_NE(std::strlen(err1.what()), 0u);

  SocketError err2("Custom error message");
  EXPECT_STREQ(err2.what(), "Custom error message");
}

TEST(TestSocketUtils, TestUsageError)
{
  UsageError err("Invalid usage");
  EXPECT_STREQ(err.what(), "Invalid usage");
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
