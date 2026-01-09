// Copyright 2025 TIER IV, Inc.

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/can.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

static const char * const g_can_interface = "vcan0";

TEST(TestCan, TestLifecycle)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    sock.subscribe([](const auto &) {});
    sock.unsubscribe();
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface << ", skipping CAN tests";
  }
}

TEST(TestCan, TestSendReceive)
{
  try {
    auto sock1 = CanSocket::Builder(g_can_interface).bind();
    auto sock2 = CanSocket::Builder(g_can_interface).bind();

    std::atomic_bool received{false};
    can_frame sent_frame{};
    sent_frame.can_id = 0x123;
    sent_frame.can_dlc = 4;
    sent_frame.data[0] = 0xDE;
    sent_frame.data[1] = 0xAD;
    sent_frame.data[2] = 0xBE;
    sent_frame.data[3] = 0xEF;

    sock2.subscribe([&](const can_frame & frame) {
      if (
        frame.can_id == sent_frame.can_id && frame.can_dlc == sent_frame.can_dlc &&
        std::memcmp(frame.data, sent_frame.data, frame.can_dlc) == 0) {
        received = true;
      }
    });

    sock1.send(sent_frame);

    for (int i = 0; i < 20; ++i) {
      if (received) break;
      std::this_thread::sleep_for(50ms);
    }

    ASSERT_TRUE(received);
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface << ", skipping CAN tests";
  }
}

TEST(TestCan, TestSendReceiveFD)
{
  try {
    auto sock1 = CanSocket::Builder(g_can_interface).bind();
    auto sock2 = CanSocket::Builder(g_can_interface).bind();

    sock1.set_fd_mode(true);
    sock2.set_fd_mode(true);

    canfd_frame sent_frame{};
    sent_frame.can_id = 0x456;
    sent_frame.len = 12;
    // Set some data (0..11)
    for (int i = 0; i < 12; ++i) sent_frame.data[i] = i;

    // CanSocket::subscribe currently only supports standard CAN frames (can_frame).
    // For CAN FD (canfd_frame), we use the explicit receive_fd method.

    std::thread sender([&]() {
      std::this_thread::sleep_for(100ms);
      sock1.send_fd(sent_frame);
    });

    canfd_frame recv_frame{};
    bool success = sock2.receive_fd(recv_frame, 1000ms);
    sender.join();

    ASSERT_TRUE(success);
    ASSERT_EQ(recv_frame.can_id, sent_frame.can_id);
    ASSERT_EQ(recv_frame.len, sent_frame.len);
    ASSERT_EQ(std::memcmp(recv_frame.data, sent_frame.data, sent_frame.len), 0);
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface << ", or FD not supported";
  }
}

TEST(TestCan, TestBuilderSetPollingInterval)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).set_polling_interval(50).bind();
    (void)sock;  // Unused, just checking it doesn't throw
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

TEST(TestCan, TestIsSubscribed)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    ASSERT_FALSE(sock.is_subscribed());
    sock.subscribe([](const auto &) {});
    ASSERT_TRUE(sock.is_subscribed());
    sock.unsubscribe();
    ASSERT_FALSE(sock.is_subscribed());
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

TEST(TestCan, TestClose)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    sock.subscribe([](const auto &) {});
    ASSERT_TRUE(sock.is_subscribed());
    sock.close();
    ASSERT_FALSE(sock.is_subscribed());
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

TEST(TestCan, TestSetTimestamping)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    ASSERT_NO_THROW(sock.set_timestamping(true));
    ASSERT_NO_THROW(sock.set_timestamping(false));
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

TEST(TestCan, TestSetFilters)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    std::vector<can_filter> filters;
    can_filter filter{};
    filter.can_id = 0x123;
    filter.can_mask = CAN_SFF_MASK;
    filters.push_back(filter);
    ASSERT_NO_THROW(sock.set_filters(filters));
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

TEST(TestCan, TestReceiveFdWithMetadata)
{
  try {
    auto sock1 = CanSocket::Builder(g_can_interface).bind();
    auto sock2 = CanSocket::Builder(g_can_interface).bind();

    sock1.set_fd_mode(true);
    sock2.set_fd_mode(true);
    sock2.set_timestamping(true);

    canfd_frame sent_frame{};
    sent_frame.can_id = 0x789;
    sent_frame.len = 8;
    for (int i = 0; i < 8; ++i) sent_frame.data[i] = i;

    std::thread sender([&]() {
      std::this_thread::sleep_for(100ms);
      sock1.send_fd(sent_frame);
    });

    canfd_frame recv_frame{};
    CanSocket::RxMetadata metadata{};
    bool success = sock2.receive_fd(recv_frame, 1000ms, metadata);
    sender.join();

    ASSERT_TRUE(success);
    ASSERT_EQ(recv_frame.can_id, sent_frame.can_id);
    // Timestamp should be non-zero (either from kernel or fallback)
    EXPECT_GT(metadata.timestamp_ns, 0u);
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface << ", or FD not supported";
  }
}

TEST(TestCan, TestReceiveFdTimeout)
{
  try {
    auto sock = CanSocket::Builder(g_can_interface).bind();
    sock.set_fd_mode(true);

    canfd_frame frame{};
    // Should timeout since nobody is sending
    bool success = sock.receive_fd(frame, 50ms);
    ASSERT_FALSE(success);
  } catch (const SocketError &) {
    GTEST_SKIP() << "Could not bind to " << g_can_interface;
  }
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
