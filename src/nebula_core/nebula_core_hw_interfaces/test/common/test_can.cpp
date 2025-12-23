// Copyright 2024 TIER IV, Inc.

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

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
