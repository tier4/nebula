// Copyright 2026 TIER IV, Inc.

#include "nebula_core_hw_interfaces/connections/can.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

class TestCan : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create an AF_UNIX datagram socketpair to seamlessly mock a bidirectional CAN bus
    // fds_[0] represents the interface seen by the CanSocket (the "receiver/listener")
    // fds_[1] represents the other end of the fake bus (the "sender/publisher")
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, fds_) == -1) {
      FAIL() << "Failed to create socketpair";
    }
  }

  void TearDown() override
  {
    if (fds_[0] != -1) close(fds_[0]);
    if (fds_[1] != -1) close(fds_[1]);
  }

  // Gives the file descriptor for the "node" simulating the ROS driver
  std::string get_receiver_interface() const { return "mock_fd:" + std::to_string(fds_[0]); }
  // Gives the file descriptor for the "node" simulating the external hardware sending data
  std::string get_sender_interface() const { return "mock_fd:" + std::to_string(fds_[1]); }

private:
  int fds_[2]{-1, -1};
};

TEST_F(TestCan, TestBuilderInvalidInterface)
{
  ASSERT_THROW(CanSocket::Builder("invalid_can").bind(), SocketError);
}

TEST_F(TestCan, TestLifecycle)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).bind();
  sock.subscribe([](const auto &, const auto &) {});
  sock.unsubscribe();
}

TEST_F(TestCan, TestSendReceive)
{
  auto sender = CanSocket::Builder(get_sender_interface()).bind();
  auto receiver = CanSocket::Builder(get_receiver_interface()).bind();

  std::atomic_bool received{false};
  can_frame sent_frame{};
  sent_frame.can_id = 0x123;
  sent_frame.can_dlc = 4;
  sent_frame.data[0] = 0xDE;
  sent_frame.data[1] = 0xAD;
  sent_frame.data[2] = 0xBE;
  sent_frame.data[3] = 0xEF;

  receiver.subscribe([&](const canfd_frame & frame, const auto &) {
    if (
      frame.can_id == sent_frame.can_id && frame.len == sent_frame.can_dlc &&
      std::memcmp(frame.data, sent_frame.data, frame.len) == 0) {
      received = true;
    }
  });

  sender.send(sent_frame);

  for (int i = 0; i < 20; ++i) {
    if (received) break;
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received);
}

TEST_F(TestCan, TestSendReceiveFD)
{
  auto sender = CanSocket::Builder(get_sender_interface()).bind();
  auto receiver = CanSocket::Builder(get_receiver_interface()).bind();

  sender.set_fd_mode(true);
  receiver.set_fd_mode(true);

  canfd_frame sent_frame{};
  sent_frame.can_id = 0x456;
  sent_frame.len = 12;
  // Set some data (0..11)
  for (int i = 0; i < 12; ++i) sent_frame.data[i] = i;

  std::thread sender_thread([&]() {
    std::this_thread::sleep_for(100ms);
    sender.send_fd(sent_frame);
  });

  canfd_frame recv_frame{};
  bool success = receiver.receive_fd(recv_frame, 1000ms);
  sender_thread.join();

  ASSERT_TRUE(success);
  ASSERT_EQ(recv_frame.can_id, sent_frame.can_id);
  ASSERT_EQ(recv_frame.len, sent_frame.len);
  ASSERT_EQ(std::memcmp(recv_frame.data, sent_frame.data, sent_frame.len), 0);
}

TEST_F(TestCan, TestBuilderSetPollingInterval)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).set_polling_interval(50).bind();
  (void)sock;
}

TEST_F(TestCan, TestIsSubscribed)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).bind();
  ASSERT_FALSE(sock.is_subscribed());
  sock.subscribe([](const auto &, const auto &) {});
  ASSERT_TRUE(sock.is_subscribed());
  sock.unsubscribe();
  ASSERT_FALSE(sock.is_subscribed());
}

TEST_F(TestCan, TestSetTimestamping)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).bind();
  ASSERT_NO_THROW(sock.set_timestamping(true));
  ASSERT_NO_THROW(sock.set_timestamping(false));
}

TEST_F(TestCan, TestAsyncTimestamp)
{
  auto sender = CanSocket::Builder(get_sender_interface()).bind();
  auto receiver = CanSocket::Builder(get_receiver_interface()).bind();

  receiver.set_timestamping(true);

  std::atomic_bool received{false};
  uint64_t received_ts{0};

  receiver.subscribe([&](const canfd_frame &, const CanSocket::RxMetadata & metadata) {
    if (metadata.timestamp_ns.has_value()) {
      received_ts = metadata.timestamp_ns.value();
    }
    received = true;
  });

  can_frame frame{};
  frame.can_id = 0x100;
  frame.can_dlc = 1;
  frame.data[0] = 0x01;

  sender.send(frame);

  for (int i = 0; i < 20; ++i) {
    if (received) break;
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received);
  EXPECT_GT(received_ts, 0u);
}

TEST_F(TestCan, TestSetFilters)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).bind();
  std::vector<can_filter> filters;
  can_filter filter{};
  filter.can_id = 0x123;
  filter.can_mask = CAN_SFF_MASK;
  filters.push_back(filter);
  ASSERT_NO_THROW(sock.set_filters(filters));
}

TEST_F(TestCan, TestReceiveFdWithMetadata)
{
  auto sender = CanSocket::Builder(get_sender_interface()).bind();
  auto receiver = CanSocket::Builder(get_receiver_interface()).bind();

  sender.set_fd_mode(true);
  receiver.set_fd_mode(true);
  receiver.set_timestamping(true);

  canfd_frame sent_frame{};
  sent_frame.can_id = 0x789;
  sent_frame.len = 8;
  for (int i = 0; i < 8; ++i) sent_frame.data[i] = i;

  std::thread sender_thread([&]() {
    std::this_thread::sleep_for(100ms);
    sender.send_fd(sent_frame);
  });

  canfd_frame recv_frame{};
  CanSocket::RxMetadata metadata{};
  bool success = receiver.receive_fd(recv_frame, 1000ms, metadata);
  sender_thread.join();

  ASSERT_TRUE(success);
  ASSERT_EQ(recv_frame.can_id, sent_frame.can_id);
  // Timestamp should be present when timestamping is enabled
  EXPECT_TRUE(metadata.timestamp_ns.has_value());
}

TEST_F(TestCan, TestReceiveFdTimeout)
{
  auto sock = CanSocket::Builder(get_receiver_interface()).bind();
  sock.set_fd_mode(true);

  canfd_frame frame{};
  // Should timeout since nobody is sending
  bool success = sock.receive_fd(frame, 50ms);
  ASSERT_FALSE(success);
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
