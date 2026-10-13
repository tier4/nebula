// Copyright 2026 TIER IV, Inc.

#include "nebula_ouster_hw_interfaces/ouster_hw_interface.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

using std::chrono_literals::operator""ms;

class TestOusterHwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_.host_ip = "127.0.0.1";
    config_.sensor_ip = "127.0.0.2";
    config_.data_port = 7502;
    config_.receiver_mtu_bytes = 65527;
    config_.filter_sender_ip = true;
  }

  void TearDown() override
  {
    if (mock_fd_ != -1) {
      close(mock_fd_);
      mock_fd_ = -1;
    }
  }

  void setup_mock_sender(uint16_t src_port, uint16_t dst_port)
  {
    mock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (mock_fd_ < 0) {
      throw std::runtime_error("socket() failed");
    }
    int opt = 1;
    if (setsockopt(mock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
      throw std::runtime_error("setsockopt failed");
    }
#ifdef SO_REUSEPORT
    if (setsockopt(mock_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
      throw std::runtime_error("setsockopt failed");
    }
#endif

    sockaddr_in src_addr{};
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(src_port);
    inet_pton(AF_INET, config_.sensor_ip.c_str(), &src_addr.sin_addr);

    if (bind(mock_fd_, reinterpret_cast<struct sockaddr *>(&src_addr), sizeof(src_addr)) < 0) {
      throw std::runtime_error("bind mock fd failed");
    }

    dst_addr_.sin_family = AF_INET;
    dst_addr_.sin_port = htons(dst_port);
    inet_pton(AF_INET, config_.host_ip.c_str(), &dst_addr_.sin_addr);
  }

  void send_mock_payload()
  {
    std::vector<uint8_t> dummy_payload(512, 0xAB);
    ssize_t sent = sendto(
      mock_fd_, dummy_payload.data(), dummy_payload.size(), 0,
      reinterpret_cast<struct sockaddr *>(&dst_addr_), sizeof(dst_addr_));
    if (sent < 0) {
      throw std::runtime_error("sendto failed");
    }
  }

  nebula::drivers::ConnectionConfiguration config_;
  int mock_fd_{-1};
  sockaddr_in dst_addr_{};
};

TEST_F(TestOusterHwInterface, TestStartWithoutCallback)
{
  nebula::drivers::OusterHwInterface hw(config_);
  auto result = hw.sensor_interface_start();
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(
    result.error().code, nebula::drivers::OusterHwInterface::Error::Code::CALLBACK_NOT_REGISTERED);
}

TEST_F(TestOusterHwInterface, TestRegisterEmptyCallback)
{
  nebula::drivers::OusterHwInterface hw(config_);
  auto result = hw.register_scan_callback(nullptr);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, nebula::drivers::OusterHwInterface::Error::Code::INVALID_CALLBACK);
}

TEST_F(TestOusterHwInterface, TestLifecycle)
{
  nebula::drivers::OusterHwInterface hw(config_);

  auto reg_result = hw.register_scan_callback(
    [](std::vector<uint8_t> &, const nebula::drivers::connections::UdpSocket::RxMetadata &) {});
  ASSERT_TRUE(reg_result.has_value());

  auto start_result = hw.sensor_interface_start();
  ASSERT_TRUE(start_result.has_value());

  auto stop_result = hw.sensor_interface_stop();
  ASSERT_TRUE(stop_result.has_value());
}

TEST_F(TestOusterHwInterface, TestDoubleStartIsIdempotent)
{
  nebula::drivers::OusterHwInterface hw(config_);

  auto reg_result = hw.register_scan_callback(
    [](std::vector<uint8_t> &, const nebula::drivers::connections::UdpSocket::RxMetadata &) {});
  ASSERT_TRUE(reg_result.has_value());

  auto start1 = hw.sensor_interface_start();
  ASSERT_TRUE(start1.has_value());

  auto start2 = hw.sensor_interface_start();
  ASSERT_TRUE(start2.has_value());

  auto stop_result = hw.sensor_interface_stop();
  ASSERT_TRUE(stop_result.has_value());
}

TEST_F(TestOusterHwInterface, TestStopWithoutStart)
{
  nebula::drivers::OusterHwInterface hw(config_);
  auto result = hw.sensor_interface_stop();
  ASSERT_TRUE(result.has_value());
}

TEST_F(TestOusterHwInterface, TestRegisterCallbackWhileRunning)
{
  nebula::drivers::OusterHwInterface hw(config_);

  auto reg_result = hw.register_scan_callback(
    [](std::vector<uint8_t> &, const nebula::drivers::connections::UdpSocket::RxMetadata &) {});
  ASSERT_TRUE(reg_result.has_value());

  auto start_result = hw.sensor_interface_start();
  ASSERT_TRUE(start_result.has_value());

  auto replace_result = hw.register_scan_callback(
    [](std::vector<uint8_t> &, const nebula::drivers::connections::UdpSocket::RxMetadata &) {});
  ASSERT_FALSE(replace_result.has_value());
  EXPECT_EQ(
    replace_result.error().code,
    nebula::drivers::OusterHwInterface::Error::Code::INVALID_OPERATION);

  auto stop_result = hw.sensor_interface_stop();
  ASSERT_TRUE(stop_result.has_value());
}

TEST_F(TestOusterHwInterface, TestScanPacketCallback)
{
  nebula::drivers::OusterHwInterface hw(config_);

  std::atomic_bool callback_triggered{false};
  auto reg_result = hw.register_scan_callback(
    [&](
      std::vector<uint8_t> & buffer, const nebula::drivers::connections::UdpSocket::RxMetadata &) {
      if (buffer.size() > 0) callback_triggered = true;
    });
  ASSERT_TRUE(reg_result.has_value());

  auto start_result = hw.sensor_interface_start();
  ASSERT_TRUE(start_result.has_value());

  setup_mock_sender(config_.data_port, config_.data_port);

  for (int i = 0; i < 5; ++i) {
    send_mock_payload();
    std::this_thread::sleep_for(10ms);
    if (callback_triggered) break;
  }

  EXPECT_TRUE(callback_triggered);

  auto stop_result = hw.sensor_interface_stop();
  ASSERT_TRUE(stop_result.has_value());
}

TEST_F(TestOusterHwInterface, TestErrorCodeToString)
{
  EXPECT_STREQ(
    nebula::drivers::OusterHwInterface::to_cstr(
      nebula::drivers::OusterHwInterface::Error::Code::CALLBACK_NOT_REGISTERED),
    "callback not registered");
  EXPECT_STREQ(
    nebula::drivers::OusterHwInterface::to_cstr(
      nebula::drivers::OusterHwInterface::Error::Code::INVALID_CALLBACK),
    "invalid callback");
  EXPECT_STREQ(
    nebula::drivers::OusterHwInterface::to_cstr(
      nebula::drivers::OusterHwInterface::Error::Code::INVALID_OPERATION),
    "invalid operation");
  EXPECT_STREQ(
    nebula::drivers::OusterHwInterface::to_cstr(
      nebula::drivers::OusterHwInterface::Error::Code::SOCKET_OPEN_FAILED),
    "failed to open UDP socket");
  EXPECT_STREQ(
    nebula::drivers::OusterHwInterface::to_cstr(
      nebula::drivers::OusterHwInterface::Error::Code::SOCKET_CLOSE_FAILED),
    "failed to close UDP socket");
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}