// Copyright 2026 TIER IV, Inc.

#include "nebula_robosense_hw_interfaces/robosense_hw_interface.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using std::chrono_literals::operator""ms;

class DummyLogger : public nebula::drivers::loggers::Logger
{
public:
  void debug(const std::string &) override {}
  void info(const std::string &) override {}
  void warn(const std::string &) override {}
  void error(const std::string &) override {}
  std::shared_ptr<nebula::drivers::loggers::Logger> child(const std::string &) override
  {
    return std::make_shared<DummyLogger>();
  }
};

class TestRobosenseHwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    logger_ = std::make_shared<DummyLogger>();
    hw_interface_ = std::make_shared<nebula::drivers::RobosenseHwInterface>(logger_);

    config_ = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
    config_->host_ip = "127.0.0.1";
    config_->data_port = 2368;
    config_->sensor_ip = "127.0.0.2";
    config_->gnss_port = 7788;
    config_->sensor_model = nebula::drivers::SensorModel::ROBOSENSE_HELIOS;

    hw_interface_->set_sensor_configuration(config_);
  }

  void TearDown() override
  {
    // Neither info nor sensor stop methods exist, they just destroy the UDP socket unique_ptrs.
    // They are closed when hw_interface_ is destroyed.
  }

  void setup_mock_sender(uint16_t src_port, uint16_t dst_port)
  {
    mock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    int opt = 1;
    setsockopt(mock_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in src_addr{};
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(src_port);                  // explicitly use the required sender port
    inet_pton(AF_INET, "127.0.0.2", &src_addr.sin_addr);  // bind to the mock IP

    if (bind(mock_fd_, reinterpret_cast<struct sockaddr *>(&src_addr), sizeof(src_addr)) < 0) {
      throw std::runtime_error("bind mock fd failed");
    }

    dst_addr_.sin_family = AF_INET;
    dst_addr_.sin_port = htons(dst_port);
    inet_pton(AF_INET, "127.0.0.1", &dst_addr_.sin_addr);
  }

  void send_mock_payload()
  {
    std::vector<uint8_t> dummy_payload(512, 0xAB);
    sendto(
      mock_fd_, dummy_payload.data(), dummy_payload.size(), 0,
      reinterpret_cast<struct sockaddr *>(&dst_addr_), sizeof(dst_addr_));
  }

  std::shared_ptr<DummyLogger> logger_;
  std::shared_ptr<nebula::drivers::RobosenseSensorConfiguration> config_;
  std::shared_ptr<nebula::drivers::RobosenseHwInterface> hw_interface_;

  int mock_fd_{-1};
  sockaddr_in dst_addr_{};
};

TEST_F(TestRobosenseHwInterface, TestLifecycle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->info_interface_start(), nebula::Status::OK);
}

TEST_F(TestRobosenseHwInterface, TestSetSensorConfigurationValidation)
{
  auto bad_config = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
  bad_config->sensor_model = nebula::drivers::SensorModel::UNKNOWN;
  EXPECT_EQ(
    hw_interface_->set_sensor_configuration(bad_config), nebula::Status::INVALID_SENSOR_MODEL);
}

TEST_F(TestRobosenseHwInterface, TestSensorPacketCallback)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  std::atomic_bool callback_triggered{false};
  hw_interface_->register_scan_callback([&](std::vector<uint8_t> & buffer) {
    if (buffer.size() > 0) callback_triggered = true;
  });

  setup_mock_sender(config_->data_port, config_->data_port);

  for (int i = 0; i < 5; ++i) {
    send_mock_payload();
    std::this_thread::sleep_for(10ms);
    if (callback_triggered) break;
  }

  EXPECT_TRUE(callback_triggered);
  if (mock_fd_ != -1) close(mock_fd_);
}

TEST_F(TestRobosenseHwInterface, TestInfoPacketCallback)
{
  EXPECT_EQ(hw_interface_->info_interface_start(), nebula::Status::OK);

  std::atomic_bool callback_triggered{false};
  hw_interface_->register_info_callback([&](std::vector<uint8_t> & buffer) {
    if (buffer.size() > 0) callback_triggered = true;
  });

  setup_mock_sender(config_->gnss_port, config_->gnss_port);

  for (int i = 0; i < 5; ++i) {
    send_mock_payload();
    std::this_thread::sleep_for(10ms);
    if (callback_triggered) break;
  }

  EXPECT_TRUE(callback_triggered);
  if (mock_fd_ != -1) close(mock_fd_);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
