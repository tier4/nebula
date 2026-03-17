// Copyright 2026 TIER IV, Inc.

#include "nebula_continental_hw_interfaces/continental_srr520_hw_interface.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

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

class MockCanServer
{
public:
  MockCanServer()
  {
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, fds_) == -1) {
      throw std::runtime_error("Failed to create socketpair");
    }

    running_ = true;
    thread_ = std::thread([this]() {
      while (running_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fds_[1], &readfds);

        // AF_UNIX datagrams are received whole
        timeval tv{0, 50000};  // 50ms timeout
        int ret = select(fds_[1] + 1, &readfds, nullptr, nullptr, &tv);

        if (ret > 0 && FD_ISSET(fds_[1], &readfds)) {
          canfd_frame frame{};
          ssize_t bytes = recv(fds_[1], &frame, sizeof(frame), 0);
          if (bytes > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            received_frames_.push_back(frame);
          }
        }
      }
    });
  }

  ~MockCanServer()
  {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fds_[0] != -1) close(fds_[0]);
    if (fds_[1] != -1) close(fds_[1]);
  }

  std::vector<canfd_frame> get_received_frames()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_frames_;
  }

  void clear_frames()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_frames_.clear();
  }

  std::string get_interface_name() const { return "mock_fd:" + std::to_string(fds_[0]); }
  int get_sender_fd() const { return fds_[1]; }

private:
  int fds_[2]{-1, -1};
  std::atomic_bool running_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<canfd_frame> received_frames_;
};

class TestSrr520HwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    logger_ = std::make_shared<DummyLogger>();
    hw_interface_ =
      std::make_shared<nebula::drivers::continental_srr520::ContinentalSRR520HwInterface>(logger_);

    mock_sensor_ = std::make_unique<MockCanServer>();

    config_ =
      std::make_shared<nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>();

    config_->interface = mock_sensor_->get_interface_name();
    config_->receiver_timeout_sec = 0.5;
    config_->use_bus_time = false;
    config_->sync_use_bus_time = false;

    hw_interface_->set_sensor_configuration(config_);
  }

  void TearDown() override
  {
    if (hw_interface_) {
      hw_interface_->sensor_interface_stop();
    }
    mock_sensor_.reset();
  }

  std::shared_ptr<DummyLogger> logger_;
  std::shared_ptr<nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    config_;
  std::shared_ptr<nebula::drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface_;
  std::unique_ptr<MockCanServer> mock_sensor_;
};

// --- Test Cases ---

TEST_F(TestSrr520HwInterface, TestLifecycle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->sensor_interface_stop(), nebula::Status::OK);
}

TEST_F(TestSrr520HwInterface, TestPacketCallback)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  std::atomic_bool callback_triggered{false};
  hw_interface_->register_packet_callback(
    [&](std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet) {
      callback_triggered = true;
      EXPECT_GT(packet->data.size(), 0u);
    });

  // Inject a fake CAN packet from the mock sensor
  canfd_frame fake_frame{};
  fake_frame.can_id = 0x123;
  fake_frame.len = 8;
  for (int i = 0; i < 8; ++i) fake_frame.data[i] = i;

  send(mock_sensor_->get_sender_fd(), &fake_frame, sizeof(fake_frame), 0);

  for (int i = 0; i < 20; ++i) {
    if (callback_triggered) break;
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_TRUE(callback_triggered);
}

TEST_F(TestSrr520HwInterface, TestConfigureSensor)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->configure_sensor(0, 40.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, false, false),
    nebula::Status::SENSOR_CONFIG_ERROR);
  EXPECT_EQ(
    hw_interface_->configure_sensor(1, 2.0f, 0.5f, 1.0f, 0.1f, 0.1f, 2.5f, 0.5f, true, false),
    nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto frames = mock_sensor_->get_received_frames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].can_id, nebula::drivers::continental_srr520::sensor_config_can_message_id);
  EXPECT_EQ(frames[0].len, 16u);

  // Verify basic payload content - sensor ID is in data[0]
  EXPECT_EQ(frames[0].data[0], 1);
  // Verify plug/reset value in data[15]
  EXPECT_EQ(frames[0].data[15], 0x00);  // plug_bottom=true -> 0x00, reset=false -> 0x00
}

TEST_F(TestSrr520HwInterface, TestSetVehicleDynamics)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_vehicle_dynamics(20.0f, 0.f, 0.f, 0.f, true),
    nebula::Status::SENSOR_CONFIG_ERROR);
  EXPECT_EQ(
    hw_interface_->set_vehicle_dynamics(2.5f, 1.0f, 0.1f, 15.0f, false), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto frames = mock_sensor_->get_received_frames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].can_id, nebula::drivers::continental_srr520::veh_dyn_can_message_id);
  EXPECT_EQ(frames[0].len, 8u);
  EXPECT_EQ(frames[0].data[5], 0x01);
}

TEST_F(TestSrr520HwInterface, TestSensorSync)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  // Force sensor_sync to send follow up messages
  for (int i = 0; i < 3; ++i) {
    hw_interface_->sensor_sync();
    // In SRR520 'sync_use_bus_time == false', it sends TWO frames per sync: sync and follow-up
    std::this_thread::sleep_for(50ms);
  }

  auto frames = mock_sensor_->get_received_frames();
  // 3 calls * 2 frames = 6 frames
  EXPECT_EQ(frames.size(), 6u);
  for (const auto & frame : frames) {
    EXPECT_EQ(frame.can_id, nebula::drivers::continental_srr520::sync_follow_up_can_message_id);
    EXPECT_EQ(frame.len, 8u);
  }
}

TEST_F(TestSrr520HwInterface, TestSensorSyncFollowUp)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  // First need to do an initial sync so that sync_follow_up_sent_ is reset
  hw_interface_->sensor_sync();
  mock_sensor_->clear_frames();

  // Test providing a follow up manual stamp
  builtin_interfaces::msg::Time follow_up_stamp;
  follow_up_stamp.sec = 1000;
  follow_up_stamp.nanosec = 0;

  config_->sync_use_bus_time = true;
  hw_interface_->sensor_sync_follow_up(follow_up_stamp);

  std::this_thread::sleep_for(50ms);
  auto frames = mock_sensor_->get_received_frames();
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
