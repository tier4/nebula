// Copyright 2025 TIER IV, Inc.

#include "nebula_continental_hw_interfaces/continental_ars548_hw_interface.hpp"

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

class MockUdpServer
{
public:
  MockUdpServer(const std::string & ip, uint16_t port)
  {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("socket failed");

    int opt = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &address.sin_addr);

    if (bind(fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error("bind failed");
    }

    running_ = true;
    thread_ = std::thread([this]() {
      while (running_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);

        timeval tv{0, 50000};  // 50ms timeout
        int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);

        if (ret > 0 && FD_ISSET(fd_, &readfds)) {
          std::vector<uint8_t> buffer(65536);
          sockaddr_in client_addr{};
          socklen_t client_len = sizeof(client_addr);
          ssize_t bytes = recvfrom(
            fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<struct sockaddr *>(&client_addr),
            &client_len);

          if (bytes > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            received_packets_.push_back(
              std::vector<uint8_t>(buffer.begin(), buffer.begin() + bytes));
          }
        }
      }
    });
  }

  ~MockUdpServer()
  {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) close(fd_);
  }

  std::vector<std::vector<uint8_t>> get_received_packets()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_packets_;
  }

  void clear_packets()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_packets_.clear();
  }

private:
  int fd_{-1};
  std::atomic_bool running_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<std::vector<uint8_t>> received_packets_;
};

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

class TestArs548HwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    logger_ = std::make_shared<DummyLogger>();
    hw_interface_ =
      std::make_shared<nebula::drivers::continental_ars548::ContinentalARS548HwInterface>(logger_);

    config_ =
      std::make_shared<nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>();
    config_->host_ip = "127.0.0.1";
    config_->data_port = 42102;
    config_->sensor_ip = "127.0.0.1";
    config_->configuration_sensor_port = 42101;
    config_->multicast_ip = "224.0.2.2";

    hw_interface_->set_sensor_configuration(config_);

    // Start a mock server representing the ARS548 sensor listening for configuration commands
    mock_sensor_ =
      std::make_unique<MockUdpServer>(config_->sensor_ip, config_->configuration_sensor_port);
  }

  void TearDown() override
  {
    if (hw_interface_) {
      hw_interface_->sensor_interface_stop();
    }
    mock_sensor_.reset();
  }

  std::shared_ptr<DummyLogger> logger_;
  std::shared_ptr<nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>
    config_;
  std::shared_ptr<nebula::drivers::continental_ars548::ContinentalARS548HwInterface> hw_interface_;
  std::unique_ptr<MockUdpServer> mock_sensor_;
};

// --- Test Cases ---

TEST_F(TestArs548HwInterface, TestLifecycle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->sensor_interface_stop(), nebula::Status::OK);
}

TEST_F(TestArs548HwInterface, TestPacketCallback)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  std::atomic_bool callback_triggered{false};
  hw_interface_->register_packet_callback(
    [&](std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet) {
      callback_triggered = true;
      EXPECT_GT(packet->data.size(), 0u);
    });

  // Inject a fake data packet from the mock sensor
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

  sockaddr_in src_addr{};
  src_addr.sin_family = AF_INET;
  src_addr.sin_port = htons(config_->data_port);
  inet_pton(AF_INET, config_->sensor_ip.c_str(), &src_addr.sin_addr);
  bind(sockfd, reinterpret_cast<struct sockaddr *>(&src_addr), sizeof(src_addr));

  sockaddr_in dest_addr{};
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(config_->data_port);
  inet_pton(AF_INET, config_->multicast_ip.c_str(), &dest_addr.sin_addr);

  // Buffer must be >= sizeof(HeaderPacket) (8 bytes) to pass validation check
  std::vector<uint8_t> fake_data(16, 0xAA);
  sendto(
    sockfd, fake_data.data(), fake_data.size(), 0, reinterpret_cast<struct sockaddr *>(&dest_addr),
    sizeof(dest_addr));
  close(sockfd);

  for (int i = 0; i < 20; ++i) {
    if (callback_triggered) break;
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_TRUE(callback_triggered);
}

TEST_F(TestArs548HwInterface, TestSetSensorMounting)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_sensor_mounting(101.f, 0.f, 1.0f, 0.f, 0.f, 0),
    nebula::Status::SENSOR_CONFIG_ERROR);

  EXPECT_EQ(
    hw_interface_->set_sensor_mounting(2.0f, 0.5f, 1.0f, 0.1f, 0.05f, 1), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::ConfigurationPacket));
}

TEST_F(TestArs548HwInterface, TestSetVehicleParameters)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_vehicle_parameters(101.f, 2.0f, 1.5f, 3.0f),
    nebula::Status::SENSOR_CONFIG_ERROR);

  EXPECT_EQ(hw_interface_->set_vehicle_parameters(4.5f, 2.0f, 1.5f, 3.0f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::ConfigurationPacket));
}

TEST_F(TestArs548HwInterface, TestSetRadarParameters)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_radar_parameters(90, 1, 50, 10, 1, 0), nebula::Status::SENSOR_CONFIG_ERROR);

  EXPECT_EQ(hw_interface_->set_radar_parameters(300, 1, 50, 10, 1, 0), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::ConfigurationPacket));
}

TEST_F(TestArs548HwInterface, TestSetSensorIpAddress)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_sensor_ip_address("invalid_ip"), nebula::Status::SENSOR_CONFIG_ERROR);
  EXPECT_EQ(hw_interface_->set_sensor_ip_address("192.168.2.5"), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::ConfigurationPacket));
}

TEST_F(TestArs548HwInterface, TestSetAccelerationLateralCog)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_acceleration_lateral_cog(70.0f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_acceleration_lateral_cog(5.0f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(
    packets[0].size(), sizeof(nebula::drivers::continental_ars548::AccelerationLateralCoGPacket));
}

TEST_F(TestArs548HwInterface, TestSetAccelerationLongitudinalCog)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_acceleration_longitudinal_cog(-70.f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_acceleration_longitudinal_cog(-2.5f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(
    packets[0].size(),
    sizeof(nebula::drivers::continental_ars548::AccelerationLongitudinalCoGPacket));
}

TEST_F(TestArs548HwInterface, TestSetCharacteristicSpeed)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_characteristic_speed(-1.f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_characteristic_speed(20.0f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(
    packets[0].size(), sizeof(nebula::drivers::continental_ars548::CharacteristicSpeedPacket));
}

TEST_F(TestArs548HwInterface, TestSetDrivingDirection)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_driving_direction(0), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_driving_direction(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_driving_direction(-1), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 3u);
  for (const auto & packet : packets) {
    EXPECT_EQ(packet.size(), sizeof(nebula::drivers::continental_ars548::DrivingDirectionPacket));
  }
}

TEST_F(TestArs548HwInterface, TestSetSteeringAngleFrontAxle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_steering_angle_front_axle(100.f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_steering_angle_front_axle(45.f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(
    packets[0].size(), sizeof(nebula::drivers::continental_ars548::SteeringAngleFrontAxlePacket));
}

TEST_F(TestArs548HwInterface, TestSetVelocityVehicle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_velocity_vehicle(400.f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_velocity_vehicle(120.f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::VelocityVehiclePacket));
}

TEST_F(TestArs548HwInterface, TestSetYawRate)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_yaw_rate(-200.f), nebula::Status::ERROR_1);
  EXPECT_EQ(hw_interface_->set_yaw_rate(10.f), nebula::Status::OK);

  std::this_thread::sleep_for(100ms);
  auto packets = mock_sensor_->get_received_packets();
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].size(), sizeof(nebula::drivers::continental_ars548::YawRatePacket));
}

TEST_F(TestArs548HwInterface, TestSafeSendFailure)
{
  // Do not call sensor_interface_start().
  // Thus udp_socket_ is null, and setting parameters should naturally fail at safe_send().
  EXPECT_EQ(hw_interface_->set_velocity_vehicle(120.f), nebula::Status::UDP_CONNECTION_ERROR);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
