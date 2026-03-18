// Copyright 2026 TIER IV, Inc.

#include "nebula_velodyne_hw_interfaces/velodyne_hw_interface.hpp"

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

class MockVelodyneServer
{
public:
  MockVelodyneServer()
  {
    http_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(http_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in http_addr{};
    http_addr.sin_family = AF_INET;
    http_addr.sin_port = htons(0);  // Ephemeral port
    inet_pton(AF_INET, "127.0.0.2", &http_addr.sin_addr);

    if (bind(http_fd_, reinterpret_cast<struct sockaddr *>(&http_addr), sizeof(http_addr)) < 0) {
      throw std::runtime_error("bind failed for http mock");
    }

    socklen_t len = sizeof(http_addr);
    if (getsockname(http_fd_, reinterpret_cast<struct sockaddr *>(&http_addr), &len) == 0) {
      http_port_ = ntohs(http_addr.sin_port);
    }

    if (listen(http_fd_, 5) < 0) {
      throw std::runtime_error("listen failed for http mock");
    }

    running_ = true;
    http_thread_ = std::thread([this]() { http_server_loop(); });
  }

  uint16_t get_port() const { return http_port_; }

  ~MockVelodyneServer()
  {
    running_ = false;
    if (http_thread_.joinable()) http_thread_.join();
    if (http_fd_ >= 0) close(http_fd_);
  }

  int get_request_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return request_count_;
  }

private:
  void http_server_loop()
  {
    while (running_) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(http_fd_, &readfds);

      timeval tv{0, 100000};
      int ret = select(http_fd_ + 1, &readfds, nullptr, nullptr, &tv);

      if (ret > 0 && FD_ISSET(http_fd_, &readfds)) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd =
          accept(http_fd_, reinterpret_cast<struct sockaddr *>(&client_addr), &client_len);

        if (client_fd >= 0) {
          handle_http_client(client_fd);
          close(client_fd);
        }
      }
    }
  }

  void handle_http_client(int client_fd)
  {
    std::vector<char> buffer(4096);
    ssize_t bytes = recv(client_fd, buffer.data(), buffer.size() - 1, 0);
    if (bytes <= 0) return;

    buffer[bytes] = '\0';
    std::string request(buffer.data());

    std::string response;
    if (request.find("GET /cgi/snapshot.hdl") != std::string::npos) {
      std::string json = R"({
  "config": {
    "returns": "Strongest",
    "rpm": 600,
    "fov": {"start": 0, "end": 359},
    "host": {"addr": "192.168.1.10", "dport": 2368, "tport": 8308}
  }
})";
      response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " +
                 std::to_string(json.length()) + "\r\n\r\n" + json;
    } else if (
      request.find("GET /cgi/status.json") != std::string::npos ||
      request.find("GET /cgi/diag.json") != std::string::npos) {
      std::string json = R"({"status":"ok"})";
      response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " +
                 std::to_string(json.length()) + "\r\n\r\n" + json;
    } else {
      std::string json = R"({"result":"success"})";
      response =
        "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(json.length()) + "\r\n\r\n" + json;
    }

    send(client_fd, response.c_str(), response.length(), 0);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      request_count_++;
    }
  }

  int http_fd_{-1};
  int http_port_;
  std::atomic_bool running_{false};
  std::thread http_thread_;
  std::mutex mutex_;
  int request_count_{0};
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

class TestVelodyneHwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    logger_ = std::make_shared<DummyLogger>();
    hw_interface_ = std::make_shared<nebula::drivers::VelodyneHwInterface>(logger_);

    config_ = std::make_shared<nebula::drivers::VelodyneSensorConfiguration>();
    config_->host_ip = "127.0.0.1";
    config_->data_port = 2368;
    config_->gnss_port = 8308;
    config_->sensor_ip = "127.0.0.2";
    hw_interface_->initialize_sensor_configuration(config_);
  }

  void setup_mock_sender(uint16_t src_port, uint16_t dst_port)
  {
    mock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    int opt = 1;
    setsockopt(mock_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in src_addr{};
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(src_port);
    inet_pton(AF_INET, "127.0.0.2", &src_addr.sin_addr);

    if (bind(mock_fd_, reinterpret_cast<struct sockaddr *>(&src_addr), sizeof(src_addr)) < 0) {
      throw std::runtime_error("bind mock fd failed");
    }

    dst_addr_.sin_family = AF_INET;
    dst_addr_.sin_port = htons(dst_port);
    inet_pton(AF_INET, "127.0.0.1", &dst_addr_.sin_addr);
  }

  void send_mock_payload()
  {
    std::vector<uint8_t> dummy_payload(1206, 0xAB);
    sendto(
      mock_fd_, dummy_payload.data(), dummy_payload.size(), 0,
      reinterpret_cast<struct sockaddr *>(&dst_addr_), sizeof(dst_addr_));
  }

  void TearDown() override
  {
    if (hw_interface_) {
      hw_interface_->sensor_interface_stop();
    }
  }

  std::shared_ptr<DummyLogger> logger_;
  std::shared_ptr<nebula::drivers::VelodyneSensorConfiguration> config_;
  std::shared_ptr<nebula::drivers::VelodyneHwInterface> hw_interface_;

  int mock_fd_{-1};
  sockaddr_in dst_addr_{};
};

TEST_F(TestVelodyneHwInterface, TestLifecycle)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->sensor_interface_stop(), nebula::Status::OK);
}

TEST_F(TestVelodyneHwInterface, TestSensorPacketCallback)
{
  EXPECT_EQ(hw_interface_->sensor_interface_start(), nebula::Status::OK);

  std::atomic_bool callback_triggered{false};
  hw_interface_->register_scan_callback([&](const std::vector<uint8_t> & buffer) {
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

TEST_F(TestVelodyneHwInterface, TestHTTPMethods)
{
  MockVelodyneServer mock_server;
  hw_interface_->set_target_port(mock_server.get_port());
  EXPECT_EQ(hw_interface_->init_http_client(), nebula::Status::OK);

  // Test getters
  auto snapshot = hw_interface_->get_snapshot();
  EXPECT_TRUE(snapshot.has_value());
  EXPECT_NE(snapshot.value().find("config"), std::string::npos);  // Should match mock JSON

  auto status = hw_interface_->get_status();
  EXPECT_TRUE(status.has_value());
  EXPECT_NE(status.value().find("ok"), std::string::npos);

  auto diag = hw_interface_->get_diag();
  EXPECT_TRUE(diag.has_value());

  int initial_requests = mock_server.get_request_count();
  EXPECT_GE(initial_requests, 3);

  // Test Setters
  EXPECT_EQ(hw_interface_->set_rpm(600), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_rpm(599), nebula::VelodyneStatus::INVALID_RPM_ERROR);

  EXPECT_EQ(hw_interface_->set_fov_start(100), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_fov_start(360), nebula::VelodyneStatus::INVALID_FOV_ERROR);

  EXPECT_EQ(hw_interface_->set_fov_end(200), nebula::Status::OK);

  EXPECT_EQ(
    hw_interface_->set_return_type(nebula::drivers::ReturnMode::SINGLE_STRONGEST),
    nebula::Status::OK);
  EXPECT_EQ(
    hw_interface_->set_return_type(nebula::drivers::ReturnMode::DUAL_ONLY), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->save_config(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->reset_system(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->laser_on(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->laser_off(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->laser_on_off(true), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_host_addr("192.168.1.11"), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_host_dport(2369), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_host_tport(8309), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_net_addr("192.168.1.201"), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_net_mask("255.255.255.0"), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_net_gateway("192.168.1.1"), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_net_dhcp(false), nebula::Status::OK);

  // Verify many requests were made
  EXPECT_GT(mock_server.get_request_count(), initial_requests + 15);
}

TEST_F(TestVelodyneHwInterface, TestSetSensorConfiguration)
{
  MockVelodyneServer mock_server;
  hw_interface_->set_target_port(mock_server.get_port());
  EXPECT_EQ(hw_interface_->init_http_client(), nebula::Status::OK);

  // The mock snapshot returns rpm 600, returns Strongest, fov start 0 end 359
  auto new_config = std::make_shared<nebula::drivers::VelodyneSensorConfiguration>(*config_);
  new_config->rotation_speed = 1200;                                 // triggers set_rpm
  new_config->return_mode = nebula::drivers::ReturnMode::DUAL_ONLY;  // triggers set_return_type
  new_config->cloud_min_angle = 10;
  new_config->cloud_max_angle = 350;

  EXPECT_EQ(hw_interface_->set_sensor_configuration(new_config), nebula::Status::OK);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
