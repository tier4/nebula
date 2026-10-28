// Copyright 2026 TIER IV, Inc.

#include "nebula_hesai_hw_interfaces/hesai_hw_interface.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
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

class MockHesaiSensor
{
public:
  MockHesaiSensor()
  {
    // Start TCP command server (PTC)
    tcp_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(tcp_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in tcp_addr{};
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    tcp_addr.sin_port = 0;  // random port

    bind(tcp_fd_, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr));
    socklen_t tcp_len = sizeof(tcp_addr);
    getsockname(tcp_fd_, (struct sockaddr *)&tcp_addr, &tcp_len);
    tcp_port_ = ntohs(tcp_addr.sin_port);
    listen(tcp_fd_, 3);

    // Start HTTP server
    http_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(http_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in http_addr{};
    http_addr.sin_family = AF_INET;
    http_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    http_addr.sin_port = 0;  // random port

    bind(http_fd_, (struct sockaddr *)&http_addr, sizeof(http_addr));
    socklen_t http_len = sizeof(http_addr);
    getsockname(http_fd_, (struct sockaddr *)&http_addr, &http_len);
    http_port_ = ntohs(http_addr.sin_port);
    listen(http_fd_, 3);

    running_ = true;

    tcp_thread_ = std::thread([this]() {
      while (running_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(tcp_fd_, &readfds);

        timeval tv{0, 50000};
        int ret = select(tcp_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(tcp_fd_, &readfds)) {
          int client = accept(tcp_fd_, nullptr, nullptr);
          if (client >= 0) {
            handle_tcp_client(client);
            close(client);
          }
        }
      }
    });

    http_thread_ = std::thread([this]() {
      while (running_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(http_fd_, &readfds);

        timeval tv{0, 50000};
        int ret = select(http_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(http_fd_, &readfds)) {
          int client = accept(http_fd_, nullptr, nullptr);
          if (client >= 0) {
            handle_http_client(client);
            close(client);
          }
        }
      }
    });
  }

  ~MockHesaiSensor()
  {
    running_ = false;
    if (tcp_thread_.joinable()) tcp_thread_.join();
    if (http_thread_.joinable()) http_thread_.join();
    if (tcp_fd_ != -1) close(tcp_fd_);
    if (http_fd_ != -1) close(http_fd_);
  }

  uint16_t get_tcp_port() const { return tcp_port_; }
  uint16_t get_http_port() const { return http_port_; }

  /// @brief Respond to `command_id` with `payload` instead of the default response
  void set_response(uint8_t command_id, std::vector<uint8_t> payload)
  {
    std::lock_guard lock(mutex_);
    responses_[command_id] = std::move(payload);
  }

  /// @brief The (command ID, payload) pairs of all requests received so far
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> get_requests()
  {
    std::lock_guard lock(mutex_);
    return requests_;
  }

private:
  void handle_tcp_client(int client)
  {
    while (running_) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(client, &readfds);
      timeval tv{0, 50000};
      if (select(client + 1, &readfds, nullptr, nullptr, &tv) <= 0) {
        continue;
      }

      std::vector<uint8_t> header(8);
      if (read(client, header.data(), 8) != 8) break;

      uint8_t command_id = header[2];
      uint32_t payload_len = (header[4] << 24) | (header[5] << 16) | (header[6] << 8) | header[7];

      std::vector<uint8_t> payload(payload_len);
      if (payload_len > 0) {
        if (read(client, payload.data(), payload_len) != payload_len) break;
      }

      // Generate Response
      std::vector<uint8_t> response_payload;
      uint8_t ptc_error = nebula::drivers::g_ptc_error_code_no_error;

      std::optional<std::vector<uint8_t>> custom_response;
      {
        std::lock_guard lock(mutex_);
        requests_.emplace_back(command_id, payload);
        if (auto it = responses_.find(command_id); it != responses_.end()) {
          custom_response = it->second;
        }
      }

      if (custom_response) {
        response_payload = *custom_response;
      } else if (command_id == nebula::drivers::g_ptc_command_get_lidar_calibration) {
        std::string calib = "mock_calibration_data";
        response_payload.assign(calib.begin(), calib.end());
      } else if (command_id == nebula::drivers::g_ptc_command_set_spin_rate) {
        // Just return OK, no payload
      } else if (command_id == nebula::drivers::g_ptc_command_get_inventory_info) {
        response_payload.resize(2048, 0);
      } else if (command_id == nebula::drivers::g_ptc_command_get_config_info) {
        response_payload.resize(2048, 0);
      } else if (command_id == nebula::drivers::g_ptc_command_get_lidar_status) {
        response_payload.resize(2048, 0);
      } else if (
        command_id == nebula::drivers::g_ptc_command_get_ptp_config ||
        command_id == nebula::drivers::g_ptc_command_ptp_diagnostics ||
        command_id == nebula::drivers::g_ptc_command_lidar_monitor) {
        response_payload.resize(4096, 0);  // Big enough for any struct parsing
      } else if (command_id == nebula::drivers::g_ptc_command_get_lidar_range) {
        response_payload.resize(5, 0);  // 0 method + 4 bytes
      } else if (
        command_id == nebula::drivers::g_ptp_command_get_ptp_lock_offset ||
        command_id == nebula::drivers::g_ptc_command_get_high_resolution_mode ||
        command_id == nebula::drivers::g_ptc_command_get_up_close_blockage_detection) {
        response_payload.resize(1, 0);
      } else if (command_id == nebula::drivers::g_ptc_command_set_ptp_config) {
        // OK
      }

      uint32_t resp_payload_len = response_payload.size();
      std::vector<uint8_t> response(8 + resp_payload_len);
      response[0] = nebula::drivers::g_ptc_command_header_high;
      response[1] = nebula::drivers::g_ptc_command_header_low;
      response[2] = command_id;
      response[3] = ptc_error;
      response[4] = (resp_payload_len >> 24) & 0xff;
      response[5] = (resp_payload_len >> 16) & 0xff;
      response[6] = (resp_payload_len >> 8) & 0xff;
      response[7] = resp_payload_len & 0xff;

      if (resp_payload_len > 0) {
        std::copy(response_payload.begin(), response_payload.end(), response.begin() + 8);
      }

      send(client, response.data(), response.size(), 0);
    }
  }

  void handle_http_client(int client)
  {
    char buf[1024];
    if (read(client, buf, 1024) > 0) {
      std::string req(buf);
      std::string resp = "HTTP/1.1 200 OK\r\nContent-Length: ";
      std::string body = "{}";

      if (req.find("GET") == 0) {
        body = "{\"system\":{\"uptime\":100}, \"ptp\":{\"status\":\"locked\"}}";
      } else if (req.find("POST") == 0) {
        body = "{\"status\": \"success\"}";
      }

      resp += std::to_string(body.length()) + "\r\n\r\n" + body;
      send(client, resp.c_str(), resp.length(), 0);
    }
  }

  int tcp_fd_;
  int http_fd_;
  uint16_t tcp_port_;
  uint16_t http_port_;
  std::atomic_bool running_;
  std::thread tcp_thread_;
  std::thread http_thread_;
  std::mutex mutex_;
  std::map<uint8_t, std::vector<uint8_t>> responses_;
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> requests_;
};

class MockHesaiCalibration : public nebula::drivers::HesaiCalibrationConfigurationBase
{
public:
  nebula::Status load_from_bytes(const std::vector<uint8_t> &) override
  {
    return nebula::Status::OK;
  }
  nebula::Status load_from_file(const std::string &) override { return nebula::Status::OK; }
  nebula::Status save_to_file_from_bytes(const std::string &, const std::vector<uint8_t> &) override
  {
    return nebula::Status::OK;
  }
  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override { return {0.0f, 360.0f}; }
};

class TestHesaiHwInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    logger_ = std::make_shared<DummyLogger>();
    hw_interface_ = std::make_shared<nebula::drivers::HesaiHwInterface>(logger_);
    mock_sensor_ = std::make_unique<MockHesaiSensor>();

    auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
    config->sensor_model = nebula::drivers::SensorModel::HESAI_PANDARAT128;
    config->sensor_ip = "127.0.0.1";
    config->host_ip = "127.0.0.1";
    config->data_port = 2368;
    config->udp_socket_receive_buffer_size_bytes = 1024 * 1024;
    config->ptp_profile = nebula::drivers::PtpProfile::IEEE_802_1AS_AUTO;

    hw_interface_->set_sensor_configuration(config);
    hw_interface_->set_target_model(config->sensor_model);

    hw_interface_->set_target_ports(mock_sensor_->get_tcp_port(), mock_sensor_->get_http_port());
  }

  void TearDown() override
  {
    hw_interface_->finalize_tcp_socket();
    mock_sensor_.reset();
  }

  std::shared_ptr<DummyLogger> logger_;
  std::unique_ptr<MockHesaiSensor> mock_sensor_;
  std::shared_ptr<nebula::drivers::HesaiHwInterface> hw_interface_;
};

TEST_F(TestHesaiHwInterface, TestInitializeFinalize)
{
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->finalize_tcp_socket(), nebula::Status::OK);
}

TEST_F(TestHesaiHwInterface, TestReceivePTCCommands)
{
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  // Test set spin rate
  EXPECT_EQ(hw_interface_->set_spin_rate(600), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_sync_angle(10, 20), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_trigger_method(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_standby_mode(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_return_mode(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_destination_ip(192, 168, 1, 100, 2368, 10110), nebula::Status::OK);
  EXPECT_EQ(
    hw_interface_->set_control_port(192, 168, 1, 100, 255, 255, 255, 0, 192, 168, 1, 1, 0, 0),
    nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_high_resolution_mode(true), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_up_close_blockage_detection(true), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_clock_source(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_ptp_lock_offset(1), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->send_reset(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_rot_dir(1), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->get_lidar_calibration_string(), "mock_calibration_data");
  EXPECT_NE(hw_interface_->get_inventory(), nullptr);
  // Wait for transactions
  std::this_thread::sleep_for(200ms);
}

TEST_F(TestHesaiHwInterface, TestXT32Commands)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_PANDARXT32;  // Not AT128!
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->udp_socket_receive_buffer_size_bytes = 1024 * 1024;

  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->set_lidar_range(0, {0x00, 0x01, 0x00, 0x01}), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_lidar_range(40, 320), nebula::Status::OK);
  EXPECT_NO_THROW(hw_interface_->get_lidar_range());

  // Also verify XT32 parsing uses different code blocks
  EXPECT_NE(hw_interface_->get_inventory(), nullptr);
  EXPECT_NE(hw_interface_->get_config(), nullptr);
  EXPECT_NE(hw_interface_->get_lidar_status(), nullptr);
  EXPECT_NO_THROW(hw_interface_->get_lidar_monitor());
}

TEST_F(TestHesaiHwInterface, TestAT128PCommands)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_AT128P;
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->udp_socket_receive_buffer_size_bytes = 1024 * 1024;

  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  // The AT128P rejects the lidar range commands, like the AT128
  EXPECT_EQ(hw_interface_->set_lidar_range(40, 320), nebula::Status::SENSOR_CONFIG_ERROR);
  EXPECT_THROW(hw_interface_->get_lidar_range(), std::runtime_error);

  auto inventory = hw_interface_->get_inventory();
  ASSERT_NE(inventory, nullptr);
  EXPECT_EQ(inventory->model_number(), nebula::HesaiInventory_AT128P::g_model_number);
  EXPECT_NE(hw_interface_->get_config(), nullptr);
  EXPECT_NE(hw_interface_->get_lidar_status(), nullptr);

  // Unlike the AT128, the AT128P answers lidar monitor requests via PTC
  EXPECT_FALSE(hw_interface_->use_http_get_lidar_monitor());
  EXPECT_NO_THROW(hw_interface_->get_lidar_monitor());
}

TEST_F(TestHesaiHwInterface, TestFTX140Commands)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_FTX140;
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->udp_socket_receive_buffer_size_bytes = 1024 * 1024;

  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  // Lidar range is not supported
  EXPECT_EQ(hw_interface_->check_and_set_config(), nebula::HesaiStatus::OK);
  EXPECT_EQ(hw_interface_->set_lidar_range(40, 320), nebula::Status::SENSOR_CONFIG_ERROR);
  EXPECT_THROW(hw_interface_->get_lidar_range(), std::runtime_error);

  auto inventory = hw_interface_->get_inventory();
  ASSERT_NE(inventory, nullptr);
  EXPECT_EQ(inventory->model_number(), nebula::HesaiInventory_AT128P::g_model_number);
  EXPECT_NE(hw_interface_->get_config(), nullptr);

  // The FTX140 does not fill in the temperature fields of its status
  auto status = hw_interface_->get_lidar_status();
  ASSERT_NE(status, nullptr);
  auto status_json = status->to_json();
  EXPECT_FALSE(status_json.contains("temperature"));
  EXPECT_TRUE(status_json.contains("ptp_status"));
}

namespace
{

/// @brief Config info of an FTX140 with the defaults of the FTX TCP API (section 3.1.3), except
/// for the given return mode and destination
std::vector<uint8_t> make_ftx_config_info(
  uint8_t return_mode, const std::array<uint8_t, 4> & dest_ip, uint16_t dest_port)
{
  std::vector<uint8_t> payload(sizeof(nebula::HesaiConfig_FTX::Internal), 0);
  const std::array<uint8_t, 12> ip_mask_gateway{192, 168, 1, 201, 255, 255, 255, 0, 192, 168, 1, 1};
  std::copy(ip_mask_gateway.begin(), ip_mask_gateway.end(), payload.begin());
  std::copy(dest_ip.begin(), dest_ip.end(), payload.begin() + 12);
  payload[16] = dest_port >> 8;
  payload[17] = dest_port & 0xff;
  payload[29] = 1;  // clock source: PTP
  payload[30] = 1;  // UDP sequence: on
  payload[31] = 1;  // trigger method: time-based
  payload[32] = return_mode;
  payload[37] = 1;  // VLAN ID
  return payload;
}

/// @brief An FTX140 configuration that matches `make_ftx_config_info(3, {127, 0, 0, 1}, 2368)` and
/// a PTP config of {enabled, automotive, domain 0, L2, TSN}
std::shared_ptr<nebula::drivers::HesaiSensorConfiguration> make_ftx_sensor_configuration()
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_FTX140;
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->udp_socket_receive_buffer_size_bytes = 1024 * 1024;
  config->return_mode = nebula::drivers::ReturnMode::FIRST;
  config->ptp_profile = nebula::drivers::PtpProfile::IEEE_802_1AS_AUTO;
  config->ptp_domain = 0;
  config->ptp_transport_type = nebula::drivers::PtpTransportType::L2;
  config->ptp_switch_type = nebula::drivers::PtpSwitchType::TSN;
  return config;
}

const std::vector<uint8_t> g_ftx_ptp_config{1, 3, 0, 1, 1};

std::vector<std::pair<uint8_t, std::vector<uint8_t>>> set_requests(
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & requests)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> result;
  std::copy_if(requests.begin(), requests.end(), std::back_inserter(result), [](const auto & r) {
    return r.first == nebula::drivers::g_ptc_command_set_return_mode ||
           r.first == nebula::drivers::g_ptc_command_set_destination_ip ||
           r.first == nebula::drivers::g_ptc_command_set_ptp_config;
  });
  return result;
}

}  // namespace

TEST(TestHesaiConfigFTX, ParsesDocumentedLayout)
{
  using nebula::HesaiConfig_FTX;
  static_assert(sizeof(HesaiConfig_FTX::Internal) == 47);

  auto payload = make_ftx_config_info(3, {255, 255, 255, 255}, 2368);
  payload[22] = 1;     // frame time alignment: enabled
  payload[23] = 0xc3;  // frame header timestamp: 49999 (x 2 us)
  payload[24] = 0x4f;
  payload[35] = 1;  // VLAN: on
  payload[37] = 14;

  HesaiConfig_FTX::Internal raw{};
  std::memcpy(&raw, payload.data(), sizeof(raw));
  HesaiConfig_FTX config(raw);

  auto json = config.to_json();
  EXPECT_EQ(json["ipaddr"], "192.168.1.201");
  EXPECT_EQ(json["mask"], "255.255.255.0");
  EXPECT_EQ(json["gateway"], "192.168.1.1");
  EXPECT_EQ(json["dest_ipaddr"], "255.255.255.255");
  EXPECT_EQ(json["dest_LiDAR_udp_port"], 2368);
  EXPECT_EQ(json["alignment_enable"], 1);
  EXPECT_EQ(json["frame_header_timestamp"], "99998 us");
  EXPECT_EQ(json["clock_source"], 1);
  EXPECT_EQ(json["return_mode"], 3);
  EXPECT_EQ(json["vlan_flag"], 1);
  EXPECT_EQ(json["vlan_id"], 14);
}

TEST_F(TestHesaiHwInterface, TestFTX140SetsDifferingConfig)
{
  auto config = make_ftx_sensor_configuration();
  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);

  // Return mode Last, broadcast destination, PTP over UDP with a non-TSN switch
  mock_sensor_->set_response(
    nebula::drivers::g_ptc_command_get_config_info,
    make_ftx_config_info(0, {255, 255, 255, 255}, 2368));
  mock_sensor_->set_response(nebula::drivers::g_ptc_command_get_ptp_config, {1, 3, 0, 0, 0});
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->check_and_set_config(), nebula::HesaiStatus::OK);

  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> expected{
    {nebula::drivers::g_ptc_command_set_return_mode, {3}},
    // 127.0.0.1:2368, GPS port 0
    {nebula::drivers::g_ptc_command_set_destination_ip, {127, 0, 0, 1, 0x09, 0x40, 0, 0}},
    // Enabled, automotive, domain 0, L2, TSN
    {nebula::drivers::g_ptc_command_set_ptp_config, g_ftx_ptp_config},
  };
  EXPECT_EQ(set_requests(mock_sensor_->get_requests()), expected);
}

TEST_F(TestHesaiHwInterface, TestFTX140KeepsMatchingConfig)
{
  auto config = make_ftx_sensor_configuration();
  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);

  mock_sensor_->set_response(
    nebula::drivers::g_ptc_command_get_config_info, make_ftx_config_info(3, {127, 0, 0, 1}, 2368));
  mock_sensor_->set_response(nebula::drivers::g_ptc_command_get_ptp_config, g_ftx_ptp_config);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->check_and_set_config(), nebula::HesaiStatus::OK);
  EXPECT_TRUE(set_requests(mock_sensor_->get_requests()).empty());
}

TEST(TestHesaiInventoryAT128P, ParsesPartNumberLayout)
{
  using nebula::HesaiInventory_AT128P;
  // Size of the AT128P inventory response
  static_assert(sizeof(HesaiInventory_AT128P::Internal) == 216);

  HesaiInventory_AT128P::Internal raw{};
  const std::string sn = "AT128P-TEST-SN-01";
  const std::string pn = "AT128P-TEST-PN";
  const std::string date = "2000-01-01";
  const std::string sw_ver = "1.2.3";
  std::copy(sn.begin(), sn.end(), raw.sn);
  std::copy(pn.begin(), pn.end(), raw.pn);
  std::copy(date.begin(), date.end(), raw.date_of_manufacture);
  std::copy(sw_ver.begin(), sw_ver.end(), raw.sw_ver);
  raw.product_model = 0;  // the AT128P reports 0 here
  raw.num_of_lines = 128;

  HesaiInventory_AT128P inventory(raw);
  EXPECT_EQ(inventory.model_number(), HesaiInventory_AT128P::g_model_number);

  auto json = inventory.to_json();
  EXPECT_EQ(json["sn"], sn);
  EXPECT_EQ(json["date_of_manufacture"], date);
  EXPECT_EQ(json["sw_ver"], sw_ver);
  EXPECT_EQ(json["pn"], pn);
  EXPECT_EQ(json["num_of_lines"], 128);
}

TEST_F(TestHesaiHwInterface, TestHTTPMethods)
{
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  // set_spin_speed_async_http is a Hesai AT128 / new style HTTP command
  EXPECT_EQ(hw_interface_->set_spin_speed_async_http(600), nebula::HesaiStatus::OK);
  EXPECT_EQ(hw_interface_->set_ptp_config_sync_http(1, 0, 0, 1, 1, 0), nebula::HesaiStatus::OK);
  EXPECT_EQ(hw_interface_->set_sync_angle_sync_http(1, 90), nebula::HesaiStatus::OK);

  // get_lidar_monitor_async_http
  std::atomic_bool callback_triggered{false};
  hw_interface_->get_lidar_monitor_async_http([&](const std::string & response) {
    callback_triggered = true;
    EXPECT_TRUE(response.find("uptime") != std::string::npos);
  });

  for (int i = 0; i < 20; ++i) {
    if (callback_triggered) break;
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(callback_triggered);
}

TEST_F(TestHesaiHwInterface, TestSetPtpConfig)
{
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_ptp_config(1, 0, 0, 0, 1, 1, 0), nebula::Status::OK);
}

TEST_F(TestHesaiHwInterface, TestAllRemainingCommands)
{
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  // Test various setters
  EXPECT_NE(hw_interface_->get_lidar_calibration_string(), "");
  EXPECT_EQ(hw_interface_->set_spin_rate(1200), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_return_mode(2), nebula::Status::OK);
  EXPECT_EQ(hw_interface_->set_up_close_blockage_detection(false), nebula::Status::OK);

  // Read struct diagnostics
  EXPECT_NE(hw_interface_->get_inventory(), nullptr);
  EXPECT_NE(hw_interface_->get_config(), nullptr);
  EXPECT_NE(hw_interface_->get_lidar_status(), nullptr);
  EXPECT_NE(hw_interface_->get_ptp_config(), nullptr);

  hw_interface_->get_ptp_diag_status();
  hw_interface_->get_ptp_diag_port();
  hw_interface_->get_ptp_diag_time();
  hw_interface_->get_ptp_diag_grandmaster();
  EXPECT_THROW(hw_interface_->get_lidar_monitor(), std::runtime_error);

  // Test failure paths (socket closed mid-operation)
  mock_sensor_.reset();
  EXPECT_THROW(hw_interface_->set_spin_rate(600), std::runtime_error);
  EXPECT_THROW(hw_interface_->get_inventory(), std::runtime_error);
  EXPECT_THROW(hw_interface_->get_config(), std::runtime_error);
  EXPECT_THROW(hw_interface_->get_lidar_status(), std::runtime_error);
}

TEST_F(TestHesaiHwInterface, TestCheckAndSetConfig)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_PANDARXT32;
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->ptp_profile = nebula::drivers::PtpProfile::IEEE_802_1AS_AUTO;
  config->return_mode = nebula::drivers::ReturnMode::DUAL_ONLY;
  config->rotation_speed = 600;
  config->sync_angle = 180;

  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->check_and_set_config(), nebula::HesaiStatus::OK);
}

TEST_F(TestHesaiHwInterface, TestCheckAndSetConfigHTTP)
{
  // AT128 calls HTTP APIs in check_and_set_config
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_PANDARAT128;
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  config->ptp_profile = nebula::drivers::PtpProfile::IEEE_802_1AS_AUTO;
  config->return_mode = nebula::drivers::ReturnMode::DUAL_ONLY;
  config->rotation_speed = 600;
  config->sync_angle = 180;

  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  EXPECT_EQ(hw_interface_->check_and_set_config(), nebula::HesaiStatus::OK);
}

TEST_F(TestHesaiHwInterface, TestCheckAndSetLidarRange)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model =
    nebula::drivers::SensorModel::HESAI_PANDARXT32;  // XT32 uses set_lidar_range
  config->sensor_ip = "127.0.0.1";
  config->host_ip = "127.0.0.1";
  config->data_port = 2368;
  hw_interface_->set_sensor_configuration(config);
  hw_interface_->set_target_model(config->sensor_model);
  EXPECT_EQ(hw_interface_->initialize_tcp_socket(), nebula::Status::OK);

  MockHesaiCalibration calib;
  // This will call the getter, calculate FoV, and then call set_lidar_range
  EXPECT_EQ(hw_interface_->check_and_set_lidar_range(calib), nebula::Status::OK);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
