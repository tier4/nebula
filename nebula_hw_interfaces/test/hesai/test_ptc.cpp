// Copyright 2024 TIER IV, Inc.

#include "hesai/test_ptc/ptc_test.hpp"
#include "hesai/test_ptc/tcp_socket_mock.hpp"
#include "hesai/test_ptc/tcp_socket_replay.hpp"
#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/tcp.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"

#include <gmock/gmock-cardinalities.h>
#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>

#ifndef _TEST_RESOURCES_PATH
static_assert(false, "No test resources path defined");
#endif

#define GTEST_SKIP_PRINT(x) \
  do {                      \
    std::cout << x << '\n'; \
    GTEST_SKIP() << x;      \
  } while (0)

#define ASSERT_NO_THROW_PRINT(expr)      \
  do {                                   \
    try {                                \
      expr;                              \
    } catch (const std::exception & e) { \
      std::cout << e.what() << '\n';     \
      ASSERT_NO_THROW(throw e);          \
    }                                    \
  } while (0)

namespace nebula::drivers
{

using testing::_;
using testing::AtLeast;
using testing::Exactly;
using testing::InSequence;

const SensorModel g_models_under_test[] = {
  SensorModel::HESAI_PANDAR64,      SensorModel::HESAI_PANDAR40P,  SensorModel::HESAI_PANDARQT64,
  SensorModel::HESAI_PANDARQT128,   SensorModel::HESAI_PANDARXT32, SensorModel::HESAI_PANDARAT128,
  SensorModel::HESAI_PANDAR128_E4X,
};

const uint16_t g_u16_invalid = 0x4242;
const uint16_t g_ptc_port = 9347;
const size_t g_ptc_header_size = 8;
const char g_host_ip[] = "192.168.42.42";
const char g_sensor_ip[] = "192.168.84.84";

auto make_sensor_config(SensorModel model)
{
  uint16_t rotation_speed = 600;
  uint16_t sync_angle = 0;
  double cut_angle = 0.0;
  uint16_t cloud_min_angle = 0;
  uint16_t cloud_max_angle = 360;

  if (model == SensorModel::HESAI_PANDARAT128) {
    rotation_speed = 200;
    sync_angle = 30;
    cut_angle = 150.0;
    cloud_min_angle = 30;
    cloud_max_angle = 150;
  }

  HesaiSensorConfiguration config{
    LidarConfigurationBase{
      EthernetSensorConfigurationBase{
        SensorConfigurationBase{model, "test"}, g_host_ip, g_sensor_ip, g_u16_invalid},
      ReturnMode::UNKNOWN,
      g_u16_invalid,
      g_u16_invalid,
      CoordinateMode::UNKNOWN,
      NAN,
      NAN,
      false,
      {},
      false},
    "",
    g_u16_invalid,
    sync_angle,
    cut_angle,
    0.1,
    "",
    rotation_speed,
    cloud_min_angle,
    cloud_max_angle,
    PtpProfile::IEEE_802_1AS_AUTO,
    0,
    PtpTransportType::L2,
    PtpSwitchType::NON_TSN};

  return std::make_shared<HesaiSensorConfiguration>(config);
}

TEST_P(PtcTest, ConnectionLifecycle)
{
  /* Constructor does not immediately connect, destructor closes socket */ {
    auto tcp_sock_ptr = std::make_shared<connections::MockTcpSocket>();
    auto & tcp_sock = *tcp_sock_ptr;

    EXPECT_CALL(tcp_sock, close()).Times(AtLeast(1));
    auto hw_interface = make_hw_interface(tcp_sock_ptr);
  }

  /* Full lifecycle without sending/receiving */ {
    auto tcp_sock_ptr = std::make_shared<connections::MockTcpSocket>();
    auto & tcp_sock = *tcp_sock_ptr;

    InSequence seq;
    EXPECT_CALL(tcp_sock, init(g_host_ip, _, g_sensor_ip, g_ptc_port)).Times(Exactly(1));
    EXPECT_CALL(tcp_sock, bind()).Times(Exactly(1));
    EXPECT_CALL(tcp_sock, close()).Times(AtLeast(1));

    auto cfg = make_sensor_config(GetParam());

    auto hw_interface = make_hw_interface(tcp_sock_ptr);
    hw_interface->set_sensor_configuration(cfg);
    hw_interface->initialize_tcp_driver();
    hw_interface->finalize_tcp_driver();
  }
}

TEST_P(PtcTest, PtcCommunication)
{
  const auto & model = GetParam();

  // ////////////////////////////////////////
  // Set up database-based replay TCP socket
  // ////////////////////////////////////////

  using ptc_handler_t = connections::ReplayTcpSocket::ptc_handler_t;
  using header_callback_t = connections::ReplayTcpSocket::header_callback_t;
  using payload_callback_t = connections::ReplayTcpSocket::payload_callback_t;
  using completion_callback_t = connections::ReplayTcpSocket::completion_callback_t;
  using connections::message_t;

  auto conversation_db_path = std::filesystem::path(_TEST_RESOURCES_PATH) / "hesai" /
                              (sensor_model_to_string(model) + ".json");
  if (!std::filesystem::exists(conversation_db_path)) {
    GTEST_SKIP_PRINT("conversation DB " << conversation_db_path << " does not exist");
  }

  auto conversation_db_exp = connections::parse_conversation_db(conversation_db_path);
  if (!conversation_db_exp.has_value()) {
    std::cout << "ParseError: " << conversation_db_exp.error().what() << '\n';
    GTEST_SKIP_PRINT(
      "conversation DB for model "
      << model << " could not be parsed due to ParseError: " << conversation_db_exp.error().what());
  }

  connections::conversation_db_t conversation_db = conversation_db_exp.value();

  ptc_handler_t cb = [&conversation_db](
                       const message_t & request, const header_callback_t & cb_header,
                       const payload_callback_t & cb_payload,
                       const completion_callback_t & cb_completion) {
    if (conversation_db.find(request) == conversation_db.end()) {
      std::stringstream ss;
      ss << "0x";
      for (const uint8_t & byte : request) {
        ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
      }
      GTEST_SKIP_PRINT("request " << ss.str() << " not found in conversation DB");
    }

    const auto & responses = conversation_db[request];

    message_t complete_response;

    for (const message_t & response : responses) {
      complete_response.insert(complete_response.end(), response.cbegin(), response.cend());
    }

    auto header = message_t(
      complete_response.cbegin(), std::next(complete_response.cbegin(), g_ptc_header_size));
    cb_header(header);
    cb_payload(complete_response);
    cb_completion();
  };

  auto tcp_sock_ptr = std::make_shared<connections::ReplayTcpSocket>(std::move(cb));

  // ////////////////////////////////////////
  // Test HW interface
  // ////////////////////////////////////////

  auto hw_interface = make_hw_interface(tcp_sock_ptr);

  auto cfg = make_sensor_config(GetParam());
  hw_interface->set_sensor_configuration(cfg);
  hw_interface->initialize_tcp_driver();

  // ////////////////////////////////////////
  // Applicable to all models
  // ////////////////////////////////////////

  std::shared_ptr<HesaiConfigBase> config;
  ASSERT_NO_THROW_PRINT(config = hw_interface->get_config());
  ASSERT_NE(config, nullptr);

  std::shared_ptr<HesaiInventoryBase> inventory;
  ASSERT_NO_THROW_PRINT(inventory = hw_interface->get_inventory());
  ASSERT_NE(inventory, nullptr);

  std::vector<uint8_t> calibration;
  ASSERT_NO_THROW_PRINT(calibration = hw_interface->get_lidar_calibration_bytes());
  ASSERT_FALSE(calibration.empty());

  std::shared_ptr<HesaiLidarStatusBase> status;
  ASSERT_NO_THROW_PRINT(status = hw_interface->get_lidar_status());
  ASSERT_NE(status, nullptr);
}

INSTANTIATE_TEST_SUITE_P(TestMain, PtcTest, testing::ValuesIn(g_models_under_test));

}  // namespace nebula::drivers

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
};
