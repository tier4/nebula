// Copyright 2024 TIER IV, Inc.

#include "hesai/test_ptc/ptc_test.hpp"
#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"

#include <gmock/gmock-cardinalities.h>
#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

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
    auto tcp_sock_ptr = make_mock_tcp_socket();
    auto & tcp_sock = *tcp_sock_ptr;

    EXPECT_CALL(tcp_sock, close()).Times(AtLeast(1));
    auto hw_interface = make_hw_interface(tcp_sock_ptr);
  }

  /* Full lifecycle without sending/receiving */ {
    auto tcp_sock_ptr = make_mock_tcp_socket();
    auto & tcp_sock = *tcp_sock_ptr;

    InSequence seq;
    EXPECT_CALL(tcp_sock, init(g_host_ip, _, g_sensor_ip, g_ptc_port)).Times(Exactly(1));
    EXPECT_CALL(tcp_sock, bind()).Times(Exactly(1));
    EXPECT_CALL(tcp_sock, close()).Times(AtLeast(1));

    auto cfg = make_sensor_config(GetParam());

    auto hw_interface = make_hw_interface(tcp_sock_ptr);
    hw_interface->SetSensorConfiguration(cfg);
    hw_interface->InitializeTcpDriver();
    hw_interface->FinalizeTcpDriver();
  }
}

INSTANTIATE_TEST_SUITE_P(TestMain, PtcTest, testing::ValuesIn(g_models_under_test));

}  // namespace nebula::drivers

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
};
