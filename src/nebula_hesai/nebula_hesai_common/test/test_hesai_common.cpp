// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebula_hesai_common/hesai_common.hpp"

#include <gtest/gtest.h>

#include <array>
#include <initializer_list>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

namespace
{

using nebula::drivers::AdvancedFunctionalSafetyConfiguration;
using nebula::drivers::HesaiSensorConfiguration;
using nebula::drivers::PtpProfile;
using nebula::drivers::PtpSwitchType;
using nebula::drivers::PtpTransportType;
using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;

template <typename T>
std::string stream_to_string(const T & value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

void expect_contains_all(
  const std::string & output, std::initializer_list<std::string_view> expected_substrings)
{
  for (const auto expected_substring : expected_substrings) {
    EXPECT_NE(output.find(expected_substring), std::string::npos) << output;
  }
}

void expect_hesai_return_mode_round_trip(
  const SensorModel sensor_model,
  std::initializer_list<std::tuple<std::string_view, int, ReturnMode>> expected_values)
{
  for (const auto & [string_value, int_value, expected_mode] : expected_values) {
    EXPECT_EQ(
      nebula::drivers::return_mode_from_string_hesai(std::string(string_value), sensor_model),
      expected_mode);
    EXPECT_EQ(nebula::drivers::return_mode_from_int_hesai(int_value, sensor_model), expected_mode);
    EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(expected_mode, sensor_model), int_value);
  }
}

HesaiSensorConfiguration make_hesai_sensor_configuration(const SensorModel sensor_model)
{
  HesaiSensorConfiguration configuration{};
  configuration.sensor_model = sensor_model;
  configuration.frame_id = "hesai_frame";
  configuration.host_ip = "192.168.1.10";
  configuration.sensor_ip = "192.168.1.201";
  configuration.data_port = 2368;
  configuration.return_mode = ReturnMode::DUAL_LAST_STRONGEST;
  configuration.packet_mtu_size = 1200;
  configuration.min_range = 1.0;
  configuration.max_range = 200.0;
  configuration.use_sensor_time = true;
  configuration.multicast_ip = "239.1.2.3";
  configuration.gnss_port = 10110;
  configuration.udp_socket_receive_buffer_size_bytes = 4096;
  configuration.sync_angle = 123;
  configuration.cut_angle = 45.5;
  configuration.dual_return_distance_threshold = 0.75;
  configuration.calibration_path = "/tmp/hesai.csv";
  configuration.calibration_download_enabled = true;
  configuration.rotation_speed = 1200;
  configuration.cloud_min_angle = 100;
  configuration.cloud_max_angle = 200;
  configuration.ptp_profile = PtpProfile::IEEE_802_1AS_AUTO;
  configuration.ptp_domain = 7;
  configuration.ptp_transport_type = PtpTransportType::UDP_IP;
  configuration.ptp_switch_type = PtpSwitchType::TSN;
  configuration.ptp_lock_threshold = 4;
  configuration.downsample_mask_path = "/tmp/downsample.csv";
  configuration.hires_mode = true;
  configuration.blockage_mask_horizontal_bin_size_mdeg = 250;
  configuration.sync_diagnostics_topic = "/diagnostics/ptp";
  return configuration;
}

TEST(HesaiCommonTest, PtpProfilesParseCaseInsensitivelyAndStreamExpectedValues)
{
  constexpr std::array<std::tuple<std::string_view, PtpProfile, std::string_view>, 3>
    expected_values{{
      {"1588V2", PtpProfile::IEEE_1588v2, "IEEE_1588v2"},
      {"802.1AS", PtpProfile::IEEE_802_1AS, "IEEE_802.1AS"},
      {"AUTOMOTIVE", PtpProfile::IEEE_802_1AS_AUTO, "IEEE_802.1AS Automotive"},
    }};

  for (const auto & [string_value, expected_profile, streamed_value] : expected_values) {
    EXPECT_EQ(nebula::drivers::ptp_profile_from_string(std::string(string_value)), expected_profile);
    EXPECT_EQ(stream_to_string(expected_profile), streamed_value);
  }

  EXPECT_EQ(
    nebula::drivers::ptp_profile_from_string("unsupported"), PtpProfile::UNKNOWN_PROFILE);
  EXPECT_EQ(stream_to_string(PtpProfile::UNKNOWN_PROFILE), "UNKNOWN");
}

TEST(HesaiCommonTest, PtpTransportTypesParseCaseInsensitivelyAndStreamExpectedValues)
{
  constexpr std::array<std::tuple<std::string_view, PtpTransportType, std::string_view>, 2>
    expected_values{{
      {"UDP", PtpTransportType::UDP_IP, "UDP/IP"},
      {"l2", PtpTransportType::L2, "L2"},
    }};

  for (const auto & [string_value, expected_transport, streamed_value] : expected_values) {
    EXPECT_EQ(
      nebula::drivers::ptp_transport_type_from_string(std::string(string_value)),
      expected_transport);
    EXPECT_EQ(stream_to_string(expected_transport), streamed_value);
  }

  EXPECT_EQ(
    nebula::drivers::ptp_transport_type_from_string("unsupported"),
    PtpTransportType::UNKNOWN_TRANSPORT);
  EXPECT_EQ(stream_to_string(PtpTransportType::UNKNOWN_TRANSPORT), "UNKNOWN");
}

TEST(HesaiCommonTest, PtpSwitchTypesParseCaseInsensitivelyAndStreamExpectedValues)
{
  constexpr std::array<std::tuple<std::string_view, PtpSwitchType, std::string_view>, 2>
    expected_values{{
      {"TSN", PtpSwitchType::TSN, "TSN"},
      {"non_tsn", PtpSwitchType::NON_TSN, "NON_TSN"},
    }};

  for (const auto & [string_value, expected_switch, streamed_value] : expected_values) {
    EXPECT_EQ(
      nebula::drivers::ptp_switch_type_from_string(std::string(string_value)), expected_switch);
    EXPECT_EQ(stream_to_string(expected_switch), streamed_value);
  }

  EXPECT_EQ(
    nebula::drivers::ptp_switch_type_from_string("unsupported"), PtpSwitchType::UNKNOWN_SWITCH);
  EXPECT_EQ(stream_to_string(PtpSwitchType::UNKNOWN_SWITCH), "UNKNOWN");
}

TEST(HesaiCommonTest, ReturnModeConversionsRoundTripForXtFamily)
{
  constexpr SensorModel sensor_model = SensorModel::HESAI_PANDARXT32;

  expect_hesai_return_mode_round_trip(
    sensor_model,
    {{"Last", 0, ReturnMode::LAST},
     {"Strongest", 1, ReturnMode::STRONGEST},
     {"Dual", 2, ReturnMode::DUAL_LAST_STRONGEST},
     {"First", 3, ReturnMode::FIRST},
     {"LastFirst", 4, ReturnMode::DUAL_LAST_FIRST},
     {"FirstStrongest", 5, ReturnMode::DUAL_FIRST_STRONGEST}});

  EXPECT_EQ(
    nebula::drivers::return_mode_from_string_hesai("LastStrongest", sensor_model),
    ReturnMode::DUAL_LAST_STRONGEST);
  EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(ReturnMode::DUAL, sensor_model), 2);
}

TEST(HesaiCommonTest, ReturnModeConversionsRoundTripForQt64Family)
{
  constexpr SensorModel sensor_model = SensorModel::HESAI_PANDARQT64;

  expect_hesai_return_mode_round_trip(
    sensor_model,
    {{"Last", 0, ReturnMode::LAST},
     {"Dual", 2, ReturnMode::DUAL_LAST_FIRST},
     {"First", 3, ReturnMode::FIRST}});

  EXPECT_EQ(
    nebula::drivers::return_mode_from_string_hesai("LastFirst", sensor_model),
    ReturnMode::DUAL_LAST_FIRST);
  EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(ReturnMode::DUAL, sensor_model), 2);
}

TEST(HesaiCommonTest, ReturnModeConversionsRoundTripForLegacyDualFamily)
{
  constexpr SensorModel sensor_model = SensorModel::HESAI_PANDARAT128;

  expect_hesai_return_mode_round_trip(
    sensor_model,
    {{"Last", 0, ReturnMode::LAST},
     {"Strongest", 1, ReturnMode::STRONGEST},
     {"Dual", 2, ReturnMode::DUAL_LAST_STRONGEST}});

  EXPECT_EQ(
    nebula::drivers::return_mode_from_string_hesai("LastStrongest", sensor_model),
    ReturnMode::DUAL_LAST_STRONGEST);
  EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(ReturnMode::DUAL, sensor_model), 2);
}

TEST(HesaiCommonTest, ReturnModeConversionsHandleUnexpectedValues)
{
  constexpr SensorModel xt_sensor_model = SensorModel::HESAI_PANDARXT16;
  constexpr SensorModel at_sensor_model = SensorModel::HESAI_PANDAR64;

  EXPECT_EQ(nebula::drivers::return_mode_from_string_hesai("last", xt_sensor_model), ReturnMode::UNKNOWN);
  EXPECT_EQ(nebula::drivers::return_mode_from_int_hesai(99, xt_sensor_model), ReturnMode::UNKNOWN);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_hesai("Strongest", SensorModel::HESAI_PANDARQT64), ReturnMode::UNKNOWN);
  EXPECT_EQ(nebula::drivers::return_mode_from_int_hesai(1, SensorModel::HESAI_PANDARQT64), ReturnMode::UNKNOWN);
  EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(ReturnMode::UNKNOWN, xt_sensor_model), -1);
  EXPECT_EQ(nebula::drivers::int_from_return_mode_hesai(ReturnMode::FIRST, at_sensor_model), -1);
}

TEST(HesaiCommonTest, ReturnModeConversionsRejectUnsupportedSensorModels)
{
  constexpr SensorModel unsupported_model = SensorModel::VELODYNE_VLP16;

  EXPECT_THROW(
    nebula::drivers::return_mode_from_string_hesai("Last", unsupported_model), std::runtime_error);
  EXPECT_THROW(nebula::drivers::return_mode_from_int_hesai(0, unsupported_model), std::runtime_error);
  EXPECT_THROW(
    nebula::drivers::int_from_return_mode_hesai(ReturnMode::LAST, unsupported_model),
    std::runtime_error);
}

TEST(HesaiCommonTest, SupportPredicatesReflectVendorCapabilities)
{
  EXPECT_FALSE(nebula::drivers::supports_lidar_monitor(SensorModel::HESAI_PANDARAT128));
  EXPECT_FALSE(nebula::drivers::supports_lidar_monitor(SensorModel::HESAI_PANDAR40P));
  EXPECT_TRUE(nebula::drivers::supports_lidar_monitor(SensorModel::HESAI_PANDARXT32));

  EXPECT_TRUE(nebula::drivers::supports_functional_safety(SensorModel::HESAI_PANDAR128_E3X));
  EXPECT_TRUE(nebula::drivers::supports_functional_safety(SensorModel::HESAI_PANDAR128_E4X));
  EXPECT_TRUE(nebula::drivers::supports_functional_safety(SensorModel::HESAI_PANDARQT128));
  EXPECT_FALSE(nebula::drivers::supports_functional_safety(SensorModel::HESAI_PANDARXT32));

  EXPECT_TRUE(nebula::drivers::supports_packet_loss_detection(SensorModel::HESAI_PANDAR40M));
  EXPECT_TRUE(nebula::drivers::supports_packet_loss_detection(SensorModel::HESAI_PANDAR128_E4X));
  EXPECT_FALSE(nebula::drivers::supports_packet_loss_detection(SensorModel::HESAI_PANDARXT16));

  EXPECT_TRUE(nebula::drivers::supports_blockage_mask(SensorModel::HESAI_PANDAR128_E4X));
  EXPECT_FALSE(nebula::drivers::supports_blockage_mask(SensorModel::HESAI_PANDAR128_E3X));
}

TEST(HesaiCommonTest, AdvancedFunctionalSafetyConfigurationStreamingReflectsConfiguredValues)
{
  const AdvancedFunctionalSafetyConfiguration advanced{
    "/tmp/error-definitions.json", {0x1a, 0x2b}};
  const AdvancedFunctionalSafetyConfiguration basic{"/tmp/error-definitions.json", {}};

  expect_contains_all(
    stream_to_string(advanced), {"advanced", "/tmp/error-definitions.json", "0x1a, 0x2b"});
  expect_contains_all(stream_to_string(basic), {"advanced", "/tmp/error-definitions.json", "none"});
}

TEST(HesaiCommonTest, HesaiSensorConfigurationStreamingReflectsConfiguredValues)
{
  HesaiSensorConfiguration configuration =
    make_hesai_sensor_configuration(SensorModel::HESAI_PANDAR128_E4X);
  configuration.functional_safety =
    AdvancedFunctionalSafetyConfiguration{"/tmp/error-definitions.json", {0x1a, 0x2b}};

  const std::string output = stream_to_string(configuration);

  expect_contains_all(
    output,
    {"Hesai Sensor Configuration:",
     "Sensor Model: Pandar128_E4X_OT",
     "Frame ID: hesai_frame",
     "Host IP: 192.168.1.10",
     "Sensor IP: 192.168.1.201",
     "Data Port: 2368",
     "Return Mode: LastStrongest",
     "MTU: 1200",
     "Use Sensor Time: 1",
     "Multicast: enabled, group: 239.1.2.3",
     "GNSS Port: 10110",
     "UDP Socket Receive Buffer Size: 4096 B",
     "Rotation Speed: 1200",
     "Sync Angle: 123",
     "Cut Angle: 45.5",
     "FoV Start: 100",
     "FoV End: 200",
     "Dual Return Distance Threshold: 0.75",
     "Calibration Path: /tmp/hesai.csv",
     "Calibration Download: enabled",
     "PTP Profile: IEEE_802.1AS Automotive",
     "PTP Domain: 7",
     "PTP Transport Type: UDP/IP",
     "PTP Switch Type: TSN",
     "PTP Lock Threshold: 4",
     "High Resolution Mode: enabled",
     "Downsample Filter: enabled, path: /tmp/downsample.csv",
     "Blockage Mask Output: enabled, horizontal bin size: 250 mdeg",
     "Synchronization Diagnostics: enabled, topic: /diagnostics/ptp",
     "Functional Safety: advanced",
     "/tmp/error-definitions.json",
     "0x1a, 0x2b"});
}

TEST(HesaiCommonTest, HesaiSensorConfigurationStreamingHandlesDisabledOptionals)
{
  HesaiSensorConfiguration configuration =
    make_hesai_sensor_configuration(SensorModel::HESAI_PANDARXT32);
  configuration.multicast_ip.clear();
  configuration.calibration_download_enabled = false;
  configuration.hires_mode = false;
  configuration.downsample_mask_path.reset();
  configuration.blockage_mask_horizontal_bin_size_mdeg.reset();
  configuration.sync_diagnostics_topic.reset();
  configuration.functional_safety =
    AdvancedFunctionalSafetyConfiguration{"/tmp/error-definitions.json", {0x1a}};

  const std::string output = stream_to_string(configuration);

  expect_contains_all(
    output,
    {"Multicast: disabled",
     "Calibration Download: disabled",
     "High Resolution Mode: disabled",
     "Downsample Filter: disabled",
     "Blockage Mask Output: disabled",
     "Synchronization Diagnostics: disabled"});
  EXPECT_EQ(output.find("Functional Safety:"), std::string::npos) << output;
}

TEST(HesaiCommonTest, HesaiSensorConfigurationStreamingFallsBackToBasicFunctionalSafety)
{
  HesaiSensorConfiguration configuration =
    make_hesai_sensor_configuration(SensorModel::HESAI_PANDARQT128);
  configuration.functional_safety = std::nullopt;

  const std::string output = stream_to_string(configuration);

  EXPECT_NE(output.find("Functional Safety: basic"), std::string::npos) << output;
}

}  // namespace
