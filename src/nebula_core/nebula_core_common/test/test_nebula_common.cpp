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

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace
{

using nebula::Status;
using nebula::drivers::CANSensorConfigurationBase;
using nebula::drivers::EthernetSensorConfigurationBase;
using nebula::drivers::LidarConfigurationBase;
using nebula::drivers::PointCloud;
using nebula::drivers::PointXYZ;
using nebula::drivers::PointXYZIR;
using nebula::drivers::PointXYZIRADT;
using nebula::drivers::PointXYZIRCAEDT;
using nebula::drivers::ReturnMode;
using nebula::drivers::ReturnType;
using nebula::drivers::SensorConfigurationBase;
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

TEST(NebulaCommonTest, ReturnTypeStreamingCoversAllEnumerators)
{
  constexpr std::array<std::pair<ReturnType, std::string_view>, 11> expected_values{{
    {ReturnType::UNKNOWN, "Unknown"},
    {ReturnType::LAST, "Last"},
    {ReturnType::FIRST, "First"},
    {ReturnType::STRONGEST, "Strongest"},
    {ReturnType::FIRST_WEAK, "FirstWeak"},
    {ReturnType::LAST_WEAK, "LastWeak"},
    {ReturnType::IDENTICAL, "Identical"},
    {ReturnType::SECOND, "Second"},
    {ReturnType::SECONDSTRONGEST, "SecondStrongest"},
    {ReturnType::FIRST_STRONGEST, "FirstStrongest"},
    {ReturnType::LAST_STRONGEST, "LastStrongest"},
  }};

  for (const auto & [value, expected_string] : expected_values) {
    EXPECT_EQ(stream_to_string(value), expected_string);
  }
}

TEST(NebulaCommonTest, ReturnModeStreamingAndParsingCoversSupportedValues)
{
  constexpr std::array<std::pair<ReturnMode, std::string_view>, 19> expected_values{{
    {ReturnMode::SINGLE_FIRST, "SingleFirst"},
    {ReturnMode::SINGLE_STRONGEST, "SingleStrongest"},
    {ReturnMode::SINGLE_LAST, "SingleLast"},
    {ReturnMode::DUAL_ONLY, "Dual"},
    {ReturnMode::DUAL_FIRST, "DualFirst"},
    {ReturnMode::DUAL_LAST, "DualLast"},
    {ReturnMode::DUAL_WEAK_FIRST, "WeakFirst"},
    {ReturnMode::DUAL_WEAK_LAST, "WeakLast"},
    {ReturnMode::DUAL_STRONGEST_LAST, "StrongLast"},
    {ReturnMode::DUAL_STRONGEST_FIRST, "StrongFirst"},
    {ReturnMode::TRIPLE, "Triple"},
    {ReturnMode::LAST, "Last"},
    {ReturnMode::STRONGEST, "Strongest"},
    {ReturnMode::DUAL_LAST_STRONGEST, "LastStrongest"},
    {ReturnMode::FIRST, "First"},
    {ReturnMode::DUAL_LAST_FIRST, "LastFirst"},
    {ReturnMode::DUAL_FIRST_STRONGEST, "FirstStrongest"},
    {ReturnMode::DUAL, "Dual"},
    {ReturnMode::UNKNOWN, "Unknown"},
  }};

  for (const auto & [value, expected_string] : expected_values) {
    EXPECT_EQ(stream_to_string(value), expected_string);
  }

  constexpr std::array<std::pair<std::string_view, ReturnMode>, 4> supported_round_trips{{
    {"SingleFirst", ReturnMode::SINGLE_FIRST},
    {"SingleStrongest", ReturnMode::SINGLE_STRONGEST},
    {"SingleLast", ReturnMode::SINGLE_LAST},
    {"Dual", ReturnMode::DUAL_ONLY},
  }};

  for (const auto & [string_value, expected_mode] : supported_round_trips) {
    EXPECT_EQ(nebula::drivers::return_mode_from_string(std::string(string_value)), expected_mode);
    EXPECT_EQ(
      nebula::drivers::return_mode_from_string(stream_to_string(expected_mode)), expected_mode);
  }

  EXPECT_EQ(nebula::drivers::return_mode_from_string("NotAReturnMode"), ReturnMode::UNKNOWN);
}

TEST(NebulaCommonTest, SensorModelStreamingAndStringConversionsRoundTrip)
{
  constexpr std::array<std::pair<SensorModel, std::string_view>, 23> streamed_values{{
    {SensorModel::HESAI_PANDAR64, "Pandar64"},
    {SensorModel::HESAI_PANDAR40P, "Pandar40P"},
    {SensorModel::HESAI_PANDAR40M, "Pandar40M"},
    {SensorModel::HESAI_PANDARQT64, "PandarQT64"},
    {SensorModel::HESAI_PANDARQT128, "PandarQT128"},
    {SensorModel::HESAI_PANDARXT16, "PandarXT16"},
    {SensorModel::HESAI_PANDARXT32, "PandarXT32"},
    {SensorModel::HESAI_PANDARXT32M, "PandarXT32M"},
    {SensorModel::HESAI_PANDARAT128, "PandarAT128"},
    {SensorModel::HESAI_PANDAR128_E3X, "Pandar128_E3X"},
    {SensorModel::HESAI_PANDAR128_E4X, "Pandar128_E4X_OT"},
    {SensorModel::VELODYNE_VLS128, "VLS128"},
    {SensorModel::VELODYNE_HDL64, "HDL64"},
    {SensorModel::VELODYNE_VLP32, "VLP32"},
    {SensorModel::VELODYNE_VLP32MR, "VLP32MR"},
    {SensorModel::VELODYNE_HDL32, "HDL32"},
    {SensorModel::VELODYNE_VLP16, "VLP16"},
    {SensorModel::ROBOSENSE_HELIOS, "HELIOS"},
    {SensorModel::ROBOSENSE_BPEARL_V3, "BPEARL V3.0"},
    {SensorModel::ROBOSENSE_BPEARL_V4, "BPEARL V4.0"},
    {SensorModel::CONTINENTAL_ARS548, "ARS548"},
    {SensorModel::CONTINENTAL_SRR520, "SRR520"},
    {SensorModel::UNKNOWN, "Sensor Unknown"},
  }};

  for (const auto & [value, expected_string] : streamed_values) {
    EXPECT_EQ(stream_to_string(value), expected_string);
  }

  constexpr std::array<std::pair<std::string_view, SensorModel>, 22> canonical_round_trips{{
    {"Pandar64", SensorModel::HESAI_PANDAR64},
    {"Pandar40P", SensorModel::HESAI_PANDAR40P},
    {"Pandar40M", SensorModel::HESAI_PANDAR40M},
    {"PandarXT16", SensorModel::HESAI_PANDARXT16},
    {"PandarXT32", SensorModel::HESAI_PANDARXT32},
    {"PandarXT32M", SensorModel::HESAI_PANDARXT32M},
    {"PandarAT128", SensorModel::HESAI_PANDARAT128},
    {"PandarQT64", SensorModel::HESAI_PANDARQT64},
    {"PandarQT128", SensorModel::HESAI_PANDARQT128},
    {"Pandar128E3X", SensorModel::HESAI_PANDAR128_E3X},
    {"Pandar128E4X", SensorModel::HESAI_PANDAR128_E4X},
    {"VLS128", SensorModel::VELODYNE_VLS128},
    {"HDL64", SensorModel::VELODYNE_HDL64},
    {"VLP32", SensorModel::VELODYNE_VLP32},
    {"VLP32MR", SensorModel::VELODYNE_VLP32MR},
    {"HDL32", SensorModel::VELODYNE_HDL32},
    {"VLP16", SensorModel::VELODYNE_VLP16},
    {"Helios", SensorModel::ROBOSENSE_HELIOS},
    {"Bpearl_V3", SensorModel::ROBOSENSE_BPEARL_V3},
    {"Bpearl_V4", SensorModel::ROBOSENSE_BPEARL_V4},
    {"ARS548", SensorModel::CONTINENTAL_ARS548},
    {"SRR520", SensorModel::CONTINENTAL_SRR520},
  }};

  for (const auto & [string_value, expected_model] : canonical_round_trips) {
    EXPECT_EQ(nebula::drivers::sensor_model_from_string(std::string(string_value)), expected_model);
    EXPECT_EQ(nebula::drivers::sensor_model_to_string(expected_model), string_value);
  }

  EXPECT_EQ(nebula::drivers::sensor_model_from_string("Bpearl"), SensorModel::ROBOSENSE_BPEARL_V4);
  EXPECT_EQ(nebula::drivers::sensor_model_from_string("NotASensor"), SensorModel::UNKNOWN);
  EXPECT_EQ(nebula::drivers::sensor_model_to_string(SensorModel::UNKNOWN), "UNKNOWN");
}

TEST(NebulaCommonTest, StatusStreamingHandlesKnownAndFallbackValues)
{
  const std::array<std::pair<Status, std::string_view>, 12> expected_values{{
    {Status::OK, "OK"},
    {Status::UDP_CONNECTION_ERROR, "Udp Connection Error"},
    {Status::CAN_CONNECTION_ERROR, "Can Connection Error"},
    {Status::SENSOR_CONFIG_ERROR, "Could not set SensorConfiguration"},
    {Status::INVALID_SENSOR_MODEL, "Invalid sensor model provided"},
    {Status::INVALID_ECHO_MODE, "Invalid echo model provided"},
    {Status::NOT_IMPLEMENTED, "Not Implemented"},
    {Status::NOT_INITIALIZED, "Not Initialized"},
    {Status::INVALID_CALIBRATION_FILE, "Invalid Calibration File"},
    {Status::CANNOT_SAVE_FILE, "Cannot Save File"},
    {Status::HTTP_CONNECTION_ERROR, "Http Connection Error"},
    {Status::WAITING_FOR_SENSOR_RESPONSE, "Waiting for Sensor Response"},
  }};

  for (const auto & [status, expected_string] : expected_values) {
    EXPECT_EQ(stream_to_string(status), expected_string);
  }

  EXPECT_EQ(stream_to_string(Status{Status::ERROR_1}), "Generic Error");
  EXPECT_EQ(stream_to_string(Status{999}), "Generic Error");
  EXPECT_EQ(Status(Status::OK), Status(Status::OK));
  EXPECT_NE(Status(Status::OK), Status(Status::ERROR_1));
}

TEST(NebulaCommonTest, ConfigurationStreamOperatorsReflectFieldValues)
{
  SensorConfigurationBase sensor_config{};
  sensor_config.sensor_model = SensorModel::VELODYNE_VLP16;
  sensor_config.frame_id = "sensor_frame";
  expect_contains_all(stream_to_string(sensor_config), {"VLP16", "sensor_frame"});

  EthernetSensorConfigurationBase ethernet_config{};
  ethernet_config.sensor_model = SensorModel::HESAI_PANDAR64;
  ethernet_config.frame_id = "ethernet_frame";
  ethernet_config.host_ip = "192.168.1.10";
  ethernet_config.sensor_ip = "192.168.1.20";
  ethernet_config.data_port = 2368;
  expect_contains_all(
    stream_to_string(ethernet_config),
    {"Pandar64", "ethernet_frame", "192.168.1.10", "192.168.1.20", "2368"});

  CANSensorConfigurationBase can_config{};
  can_config.sensor_model = SensorModel::CONTINENTAL_SRR520;
  can_config.frame_id = "can_frame";
  can_config.interface = "can0";
  can_config.receiver_timeout_sec = 0.25F;
  can_config.sender_timeout_sec = 0.5F;
  can_config.filters = "100:7FF";
  can_config.use_bus_time = true;
  expect_contains_all(
    stream_to_string(can_config), {"SRR520", "can_frame", "can0", "0.25", "0.5", "100:7FF", "1"});

  LidarConfigurationBase lidar_config{};
  lidar_config.sensor_model = SensorModel::ROBOSENSE_HELIOS;
  lidar_config.frame_id = "lidar_frame";
  lidar_config.host_ip = "10.0.0.1";
  lidar_config.sensor_ip = "10.0.0.2";
  lidar_config.data_port = 6699;
  lidar_config.return_mode = ReturnMode::SINGLE_STRONGEST;
  lidar_config.packet_mtu_size = 1500;
  lidar_config.use_sensor_time = true;
  expect_contains_all(
    stream_to_string(lidar_config),
    {"HELIOS", "lidar_frame", "10.0.0.1", "10.0.0.2", "6699", "SingleStrongest", "1500", "1"});
}

TEST(NebulaCommonTest, PointConversionsPreserveRelevantFields)
{
  PointCloud<PointXYZIRCAEDT> input_cloud;

  PointXYZIRCAEDT first{};
  first.x = 1.0F;
  first.y = -2.0F;
  first.z = 3.5F;
  first.intensity = 7U;
  first.return_type = static_cast<std::uint8_t>(ReturnType::SECOND);
  first.channel = 12U;
  first.azimuth = nebula::drivers::deg2rad(45.0F);
  first.distance = 9.5F;
  first.time_stamp = 250000000U;
  input_cloud.push_back(first);

  PointXYZIRCAEDT second{};
  second.x = -4.0F;
  second.y = 5.0F;
  second.z = -6.0F;
  second.intensity = 200U;
  second.return_type = static_cast<std::uint8_t>(ReturnType::LAST_STRONGEST);
  second.channel = 99U;
  second.azimuth = nebula::drivers::deg2rad(180.0F);
  second.distance = 42.0F;
  second.time_stamp = 999U;
  input_cloud.push_back(second);

  const PointCloud<PointXYZIR> xyzir_cloud =
    nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(input_cloud);
  ASSERT_EQ(xyzir_cloud.size(), input_cloud.size());
  EXPECT_FLOAT_EQ(xyzir_cloud[0].x, first.x);
  EXPECT_FLOAT_EQ(xyzir_cloud[0].y, first.y);
  EXPECT_FLOAT_EQ(xyzir_cloud[0].z, first.z);
  EXPECT_FLOAT_EQ(xyzir_cloud[0].intensity, static_cast<float>(first.intensity));
  EXPECT_EQ(xyzir_cloud[0].ring, first.channel);
  EXPECT_EQ(xyzir_cloud[1].ring, second.channel);

  constexpr double stamp = 1000.5;
  const PointCloud<PointXYZIRADT> xyziradt_cloud =
    nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(input_cloud, stamp);
  ASSERT_EQ(xyziradt_cloud.size(), input_cloud.size());
  EXPECT_FLOAT_EQ(xyziradt_cloud[0].azimuth, 4500.0F);
  EXPECT_FLOAT_EQ(xyziradt_cloud[1].azimuth, 18000.0F);
  EXPECT_FLOAT_EQ(xyziradt_cloud[0].distance, first.distance);
  EXPECT_EQ(xyziradt_cloud[0].return_type, first.return_type);
  EXPECT_DOUBLE_EQ(xyziradt_cloud[0].time_stamp, stamp + 0.25);
  EXPECT_DOUBLE_EQ(xyziradt_cloud[1].time_stamp, stamp + 999.0e-9);

  const PointCloud<PointXYZ> xyz_cloud =
    nebula::drivers::convert_point_xyzircaedt_to_point_xyz(input_cloud);
  ASSERT_EQ(xyz_cloud.size(), input_cloud.size());
  EXPECT_FLOAT_EQ(xyz_cloud[1].x, second.x);
  EXPECT_FLOAT_EQ(xyz_cloud[1].y, second.y);
  EXPECT_FLOAT_EQ(xyz_cloud[1].z, second.z);
}

TEST(NebulaCommonTest, AngleAndSpeedHelpersConvertUnits)
{
  constexpr double pi = 3.14159265358979323846;

  EXPECT_NEAR(nebula::drivers::deg2rad(180.0), pi, 1.0e-12);
  EXPECT_NEAR(nebula::drivers::deg2rad(180), pi, 1.0e-12);
  EXPECT_NEAR(nebula::drivers::rad2deg(pi / 2.0), 90.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(nebula::drivers::rpm2hz(600.0), 10.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
