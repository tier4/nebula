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

#include <nebula_core_common/sensor_output.hpp>
#include <nebula_core_common/sensor_packet.hpp>
#include <nebula_core_common/sensor_runtime_common.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using nebula::drivers::NebulaPointCloud;
using nebula::drivers::NebulaPointCloudPtr;
using nebula::drivers::return_mode_from_string;
using nebula::drivers::ReturnMode;
using nebula::drivers::sensor_model_from_string;
using nebula::drivers::sensor_model_to_string;
using nebula::drivers::SensorDecodedOutput;
using nebula::drivers::SensorModel;
using nebula::drivers::SensorOutputKind;
using nebula::drivers::SensorPacket;
using nebula::drivers::SensorPacketChannel;
using nebula::drivers::SensorTransportKind;

TEST(TestCoreTypes, SensorPacket)
{
  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.channel = SensorPacketChannel::Data;
  packet.timestamp_ns = 123456789;
  packet.payload = {0x01, 0x02, 0x03};

  EXPECT_EQ(packet.transport, SensorTransportKind::UDP);
  EXPECT_EQ(packet.channel, SensorPacketChannel::Data);
  EXPECT_EQ(packet.timestamp_ns, 123456789);
  EXPECT_EQ(packet.payload.size(), 3u);
}

TEST(TestCoreTypes, SensorDecodedOutput)
{
  SensorDecodedOutput output;
  output.kind = SensorOutputKind::PointCloud;
  output.timestamp_ns = 987654321;
  output.sensor_id = "test_sensor";
  auto pointcloud = std::make_shared<NebulaPointCloud>();
  output.payload = pointcloud;

  EXPECT_EQ(output.kind, SensorOutputKind::PointCloud);
  EXPECT_EQ(output.timestamp_ns, 987654321);
  EXPECT_EQ(output.sensor_id, "test_sensor");
  ASSERT_TRUE(std::holds_alternative<NebulaPointCloudPtr>(output.payload));
  EXPECT_EQ(std::get<NebulaPointCloudPtr>(output.payload), pointcloud);
}

TEST(TestCoreTypes, SensorModelRoundTrip)
{
  EXPECT_EQ(sensor_model_from_string("Sample"), SensorModel::SAMPLE);
  EXPECT_EQ(sensor_model_to_string(SensorModel::SAMPLE), "Sample");
}

TEST(TestCoreTypes, ReturnModeRoundTrip)
{
  const std::vector<std::pair<ReturnMode, std::string>> cases = {
    {ReturnMode::SINGLE_FIRST, "SingleFirst"},
    {ReturnMode::SINGLE_STRONGEST, "SingleStrongest"},
    {ReturnMode::SINGLE_LAST, "SingleLast"},
    {ReturnMode::DUAL_ONLY, "DualOnly"},
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
  };

  for (const auto & test_case : cases) {
    std::stringstream stream;
    stream << test_case.first;
    EXPECT_EQ(stream.str(), test_case.second);
    EXPECT_EQ(return_mode_from_string(test_case.second), test_case.first);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
