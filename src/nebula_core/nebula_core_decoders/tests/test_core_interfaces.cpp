// Copyright 2024 TIER IV, Inc.
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

#include <nebula_core_decoders/sensor_decoder_runtime.hpp>
#include <nebula_core_decoders/sensor_plugin.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using nebula::drivers::LiveTransportRequirement;
using nebula::drivers::PacketChannelRequirement;
using nebula::drivers::SensorConfiguration;
using nebula::drivers::SensorDecoderRuntime;
using nebula::drivers::SensorErrorCallback;
using nebula::drivers::SensorModelInfo;
using nebula::drivers::SensorOutputCallback;
using nebula::drivers::SensorPacket;
using nebula::drivers::SensorPacketResult;
using nebula::drivers::SensorPacketView;
using nebula::drivers::SensorPlugin;
using nebula::drivers::SensorPluginMetadata;
using nebula::drivers::SensorProgressCallback;

class MockSensorDecoderRuntime : public SensorDecoderRuntime
{
public:
  void configure(const SensorConfiguration &) override {}
  void set_output_callback(SensorOutputCallback) override {}
  void set_error_callback(SensorErrorCallback) override {}
  void set_progress_callback(SensorProgressCallback) override {}
  void set_sink(SensorOutputSink *) override {}
  SensorPacketResult process_packet(const SensorPacketView &) override
  {
    return SensorPacketResult::Success;
  }
  void flush() override {}
};

class MockSensorPlugin : public SensorPlugin
{
public:
  SensorPluginMetadata metadata() const override { return {}; }
  std::vector<SensorModelInfo> supported_models() const override { return {}; }
  std::vector<PacketChannelRequirement> packet_requirements(
    const SensorConfiguration &) const override
  {
    return {};
  }
  std::vector<LiveTransportRequirement> live_transport_requirements(
    const SensorConfiguration &) const override
  {
    return {};
  }
  std::unique_ptr<SensorDecoderRuntime> create_decoder_runtime() const override
  {
    return std::make_unique<MockSensorDecoderRuntime>();
  }
};

TEST(TestCoreInterfaces, SensorDecoderRuntime)
{
  std::unique_ptr<SensorDecoderRuntime> runtime = std::make_unique<MockSensorDecoderRuntime>();
  SensorPacket packet;
  EXPECT_EQ(runtime->process_packet(SensorPacketView::from(packet)), SensorPacketResult::Success);
}

TEST(TestCoreInterfaces, SensorPlugin)
{
  std::unique_ptr<SensorPlugin> plugin = std::make_unique<MockSensorPlugin>();
  auto runtime = plugin->create_decoder_runtime();
  EXPECT_NE(runtime, nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
