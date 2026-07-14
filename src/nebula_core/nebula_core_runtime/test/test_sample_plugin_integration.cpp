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

#include "sample_plugin_test_utils.hpp"

#include <nebula_core_runtime/packet_router.hpp>
#include <nebula_core_runtime/sensor_registry.hpp>

#include <boost/filesystem.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers::test
{
class TestSamplePluginIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    plugin_library_path_ = find_sample_plugin_library();
    ASSERT_NO_THROW(dependency_handle_ = load_sample_plugin_dependency(plugin_library_path_));
  }

  void TearDown() override
  {
    if (dependency_handle_) {
      dlclose(dependency_handle_);
      dependency_handle_ = nullptr;
    }
  }

  std::string plugin_library_path_;
  void * dependency_handle_{nullptr};
};

TEST_F(TestSamplePluginIntegration, LoadAndRunSamplePlugin)
{
  if (!fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found at " << plugin_library_path_;
  }

  SensorRegistry registry;

  SensorPluginMetadata metadata;
  metadata.vendor = "nebula";
  metadata.package_name = "nebula_sample_decoders";
  metadata.library_path = plugin_library_path_;
  metadata.factory_symbol = "create_nebula_sensor_plugin";
  metadata.destroy_symbol = "destroy_nebula_sensor_plugin";
  metadata.supported_models = {SensorModel::SAMPLE};

  auto plugin = registry.load_plugin(metadata);
  ASSERT_NE(plugin, nullptr);
  EXPECT_EQ(plugin->metadata().vendor, "nebula");

  auto runtime = plugin->create_decoder_runtime();
  ASSERT_NE(runtime, nullptr);

  SensorConfiguration config;
  config.sensor_model = SensorModel::SAMPLE;
  config.data_port = 2368;
  runtime->configure(config);

  bool output_received = false;
  runtime->set_output_callback([&](const SensorDecodedOutput & output) {
    EXPECT_EQ(output.kind, SensorOutputKind::PointCloud);
    output_received = true;
  });

  PacketRouter router;
  router.configure(plugin->packet_requirements(config));

  // SampleDecoder needs 10 packets to emit a pointcloud
  for (int i = 0; i < 10; ++i) {
    SensorPacket packet;
    packet.transport = SensorTransportKind::UDP;
    packet.destination = SensorEndpoint{"", 2368};
    packet.payload = {0xde, 0xad, 0xbe, 0xef};
    SensorPacketView view = SensorPacketView::from(packet);
    ASSERT_TRUE(router.route(view));
    EXPECT_EQ(view.channel, SensorPacketChannel::Data);
    auto result = runtime->process_packet(view);
    EXPECT_EQ(result, SensorPacketResult::Success);
  }

  EXPECT_TRUE(output_received);
}

TEST_F(TestSamplePluginIntegration, PluginCanOutliveRegistry)
{
  if (!fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found at " << plugin_library_path_;
  }

  std::shared_ptr<SensorPlugin> plugin;

  {
    SensorRegistry registry;

    SensorPluginMetadata metadata;
    metadata.vendor = "nebula";
    metadata.package_name = "nebula_sample_decoders";
    metadata.library_path = plugin_library_path_;
    metadata.factory_symbol = "create_nebula_sensor_plugin";
    metadata.destroy_symbol = "destroy_nebula_sensor_plugin";
    metadata.supported_models = {SensorModel::SAMPLE};

    plugin = registry.load_plugin(metadata);
    ASSERT_NE(plugin, nullptr);
  }

  EXPECT_EQ(plugin->metadata().vendor, "nebula");
  plugin.reset();
}

}  // namespace nebula::drivers::test
