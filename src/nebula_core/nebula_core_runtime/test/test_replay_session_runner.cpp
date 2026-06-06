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

#include <nebula_core_runtime/replay_session_runner.hpp>

#include "sample_plugin_test_utils.hpp"

#include <boost/filesystem.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers::test
{
namespace
{
void append_u16_be(std::vector<uint8_t> & bytes, uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  bytes.push_back(static_cast<uint8_t>(value & 0xff));
}

void append_u32_be(std::vector<uint8_t> & bytes, uint32_t value)
{
  bytes.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
  bytes.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
  bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  bytes.push_back(static_cast<uint8_t>(value & 0xff));
}

void write_u16_le(std::ofstream & out, uint16_t value)
{
  const std::array<char, 2> bytes{
    static_cast<char>(value & 0xff), static_cast<char>((value >> 8) & 0xff)};
  out.write(bytes.data(), bytes.size());
}

void write_u32_le(std::ofstream & out, uint32_t value)
{
  const std::array<char, 4> bytes{
    static_cast<char>(value & 0xff), static_cast<char>((value >> 8) & 0xff),
    static_cast<char>((value >> 16) & 0xff), static_cast<char>((value >> 24) & 0xff)};
  out.write(bytes.data(), bytes.size());
}

std::vector<uint8_t> make_udp_ethernet_packet(uint16_t destination_port)
{
  std::vector<uint8_t> packet;
  const std::array<uint8_t, 4> payload{0xde, 0xad, 0xbe, 0xef};

  packet.insert(packet.end(), {0x02, 0x00, 0x00, 0x00, 0x00, 0x02});
  packet.insert(packet.end(), {0x02, 0x00, 0x00, 0x00, 0x00, 0x01});
  append_u16_be(packet, 0x0800);

  packet.push_back(0x45);
  packet.push_back(0x00);
  append_u16_be(packet, static_cast<uint16_t>(20 + 8 + payload.size()));
  append_u16_be(packet, 0x1234);
  append_u16_be(packet, 0x0000);
  packet.push_back(64);
  packet.push_back(17);
  append_u16_be(packet, 0x0000);
  packet.insert(packet.end(), {192, 168, 1, 201});
  packet.insert(packet.end(), {192, 168, 1, 100});

  append_u16_be(packet, 50000);
  append_u16_be(packet, destination_port);
  append_u16_be(packet, static_cast<uint16_t>(8 + payload.size()));
  append_u16_be(packet, 0x0000);
  packet.insert(packet.end(), payload.begin(), payload.end());
  return packet;
}

void write_sample_pcap(const fs::path & pcap_path, uint16_t destination_port, size_t packet_count)
{
  std::ofstream out(pcap_path.string(), std::ios::binary);
  write_u32_le(out, 0xa1b2c3d4);
  write_u16_le(out, 2);
  write_u16_le(out, 4);
  write_u32_le(out, 0);
  write_u32_le(out, 0);
  write_u32_le(out, 65535);
  write_u32_le(out, 1);

  const auto packet = make_udp_ethernet_packet(destination_port);
  for (size_t i = 0; i < packet_count; ++i) {
    write_u32_le(out, 1);
    write_u32_le(out, static_cast<uint32_t>(i));
    write_u32_le(out, static_cast<uint32_t>(packet.size()));
    write_u32_le(out, static_cast<uint32_t>(packet.size()));
    out.write(reinterpret_cast<const char *>(packet.data()), packet.size());
  }
}
}  // namespace

class TestReplaySessionRunner : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = fs::temp_directory_path() / fs::unique_path();
    fs::create_directories(test_dir_);

    plugin_library_path_ = find_sample_plugin_library();
    ASSERT_NO_THROW(dependency_handle_ = load_sample_plugin_dependency(plugin_library_path_));
  }

  void TearDown() override
  {
    fs::remove_all(test_dir_);
    if (dependency_handle_) {
      dlclose(dependency_handle_);
      dependency_handle_ = nullptr;
    }
  }

  fs::path test_dir_;
  std::string plugin_library_path_;
  void * dependency_handle_{nullptr};
};

TEST_F(TestReplaySessionRunner, ReplaySampleSensor)
{
  if (plugin_library_path_.empty() || !fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found";
  }

  // Create a dummy descriptor
  fs::path descriptor_path = test_dir_ / "nebula_sample_decoders.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "nebula",
    "package": "nebula_sample_decoders",
    "library": ")"
      << plugin_library_path_ << R"(",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Sample"]
  })";
  ofs.close();

  auto registry = std::make_shared<SensorRegistry>();
  registry->load_registry({test_dir_.string()});

  ReplaySessionRunner runner(registry);
  bool output_received = false;
  runner.set_output_callback([&](const SensorDecodedOutput & output) {
    EXPECT_EQ(output.kind, SensorOutputKind::PointCloud);
    output_received = true;
  });

  const fs::path pcap_path = test_dir_ / "sample_udp.pcap";
  write_sample_pcap(pcap_path, 2368, 10);

  ReplaySessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.pcap_file = pcap_path.string();
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.data_port = 2368;

  EXPECT_NO_THROW(runner.configure(config));
  runner.start();
  runner.wait_until_finished();
  EXPECT_TRUE(output_received);
}

TEST_F(TestReplaySessionRunner, ConfigureRejectsMissingPcap)
{
  if (plugin_library_path_.empty() || !fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found";
  }

  fs::path descriptor_path = test_dir_ / "nebula_sample_decoders.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "nebula",
    "package": "nebula_sample_decoders",
    "library": ")"
      << plugin_library_path_ << R"(",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Sample"]
  })";
  ofs.close();

  auto registry = std::make_shared<SensorRegistry>();
  registry->load_registry({test_dir_.string()});

  ReplaySessionRunner runner(registry);
  size_t error_count = 0;
  runner.set_error_callback([&](const SensorError &) { ++error_count; });

  ReplaySessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.pcap_file = (test_dir_ / "missing.pcap").string();
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.data_port = 2368;

  EXPECT_THROW(runner.configure(config), std::runtime_error);
  EXPECT_NO_THROW(runner.start());
  EXPECT_NO_THROW(runner.wait_until_finished());
  EXPECT_EQ(error_count, 0u);
}

}  // namespace nebula::drivers::test
