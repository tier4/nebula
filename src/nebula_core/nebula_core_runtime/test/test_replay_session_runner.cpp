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

#include <boost/algorithm/string.hpp>
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
namespace fs = boost::filesystem;

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

    std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
    for (const auto & env_name : prefix_envs) {
      char * env_val = std::getenv(env_name.c_str());
      if (env_val) {
        std::vector<std::string> prefixes;
        boost::split(prefixes, env_val, boost::is_any_of(":"));
        for (const auto & prefix : prefixes) {
          if (prefix.empty()) continue;
          // Try common lib
          fs::path p1 = fs::path(prefix) / "lib" / "libnebula_sample_decoders_plugin.so";
          if (fs::exists(p1)) {
            plugin_library_path_ = p1.string();
            break;
          }
          // Try isolated lib
          fs::path p2 = fs::path(prefix) / "nebula_sample_decoders" / "lib" /
                        "libnebula_sample_decoders_plugin.so";
          if (fs::exists(p2)) {
            plugin_library_path_ = p2.string();
            break;
          }
        }
        if (!plugin_library_path_.empty()) break;
      }
    }

    if (plugin_library_path_.empty()) {
      fs::path install_prefix(NEBULA_TEST_INSTALL_PREFIX);
      fs::path isolated = install_prefix.parent_path() / "nebula_sample_decoders" / "lib" /
                          "libnebula_sample_decoders_plugin.so";
      if (fs::exists(isolated)) {
        plugin_library_path_ = isolated.string();
      }
    }
  }

  void TearDown() override { fs::remove_all(test_dir_); }

  fs::path test_dir_;
  std::string plugin_library_path_;
};

TEST_F(TestReplaySessionRunner, ReplaySampleSensor)
{
  if (plugin_library_path_.empty() || !fs::exists(plugin_library_path_)) {
    GTEST_SKIP() << "Sample plugin library not found";
  }

  fs::path dependency_path =
    fs::path(plugin_library_path_).parent_path() / "libnebula_sample_decoders.so";
  if (fs::exists(dependency_path)) {
    void * dependency = dlopen(dependency_path.string().c_str(), RTLD_LAZY | RTLD_GLOBAL);
    ASSERT_NE(dependency, nullptr) << dlerror();
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
  ReplaySessionConfig config;
  config.model = SensorModel::SAMPLE;
  config.pcap_file = (test_dir_ / "missing.pcap").string();
  config.sensor_config.sensor_model = SensorModel::SAMPLE;
  config.sensor_config.data_port = 2368;

  EXPECT_THROW(runner.configure(config), std::runtime_error);
}

}  // namespace nebula::drivers::test
