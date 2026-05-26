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

#include <nebula_core_runtime/sensor_registry.hpp>

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <string>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

class TestSensorRegistry : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = fs::temp_directory_path() / fs::unique_path();
    fs::create_directories(test_dir_);
  }

  void TearDown() override { fs::remove_all(test_dir_); }

  fs::path test_dir_;
};

TEST_F(TestSensorRegistry, LoadRegistry)
{
  fs::create_directories(test_dir_ / "schemas");
  std::ofstream schema((test_dir_ / "schemas" / "test.schema.json").string());
  schema << R"({"type":"object"})";
  schema.close();

  fs::path descriptor_path = test_dir_ / "test_plugin.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "test_vendor",
    "package": "nebula_test_decoders",
    "library": "libnebula_test_plugin.so",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Pandar64", "VLP16"],
    "schema": "schemas/test.schema.json"
  })";
  ofs.close();

  SensorRegistry registry;
  registry.load_registry({test_dir_.string()});

  auto plugins = registry.get_registered_plugins();
  EXPECT_EQ(plugins.size(), 1u);
  EXPECT_EQ(plugins.count("nebula_test_decoders"), 1u);
  EXPECT_EQ(plugins["nebula_test_decoders"].vendor, "test_vendor");
  EXPECT_EQ(plugins["nebula_test_decoders"].supported_models.size(), 2u);
  EXPECT_EQ(plugins["nebula_test_decoders"].descriptor_path, descriptor_path.string());
  EXPECT_EQ(
    plugins["nebula_test_decoders"].schema_path,
    (test_dir_ / "schemas" / "test.schema.json").string());
}

TEST_F(TestSensorRegistry, FindPluginForModel)
{
  fs::path descriptor_path = test_dir_ / "test_plugin.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "test_vendor",
    "package": "nebula_test_decoders",
    "library": "libnebula_test_plugin.so",
    "factory": "create_nebula_sensor_plugin",
    "models": ["Pandar64"]
  })";
  ofs.close();

  SensorRegistry registry;
  registry.load_registry({test_dir_.string()});

  auto metadata = registry.find_plugin_for_model(SensorModel::HESAI_PANDAR64);
  ASSERT_TRUE(metadata.has_value());
  EXPECT_EQ(metadata->package_name, "nebula_test_decoders");

  auto not_found = registry.find_plugin_for_model(SensorModel::VELODYNE_VLP16);
  EXPECT_FALSE(not_found.has_value());
}

}  // namespace nebula::drivers::test
