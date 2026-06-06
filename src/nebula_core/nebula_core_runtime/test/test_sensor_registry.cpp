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

#include <cstdlib>
#include <fstream>
#include <string>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

class ScopedEnvVar
{
public:
  ScopedEnvVar(const std::string & name, const std::string & value) : name_(name)
  {
    const char * old_value = std::getenv(name_.c_str());
    if (old_value) {
      had_old_value_ = true;
      old_value_ = old_value;
    }
    setenv(name_.c_str(), value.c_str(), 1);
  }

  ~ScopedEnvVar()
  {
    if (had_old_value_) {
      setenv(name_.c_str(), old_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  bool had_old_value_{false};
  std::string old_value_;
};

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

TEST_F(TestSensorRegistry, LoadRegistryRejectsUnknownModelNames)
{
  fs::path descriptor_path = test_dir_ / "bad_model.json";
  std::ofstream ofs(descriptor_path.string());
  ofs << R"({
    "vendor": "test_vendor",
    "package": "nebula_test_decoders",
    "library": "libnebula_test_plugin.so",
    "models": ["Pandar64 "]
  })";
  ofs.close();

  SensorRegistry registry;
  registry.load_registry({test_dir_.string()});

  EXPECT_TRUE(registry.get_registered_plugins().empty());
  EXPECT_FALSE(registry.find_plugin_for_model(SensorModel::UNKNOWN).has_value());
}

TEST_F(TestSensorRegistry, LoadRegistryKeepsFirstDuplicatePackageDescriptor)
{
  std::ofstream first((test_dir_ / "a_plugin.json").string());
  first << R"({
    "vendor": "first_vendor",
    "package": "nebula_test_decoders",
    "library": "libfirst_plugin.so",
    "models": ["Pandar64"]
  })";
  first.close();

  std::ofstream second((test_dir_ / "b_plugin.json").string());
  second << R"({
    "vendor": "second_vendor",
    "package": "nebula_test_decoders",
    "library": "libsecond_plugin.so",
    "models": ["VLP16"]
  })";
  second.close();

  SensorRegistry registry;
  registry.load_registry({test_dir_.string()});

  const auto & plugins = registry.get_registered_plugins();
  ASSERT_EQ(plugins.size(), 1u);
  ASSERT_EQ(plugins.count("nebula_test_decoders"), 1u);
  EXPECT_EQ(plugins.at("nebula_test_decoders").vendor, "first_vendor");
  EXPECT_EQ(plugins.at("nebula_test_decoders").library_path, "libfirst_plugin.so");
}

TEST_F(TestSensorRegistry, PrefixDiscoveryDoesNotRequirePluginFilename)
{
  fs::path share_dir = test_dir_ / "share" / "nebula_test_decoders";
  fs::create_directories(share_dir);
  std::ofstream descriptor((share_dir / "hesai_pandar64.json").string());
  descriptor << R"({
    "vendor": "test_vendor",
    "package": "nebula_test_decoders",
    "library": "libnebula_test_plugin.so",
    "models": ["Pandar64"]
  })";
  descriptor.close();

  ScopedEnvVar plugins_path("NEBULA_PLUGINS_PATH", "");
  ScopedEnvVar ament_prefix_path("AMENT_PREFIX_PATH", test_dir_.string());
  ScopedEnvVar colcon_prefix_path("COLCON_PREFIX_PATH", "");

  SensorRegistry registry;
  registry.load_registry({});

  auto metadata = registry.find_plugin_for_model(SensorModel::HESAI_PANDAR64);
  ASSERT_TRUE(metadata.has_value());
  EXPECT_EQ(metadata->package_name, "nebula_test_decoders");
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

TEST_F(TestSensorRegistry, LoadRegistryThrowsAfterFinalize)
{
  SensorRegistry registry;
  registry.finalize();
  EXPECT_THROW(registry.load_registry({test_dir_.string()}), std::logic_error);
}

TEST_F(TestSensorRegistry, LoadPluginThrowsForNewPluginAfterFinalize)
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
  registry.finalize();

  auto metadata = registry.find_plugin_for_model(SensorModel::HESAI_PANDAR64);
  ASSERT_TRUE(metadata.has_value());
  EXPECT_THROW(registry.load_plugin(*metadata), std::logic_error);
}

}  // namespace nebula::drivers::test
