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

#ifndef NEBULA_CORE_RUNTIME_TEST_SAMPLE_PLUGIN_TEST_UTILS_HPP
#define NEBULA_CORE_RUNTIME_TEST_SAMPLE_PLUGIN_TEST_UTILS_HPP

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <dlfcn.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers::test
{
namespace fs = boost::filesystem;

inline std::string find_sample_plugin_library()
{
  const std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
  for (const auto & env_name : prefix_envs) {
    char * env_val = std::getenv(env_name.c_str());
    if (!env_val) {
      continue;
    }

    std::vector<std::string> prefixes;
    boost::split(prefixes, env_val, boost::is_any_of(":"));
    for (const auto & prefix : prefixes) {
      if (prefix.empty()) {
        continue;
      }

      const fs::path common = fs::path(prefix) / "lib" / "libnebula_sample_decoders_plugin.so";
      if (fs::exists(common)) {
        return common.string();
      }

      const fs::path isolated =
        fs::path(prefix) / "nebula_sample_decoders" / "lib" / "libnebula_sample_decoders_plugin.so";
      if (fs::exists(isolated)) {
        return isolated.string();
      }
    }
  }

  const fs::path install_prefix(NEBULA_TEST_INSTALL_PREFIX);
  const fs::path isolated = install_prefix.parent_path() / "nebula_sample_decoders" / "lib" /
                            "libnebula_sample_decoders_plugin.so";
  if (fs::exists(isolated)) {
    return isolated.string();
  }

  return "install/nebula_sample_decoders/lib/libnebula_sample_decoders_plugin.so";
}

inline void * load_sample_plugin_dependency(const std::string & plugin_library_path)
{
  if (plugin_library_path.empty() || !fs::exists(plugin_library_path)) {
    return nullptr;
  }

  const fs::path dependency =
    fs::path(plugin_library_path).parent_path() / "libnebula_sample_decoders.so";
  if (!fs::exists(dependency)) {
    return nullptr;
  }

  void * handle = dlopen(dependency.string().c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!handle) {
    const char * error = dlerror();
    throw std::runtime_error(error ? error : "unknown dlopen error");
  }
  return handle;
}

}  // namespace nebula::drivers::test

#endif  // NEBULA_CORE_RUNTIME_TEST_SAMPLE_PLUGIN_TEST_UTILS_HPP
