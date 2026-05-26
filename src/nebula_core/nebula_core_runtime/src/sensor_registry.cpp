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

#include <nebula_core_runtime/sensor_registry.hpp>
#include <nlohmann/json.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <dlfcn.h>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{
namespace fs = boost::filesystem;

struct SensorRegistry::LoadedLibrary
{
  explicit LoadedLibrary(void * library_handle) : handle(library_handle) {}

  ~LoadedLibrary()
  {
    if (handle) {
      dlclose(handle);
    }
  }

  void * handle{nullptr};
};

namespace
{
std::string resolve_descriptor_path(
  const fs::path & descriptor_path, const std::string & path_value)
{
  if (path_value.empty()) {
    return {};
  }

  const fs::path raw_path(path_value);
  if (raw_path.is_absolute()) {
    return raw_path.string();
  }

  const fs::path relative_to_descriptor = descriptor_path.parent_path() / raw_path;
  return relative_to_descriptor.string();
}

std::vector<std::string> get_prefix_paths()
{
  std::vector<std::string> prefixes;
  for (const auto & env_name : {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"}) {
    char * env_val = std::getenv(env_name);
    if (!env_val) {
      continue;
    }

    std::vector<std::string> env_prefixes;
    boost::split(env_prefixes, env_val, boost::is_any_of(":"));
    for (const auto & prefix : env_prefixes) {
      if (!prefix.empty()) {
        prefixes.push_back(prefix);
      }
    }
  }
  return prefixes;
}

std::optional<std::string> resolve_library_from_prefixes(
  const std::string & library_path, const std::string & package_name)
{
  for (const auto & prefix : get_prefix_paths()) {
    const fs::path common_lib = fs::path(prefix) / "lib" / library_path;
    if (fs::exists(common_lib)) {
      return common_lib.string();
    }

    const fs::path isolated_lib = fs::path(prefix) / package_name / "lib" / library_path;
    if (!package_name.empty() && fs::exists(isolated_lib)) {
      return isolated_lib.string();
    }
  }
  return std::nullopt;
}

std::string resolve_library_path(
  const fs::path & descriptor_path, const std::string & library_path,
  const std::string & package_name)
{
  const fs::path raw_path(library_path);
  if (raw_path.is_absolute()) {
    return library_path;
  }

  const fs::path relative_to_descriptor = descriptor_path.parent_path() / raw_path;
  if (fs::exists(relative_to_descriptor)) {
    return relative_to_descriptor.string();
  }

  auto resolved = resolve_library_from_prefixes(library_path, package_name);
  return resolved.value_or(library_path);
}
}  // namespace

SensorRegistry::~SensorRegistry()
{
  instantiated_plugins_.clear();
  loaded_libraries_.clear();
}

void SensorRegistry::load_registry(const std::vector<std::string> & search_paths)
{
  if (finalized_) {
    throw std::logic_error(
      "SensorRegistry::load_registry called after finalize() — all descriptors must be loaded "
      "before the registry is frozen");
  }

  std::vector<std::string> descriptor_files;

  // 1. Explicit search paths
  for (const auto & path : search_paths) {
    if (fs::exists(path) && fs::is_directory(path)) {
      for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
        if (fs::is_regular_file(it->status()) && it->path().extension() == ".json") {
          descriptor_files.push_back(it->path().string());
        }
      }
    }
  }

  // 2. Add paths from NEBULA_PLUGINS_PATH
  char * nebula_env = std::getenv("NEBULA_PLUGINS_PATH");
  if (nebula_env) {
    std::vector<std::string> env_paths;
    boost::split(env_paths, nebula_env, boost::is_any_of(":"));
    for (const auto & path : env_paths) {
      if (fs::exists(path) && fs::is_directory(path)) {
        for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
          if (fs::is_regular_file(it->status()) && it->path().extension() == ".json") {
            descriptor_files.push_back(it->path().string());
          }
        }
      }
    }
  }

  // 3. Auto-discover from AMENT_PREFIX_PATH and COLCON_PREFIX_PATH
  std::vector<std::string> prefix_envs = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"};
  for (const auto & env_name : prefix_envs) {
    char * env_val = std::getenv(env_name.c_str());
    if (env_val) {
      std::vector<std::string> prefixes;
      boost::split(prefixes, env_val, boost::is_any_of(":"));
      for (const auto & prefix : prefixes) {
        if (prefix.empty()) continue;

        std::vector<fs::path> share_paths;
        // Add <prefix>/share (merged install or ament prefix)
        share_paths.push_back(fs::path(prefix) / "share");

        // Add <prefix>/*/share (isolated colcon install)
        if (fs::exists(prefix) && fs::is_directory(prefix)) {
          for (fs::directory_iterator it(prefix); it != fs::directory_iterator(); ++it) {
            if (fs::is_directory(it->status())) {
              share_paths.push_back(it->path() / "share");
            }
          }
        }

        for (const auto & share_path : share_paths) {
          if (fs::exists(share_path) && fs::is_directory(share_path)) {
            for (fs::directory_iterator it(share_path); it != fs::directory_iterator(); ++it) {
              if (fs::is_directory(it->status())) {
                for (fs::directory_iterator pkg_it(it->path()); pkg_it != fs::directory_iterator();
                     ++pkg_it) {
                  if (
                    fs::is_regular_file(pkg_it->status()) &&
                    (pkg_it->path().extension() == ".json") &&
                    (pkg_it->path().filename().string().find("plugin") != std::string::npos)) {
                    descriptor_files.push_back(pkg_it->path().string());
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  for (const auto & file_path : descriptor_files) {
    try {
      std::ifstream ifs(file_path);
      nlohmann::json j = nlohmann::json::parse(ifs);

      if (!j.contains("vendor") || !j.contains("package") || !j.contains("library")) {
        continue;
      }

      SensorPluginMetadata metadata;
      metadata.vendor = j.at("vendor").get<std::string>();
      metadata.package_name = j.at("package").get<std::string>();
      metadata.library_path = j.at("library").get<std::string>();
      metadata.factory_symbol = j.value("factory", "create_nebula_sensor_plugin");
      metadata.descriptor_path = file_path;
      metadata.package_share_path = fs::path(file_path).parent_path().string();
      metadata.schema_path = resolve_descriptor_path(file_path, j.value("schema", ""));
      metadata.config_defaults_path =
        resolve_descriptor_path(file_path, j.value("config_defaults", ""));
      metadata.calibration_assets_path =
        resolve_descriptor_path(file_path, j.value("calibration_assets", ""));

      for (const auto & m : j.at("models")) {
        metadata.supported_models.push_back(sensor_model_from_string(m.get<std::string>()));
      }

      metadata.library_path =
        resolve_library_path(file_path, metadata.library_path, metadata.package_name);

      registered_plugins_[metadata.package_name] = metadata;
    } catch (const std::exception & e) {
      std::cerr << "Failed to parse plugin descriptor " << file_path << ": " << e.what()
                << std::endl;
    }
  }
}

std::optional<SensorPluginMetadata> SensorRegistry::find_plugin_for_model(SensorModel model) const
{
  for (const auto & pair : registered_plugins_) {
    for (const auto & supported_model : pair.second.supported_models) {
      if (supported_model == model) {
        return pair.second;
      }
    }
  }
  return std::nullopt;
}

void SensorRegistry::finalize()
{
  finalized_ = true;
}

std::shared_ptr<SensorPlugin> SensorRegistry::load_plugin(const SensorPluginMetadata & metadata)
{
  if (finalized_) {
    throw std::logic_error(
      "SensorRegistry::load_plugin called after finalize() — all plugins must be loaded before "
      "the registry is frozen");
  }

  auto plugin_it = instantiated_plugins_.find(metadata.package_name);
  if (plugin_it != instantiated_plugins_.end()) {
    return plugin_it->second;
  }

  std::string load_error;
  auto library = load_library(metadata.library_path, metadata.package_name, load_error);
  if (!library) {
    std::cerr << "Failed to load library " << metadata.library_path << " for plugin "
              << metadata.package_name << ": " << load_error << std::endl;
    return nullptr;
  }

  using CreateFunc = SensorPlugin * (*)();
  using DestroyFunc = void (*)(SensorPlugin *);

  auto create_func =
    reinterpret_cast<CreateFunc>(dlsym(library->handle, metadata.factory_symbol.c_str()));
  if (!create_func) {
    std::cerr << "Failed to find factory symbol '" << metadata.factory_symbol << "' in "
              << metadata.library_path << ": " << dlerror() << std::endl;
    return nullptr;
  }

  auto destroy_func =
    reinterpret_cast<DestroyFunc>(dlsym(library->handle, "destroy_nebula_sensor_plugin"));
  if (!destroy_func) {
    std::cerr << "Failed to find destroy symbol 'destroy_nebula_sensor_plugin' in "
              << metadata.library_path << ": " << dlerror() << std::endl;
    return nullptr;
  }

  using VersionFunc = uint32_t (*)();
  auto version_func =
    reinterpret_cast<VersionFunc>(dlsym(library->handle, "nebula_plugin_abi_version"));
  if (version_func) {
    const uint32_t plugin_version = version_func();
    if (plugin_version != kNebulaPluginAbiVersion) {
      std::cerr << "Plugin ABI version mismatch for " << metadata.package_name << ": host expects "
                << kNebulaPluginAbiVersion << ", plugin reports " << plugin_version << std::endl;
      return nullptr;
    }
  } else {
    std::cerr << "Warning: plugin " << metadata.package_name
              << " does not export nebula_plugin_abi_version(); assuming compatible" << std::endl;
  }

  std::shared_ptr<SensorPlugin> plugin(create_func(), [library, destroy_func](SensorPlugin * ptr) {
    if (ptr) {
      destroy_func(ptr);
    }
  });
  if (plugin) {
    instantiated_plugins_[metadata.package_name] = plugin;
  } else {
    std::cerr << "Factory function '" << metadata.factory_symbol << "' returned nullptr for plugin "
              << metadata.package_name << std::endl;
  }
  return plugin;
}

const std::map<std::string, SensorPluginMetadata> & SensorRegistry::get_registered_plugins() const
{
  return registered_plugins_;
}

std::shared_ptr<SensorRegistry::LoadedLibrary> SensorRegistry::load_library(
  const std::string & library_path, const std::string & package_name, std::string & error)
{
  auto library_it = loaded_libraries_.find(library_path);
  if (library_it != loaded_libraries_.end()) {
    return library_it->second;
  }
  auto failure_it = failed_library_errors_.find(library_path);
  if (failure_it != failed_library_errors_.end()) {
    error = failure_it->second;
    return nullptr;
  }

  void * handle = dlopen(library_path.c_str(), RTLD_LAZY | RTLD_LOCAL);
  // Capture dlerror() immediately into std::string before any subsequent dlopen()
  // call could overwrite the thread-local error buffer.
  if (!handle) {
    const char * raw = dlerror();
    error = raw ? raw : "unknown dlopen error";
  }

  if (!handle && fs::path(library_path).is_relative()) {
    auto resolved = resolve_library_from_prefixes(library_path, package_name);
    if (resolved) {
      handle = dlopen(resolved->c_str(), RTLD_LAZY | RTLD_LOCAL);
      if (!handle) {
        const char * raw = dlerror();
        error = raw ? raw : "unknown dlopen error";
      }
    }
  }

  if (handle) {
    auto library = std::make_shared<LoadedLibrary>(handle);
    // Intentional: successful loads are cached so subsequent calls return the
    // same shared_ptr (and thus the same dlopen handle). Once in loaded_libraries_,
    // a path is never retried even after a factory symbol error — callers that
    // need a fresh load must construct a new SensorRegistry.
    loaded_libraries_[library_path] = library;
    return library;
  } else {
    if (error.empty()) {
      error = "library was not found in the configured plugin search paths";
    }
    failed_library_errors_[library_path] = error;
    return nullptr;
  }
}

}  // namespace nebula::drivers
