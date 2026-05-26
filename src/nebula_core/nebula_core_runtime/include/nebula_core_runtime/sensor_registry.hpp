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

#ifndef NEBULA_SENSOR_REGISTRY_HPP
#define NEBULA_SENSOR_REGISTRY_HPP

#include <nebula_core_common/sensor_runtime_common.hpp>
#include <nebula_core_decoders/sensor_plugin.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nebula::drivers
{
class SensorRegistry
{
public:
  SensorRegistry() = default;
  ~SensorRegistry();

  /// @brief Load all plugin descriptors from the given paths
  /// @param search_paths List of directories to search for .json descriptors
  void load_registry(const std::vector<std::string> & search_paths);

  /// @brief Find a plugin that supports the given sensor model
  /// @param model The sensor model to find a plugin for
  /// @return Metadata of the plugin, or empty optional if not found
  std::optional<SensorPluginMetadata> find_plugin_for_model(SensorModel model) const;

  /// @brief Load a plugin library and instantiate the plugin object
  /// @param metadata The metadata of the plugin to load
  /// @return A unique pointer to the instantiated plugin, or nullptr on failure
  std::shared_ptr<SensorPlugin> load_plugin(const SensorPluginMetadata & metadata);

  /// @brief Get all registered plugins
  const std::map<std::string, SensorPluginMetadata> & get_registered_plugins() const;

  /// @brief Mark the registry as immutable. After this call load_plugin() throws
  /// std::logic_error. Call once all required plugins have been loaded, before
  /// starting any real-time threads.
  void finalize();
  bool is_finalized() const { return finalized_; }

private:
  struct LoadedLibrary;

  bool finalized_{false};
  std::map<std::string, SensorPluginMetadata> registered_plugins_;
  std::map<std::string, std::shared_ptr<LoadedLibrary>> loaded_libraries_;
  std::map<std::string, std::string> failed_library_errors_;
  std::map<std::string, std::shared_ptr<SensorPlugin>> instantiated_plugins_;

  std::shared_ptr<LoadedLibrary> load_library(
    const std::string & library_path, const std::string & package_name, std::string & error);
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_REGISTRY_HPP
