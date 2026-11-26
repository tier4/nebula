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

#include "nebula_ros2_control/component_loader.hpp"

#include "nebula_ros2_control/driver_config.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>

#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros2_control
{

namespace
{

constexpr const char * k_resource_type = "rclcpp_components";

struct ComponentEntry
{
  std::string class_name;
  std::string library_path;
};

/// Parse the `rclcpp_components` resource of one package (`<class>;<library>` per line).
std::vector<ComponentEntry> get_package_components(const std::string & package)
{
  std::string content;
  std::string prefix;
  if (!ament_index_cpp::get_resource(k_resource_type, package, content, &prefix)) {
    return {};
  }

  std::vector<ComponentEntry> entries;
  std::istringstream lines(content);
  std::string line;
  while (std::getline(lines, line)) {
    const auto separator = line.find(';');
    if (separator == std::string::npos) {
      continue;
    }
    ComponentEntry entry{line.substr(0, separator), line.substr(separator + 1)};
    if (!entry.library_path.empty() && entry.library_path.front() != '/') {
      entry.library_path = prefix + "/" + entry.library_path;
    }
    entries.push_back(std::move(entry));
  }
  return entries;
}

std::string find_library(
  const std::string & plugin, const std::string & package, std::vector<std::string> & available)
{
  std::vector<std::string> packages;
  if (!package.empty()) {
    packages.push_back(package);
  } else {
    for (const auto & [name, prefix] : ament_index_cpp::get_resources(k_resource_type)) {
      (void)prefix;
      packages.push_back(name);
    }
  }

  for (const auto & candidate_package : packages) {
    for (const auto & entry : get_package_components(candidate_package)) {
      if (entry.class_name == plugin) {
        return entry.library_path;
      }
      available.push_back(candidate_package + ": " + entry.class_name);
    }
  }
  return {};
}

}  // namespace

ComponentLoader::ComponentLoader(
  const std::string & plugin, const std::string & package, const std::string & library)
: plugin_(plugin)
{
  if (!library.empty()) {
    library_path_ = library;
  } else {
    std::vector<std::string> available;
    library_path_ = find_library(plugin, package, available);
    if (library_path_.empty()) {
      std::string message = "Component '" + plugin + "' not found";
      message += package.empty() ? " in any package" : " in package '" + package + "'";
      if (!available.empty()) {
        message += ". Available components:";
        for (const auto & name : available) {
          message += "\n  - " + name;
        }
      }
      throw std::runtime_error(message);
    }
  }

  try {
    class_loader_ = std::make_unique<class_loader::ClassLoader>(library_path_);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to load library '" + library_path_ + "': " + e.what());
  }

  const std::string factory_class = component_factory_class_name(plugin);
  const auto classes = class_loader_->getAvailableClasses<rclcpp_components::NodeFactory>();
  for (const auto & candidate : classes) {
    if (candidate == factory_class) {
      factory_ = class_loader_->createInstance<rclcpp_components::NodeFactory>(candidate);
      break;
    }
  }

  if (!factory_) {
    std::string message =
      "Library '" + library_path_ + "' does not register component '" + plugin + "'";
    if (!classes.empty()) {
      message += ". Registered classes:";
      for (const auto & name : classes) {
        message += "\n  - " + name;
      }
    }
    throw std::runtime_error(message);
  }
}

ComponentLoader::~ComponentLoader()
{
  // The factory was created from the library and has to go before the library is unloaded.
  factory_.reset();
  class_loader_.reset();
}

rclcpp_components::NodeInstanceWrapper ComponentLoader::create_node(
  const rclcpp::NodeOptions & options)
{
  return factory_->create_node_instance(options);
}

}  // namespace nebula::ros2_control
