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

#ifndef NEBULA_ROS2_CONTROL__COMPONENT_LOADER_HPP_
#define NEBULA_ROS2_CONTROL__COMPONENT_LOADER_HPP_

#include <class_loader/class_loader.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>

#include <memory>
#include <string>

namespace nebula::ros2_control
{

/// Locates the shared library registering an rclcpp component (the same way
/// `ros2 component load` does) and keeps it loaded for as long as this object lives.
///
/// Node instances created by this loader must be destroyed before the loader.
class ComponentLoader
{
public:
  /// @param plugin Component class name as listed by `ros2 component types`.
  /// @param package Package to search in. Empty: search all packages.
  /// @param library Library path or file name to load directly, bypassing the ament index.
  /// @throws std::runtime_error if the component cannot be found or loaded.
  ComponentLoader(
    const std::string & plugin, const std::string & package, const std::string & library);

  ComponentLoader(const ComponentLoader &) = delete;
  ComponentLoader & operator=(const ComponentLoader &) = delete;
  ~ComponentLoader();

  /// Instantiate the component. Throws whatever the component's constructor throws.
  rclcpp_components::NodeInstanceWrapper create_node(const rclcpp::NodeOptions & options);

  [[nodiscard]] const std::string & library_path() const { return library_path_; }

private:
  std::string plugin_;
  std::string library_path_;
  std::unique_ptr<class_loader::ClassLoader> class_loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> factory_;
};

}  // namespace nebula::ros2_control

#endif  // NEBULA_ROS2_CONTROL__COMPONENT_LOADER_HPP_
