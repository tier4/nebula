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

#pragma once

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
namespace nebula::ros
{

rcl_interfaces::msg::ParameterDescriptor param_read_write();

rcl_interfaces::msg::ParameterDescriptor param_read_only();

rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type float_range(
  double start, double stop, double step);

rcl_interfaces::msg::ParameterDescriptor::_integer_range_type int_range(
  int start, int stop, int step);

/// @brief Get a parameter's value from a list of parameters, if that parameter is in the list.
/// @tparam T The parameter's expected value type
/// @param p A vector of parameters
/// @param name Target parameter name
/// @param value (out) Parameter value. Set if parameter is found, left untouched otherwise.
/// @return Whether the target name existed
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

}  // namespace nebula::ros
