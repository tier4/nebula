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

#include "params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>

#include <regex>
#include <stdexcept>

namespace nebula::cli
{

void Params::load_yaml(const std::string & path)
{
  const auto map = rclcpp::parameter_map_from_yaml_file(path);
  // Param files key their parameters by node name; "/**" matches every node.
  const auto it = map.find("/**");
  if (it == map.end()) {
    throw std::runtime_error("No '/**: ros__parameters' section in " + path);
  }
  for (const auto & p : it->second) {
    values_[p.get_name()] = p.value_to_string();
  }
}

void Params::set_assignment(const std::string & assignment)
{
  const auto pos = assignment.find(":=");
  if (pos == std::string::npos || pos == 0) {
    throw std::runtime_error("Expected name:=value, got '" + assignment + "'");
  }
  values_[assignment.substr(0, pos)] = assignment.substr(pos + 2);
}

std::string Params::get_string(const std::string & key, const std::string & fallback) const
{
  const auto it = values_.find(key);
  return it == values_.end() ? fallback : substitute(it->second);
}

double Params::get_double(const std::string & key, double fallback) const
{
  const auto it = values_.find(key);
  if (it == values_.end()) return fallback;
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    throw std::runtime_error("Parameter '" + key + "' is not a number: " + it->second);
  }
}

int64_t Params::get_int(const std::string & key, int64_t fallback) const
{
  const auto it = values_.find(key);
  if (it == values_.end()) return fallback;
  try {
    // stod so that "600.0" from YAML is accepted
    return static_cast<int64_t>(std::stod(it->second));
  } catch (const std::exception &) {
    throw std::runtime_error("Parameter '" + key + "' is not a number: " + it->second);
  }
}

std::string Params::substitute(const std::string & text) const
{
  static const std::regex re(R"(\$\(\s*(find-pkg-share|var)\s+([^)\s]+)\s*\))");
  std::string out;
  auto last = text.cbegin();
  for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it) {
    out.append(last, text.cbegin() + it->position());
    const std::string arg = (*it)[2];
    if ((*it)[1] == "var") {
      const auto v = values_.find(arg);
      if (v == values_.end()) throw std::runtime_error("Unknown $(var " + arg + ")");
      out += v->second;
    } else {
      out += ament_index_cpp::get_package_share_directory(arg);
    }
    last = text.cbegin() + it->position() + it->length();
  }
  out.append(last, text.cend());
  return out;
}

}  // namespace nebula::cli
