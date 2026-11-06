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

#pragma once

#include <map>
#include <string>

namespace nebula::cli
{

/// @brief Flat string key/value parameters, typed on read. Mirrors the keys of the ROS param files.
class Params
{
public:
  /// @brief Load `/**: ros__parameters` of a ROS param YAML. Throws on I/O or format errors.
  void load_yaml(const std::string & path);

  /// @brief Set a value from a `name:=value` string. Throws on missing `:=`.
  void set_assignment(const std::string & assignment);

  void set(const std::string & key, const std::string & value) { values_[key] = value; }
  bool has(const std::string & key) const { return values_.count(key) > 0; }

  /// @brief Typed getters. Throw std::runtime_error when the value cannot be parsed.
  std::string get_string(const std::string & key, const std::string & fallback) const;
  double get_double(const std::string & key, double fallback) const;
  int64_t get_int(const std::string & key, int64_t fallback) const;

  /// @brief Replace `$(find-pkg-share <pkg>)` and `$(var <key>)` in a string.
  std::string substitute(const std::string & text) const;

private:
  std::map<std::string, std::string> values_;
};

}  // namespace nebula::cli
