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

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <optional>
#include <string>
#include <vector>

namespace nebula::util
{

using nlohmann::json;

inline std::vector<std::string> to_json_path(const std::string & dot_delimited_path)
{
  std::vector<std::string> result;
  boost::split(result, dot_delimited_path, boost::is_any_of("."));
  return result;
}

inline json to_json_tree(const json & node, const std::vector<std::string> & path)
{
  json result = node;
  for (auto it = path.crbegin(); it != path.crend(); ++it) {
    auto current_key = *it;
    result = {{current_key, result}};
  }

  return result;
}

template <typename T>
bool update_if_exists(const json & node, const std::vector<std::string> & path, T & out_value)
{
  auto * current_node = &node;
  for (const auto & current_key : path) {
    if (!current_node->contains(current_key)) return false;
    current_node = &current_node->at(current_key);
  }

  out_value = current_node->template get<T>();
  return true;
}

template <typename T>
std::optional<T> get_if_exists(const json & node, const std::vector<std::string> & path)
{
  T result;
  if (update_if_exists(node, path, result)) {
    return result;
  }

  return {};
}

}  // namespace nebula::util
