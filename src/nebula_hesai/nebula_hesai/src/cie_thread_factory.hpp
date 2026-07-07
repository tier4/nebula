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

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast_cie_thread_configurator/cie_thread_configurator.hpp>
#endif

#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace nebula::ros
{

/// @brief Create a thread factory that registers spawned threads with the Agnocast CIE thread
/// configurator under the given name. Returns nullptr (callers spawn plain std::threads) when
/// Agnocast support is disabled.
/// @param thread_name Name announced to the configurator. Must be unique per process, so append
/// the sensor frame_id when one node can host multiple sensors.
inline std::function<std::thread(std::function<void()> &&)> make_cie_thread_factory(
  std::string thread_name)
{
#ifdef USE_AGNOCAST_ENABLED
  return [thread_name = std::move(thread_name)](std::function<void()> && thread_body) {
    return agnocast_cie_thread_configurator::spawn_non_ros2_thread(
      thread_name.c_str(), std::move(thread_body));
  };
#else
  (void)thread_name;
  return nullptr;
#endif
}

}  // namespace nebula::ros
