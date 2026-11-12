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

#include <functional>
#include <thread>
#include <utility>

namespace nebula::util
{

/// Creates a thread from a given thread body. Allows callers to integrate worker threads with
/// external thread management frameworks (e.g. to control their scheduling policy).
using thread_factory_t = std::function<std::thread(std::function<void()> && thread_body)>;

/// The default thread factory: creates plain `std::thread`s.
struct StdThreadFactory
{
  std::thread operator()(std::function<void()> && thread_body) const
  {
    return std::thread(std::move(thread_body));
  }
};

}  // namespace nebula::util
