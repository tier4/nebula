// Copyright 2025 TIER IV, Inc.
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

#include <cstdint>
#include <optional>
#include <variant>

namespace nebula::drivers::scan_cutter
{

using buffer_index_t = uint8_t;

template <typename T>
struct AllSame
{
  T value;
};

struct Different
{
};

using ChannelBufferState = std::variant<AllSame<buffer_index_t>, Different>;
using ChannelFovState = std::variant<AllSame<bool>, Different>;

struct TransitionActions
{
  /// Buffer to reset timestamp for, or std::nullopt if no timestamp reset needed.
  std::optional<buffer_index_t> reset_timestamp_buffer;
  /// Buffer to emit/publish, or std::nullopt if no emission needed.
  std::optional<buffer_index_t> emit_scan_buffer;
};

}  // namespace nebula::drivers::scan_cutter
