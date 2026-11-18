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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nebula::cli
{

/// @brief One text line `name [=====>     ]  45%  1234/2740`. `total` 0 means unknown (no bar).
std::string render_progress_line(
  const std::string & name, size_t name_width, uint64_t done, uint64_t total, size_t bar_width = 30);

/// @brief One progress line per topic on stderr, redrawn in place from a background thread.
/// Without a terminal on stderr, nothing is drawn until stop() prints the final lines once.
class ProgressBars
{
public:
  ProgressBars(std::vector<std::string> names, std::vector<uint64_t> totals);
  ~ProgressBars();
  ProgressBars(const ProgressBars &) = delete;
  ProgressBars & operator=(const ProgressBars &) = delete;

  /// Thread safe
  void add(size_t index, uint64_t count = 1) { done_[index].fetch_add(count); }

  void start();
  /// Stops the redraw thread and draws the final state. Safe to call twice.
  void stop();

private:
  void draw(bool redraw);

  std::vector<std::string> names_;
  std::vector<uint64_t> totals_;
  std::unique_ptr<std::atomic<uint64_t>[]> done_;
  size_t name_width_{0};
  bool tty_;
  bool drawn_{false};
  bool running_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
};

}  // namespace nebula::cli
