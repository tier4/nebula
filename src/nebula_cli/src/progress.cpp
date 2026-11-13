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

#include "progress.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <utility>

namespace nebula::cli
{

std::string render_progress_line(
  const std::string & name, size_t name_width, uint64_t done, uint64_t total, size_t bar_width)
{
  std::string line = name;
  line.resize(std::max(name_width, name.size()), ' ');
  char buf[96];
  if (total == 0) {
    std::snprintf(buf, sizeof(buf), "  %" PRIu64 " msgs", done);
    return line + buf;
  }
  done = std::min(done, total);
  const size_t filled = static_cast<size_t>(done * bar_width / total);
  std::string bar(bar_width, ' ');
  std::fill_n(bar.begin(), filled, '=');
  if (filled < bar_width) bar[filled] = '>';
  std::snprintf(
    buf, sizeof(buf), "  [%s] %3" PRIu64 "%%  %" PRIu64 "/%" PRIu64, bar.c_str(), done * 100 / total,
    done, total);
  return line + buf;
}

ProgressBars::ProgressBars(std::vector<std::string> names, std::vector<uint64_t> totals)
: names_(std::move(names)),
  totals_(std::move(totals)),
  done_(new std::atomic<uint64_t>[names_.size()]()),
  tty_(isatty(fileno(stderr)) != 0)
{
  for (const auto & n : names_) name_width_ = std::max(name_width_, n.size());
}

ProgressBars::~ProgressBars()
{
  stop();
}

void ProgressBars::draw(bool redraw)
{
  std::string out;
  if (redraw && drawn_) out += "\033[" + std::to_string(names_.size()) + "A";
  for (size_t i = 0; i < names_.size(); ++i) {
    if (tty_) out += "\r\033[2K";
    out += render_progress_line(names_[i], name_width_, done_[i].load(), totals_[i]) + "\n";
  }
  std::fputs(out.c_str(), stderr);
  std::fflush(stderr);
  drawn_ = true;
}

void ProgressBars::start()
{
  if (!tty_ || running_) return;
  running_ = true;
  thread_ = std::thread([this]() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (running_) {
      draw(true);
      cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
  });
}

void ProgressBars::stop()
{
  if (thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    cv_.notify_all();
    thread_.join();
    draw(true);
  } else if (!drawn_) {
    draw(false);
  }
}

}  // namespace nebula::cli
