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

#include <gtest/gtest.h>

using nebula::cli::render_progress_line;

TEST(ProgressLine, HalfDone)
{
  EXPECT_EQ(
    render_progress_line("/a", 4, 5, 10, 10), "/a    [=====>    ]  50%  5/10");
}

TEST(ProgressLine, FullIsClampedAndHasNoArrow)
{
  EXPECT_EQ(
    render_progress_line("/a", 2, 99, 10, 10), "/a  [==========] 100%  10/10");
}

TEST(ProgressLine, UnknownTotalShowsCountOnly)
{
  EXPECT_EQ(render_progress_line("/a", 2, 7, 0), "/a  7 msgs");
}
