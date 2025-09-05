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

#include "hesai_ros_decoder_test.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace nebula::test
{
class DecoderTest : public ::testing::TestWithParam<nebula::ros::HesaiRosDecoderTestParams>
{
protected:
  /// @brief Instantiates the Hesai driver node with the given test parameters
  void SetUp() override;

  /// @brief Destroys the Hesai driver node
  void TearDown() override;

  std::shared_ptr<nebula::ros::HesaiRosDecoderTest> hesai_driver_;
  std::shared_ptr<rclcpp::Logger> logger_;
};

}  // namespace nebula::test
