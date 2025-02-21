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

#include "nebula_common/loggers/console_logger.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/tcp.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <utility>

namespace nebula::drivers
{
class PtcTest : public ::testing::TestWithParam<SensorModel>
{
protected:
  void SetUp() override { std::cout << "GetParam() = " << GetParam() << '\n'; }

  void TearDown() override {}

  static auto make_hw_interface(std::shared_ptr<connections::AbstractTcpSocket> tcp_socket)
  {
    auto model = GetParam();

    auto logger = std::make_shared<loggers::ConsoleLogger>("HwInterface");

    auto hw_interface = std::make_unique<HesaiHwInterface>(logger, std::move(tcp_socket));
    hw_interface->set_target_model(hw_interface->nebula_model_to_hesai_model_no(model));
    return hw_interface;
  }
};

}  // namespace nebula::drivers
