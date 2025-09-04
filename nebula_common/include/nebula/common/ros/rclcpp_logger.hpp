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

#include <nebula_common/loggers/logger.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>

namespace nebula::drivers::loggers
{

class RclcppLogger : public Logger
{
public:
  explicit RclcppLogger(const std::string & name) : underlying_logger_(rclcpp::get_logger(name)) {}
  explicit RclcppLogger(const rclcpp::Logger & underlying) : underlying_logger_(underlying) {}

  void debug(const std::string & message) override
  {
    RCLCPP_DEBUG_STREAM(underlying_logger_, message);
  }
  void info(const std::string & message) override
  {
    RCLCPP_INFO_STREAM(underlying_logger_, message);
  }
  void warn(const std::string & message) override
  {
    RCLCPP_WARN_STREAM(underlying_logger_, message);
  }
  void error(const std::string & message) override
  {
    RCLCPP_ERROR_STREAM(underlying_logger_, message);
  }

  std::shared_ptr<Logger> child(const std::string & name) override
  {
    return std::make_shared<RclcppLogger>(underlying_logger_.get_child(name));
  }

private:
  rclcpp::Logger underlying_logger_;
};

}  // namespace nebula::drivers::loggers
