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

#include <nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>

namespace nebula::ros::loggers
{

class RosLogger : public nebula::drivers::loggers::Logger
{
public:
  RosLogger(rclcpp::Logger parent, const std::string & name) : logger_(parent.get_child(name)) {}

  explicit RosLogger(const std::string & name) : logger_(rclcpp::get_logger(name)) {}

  void debug(const std::string & message) override { RCLCPP_DEBUG_STREAM(logger_, message); }

  void info(const std::string & message) override { RCLCPP_INFO_STREAM(logger_, message); }

  void warn(const std::string & message) override { RCLCPP_WARN_STREAM(logger_, message); }

  void error(const std::string & message) override { RCLCPP_ERROR_STREAM(logger_, message); }

  std::shared_ptr<Logger> child(const std::string & name) override
  {
    return std::static_pointer_cast<Logger>(std::make_shared<RosLogger>(logger_, name));
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace nebula::ros::loggers
