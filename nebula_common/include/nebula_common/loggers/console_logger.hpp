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

#include "nebula_common/loggers/logger.hpp"

#include <cstdio>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

namespace nebula::drivers::loggers
{

class ConsoleLogger : public Logger
{
public:
  explicit ConsoleLogger(std::string name) : name_(std::move(name)) {}

  void debug(const std::string & message) override { print_tagged(std::cout, "DEBUG", message); }
  void info(const std::string & message) override { print_tagged(std::cout, "INFO", message); }
  void warn(const std::string & message) override { print_tagged(std::cerr, "WARN", message); }
  void error(const std::string & message) override { print_tagged(std::cerr, "ERROR", message); }

  std::shared_ptr<Logger> child(const std::string & name) override
  {
    return std::make_shared<ConsoleLogger>(name_ + "." + name);
  }

private:
  std::string name_;

  void print_tagged(std::ostream & os, const std::string & severity, const std::string & message)
  {
    // In multithreaded logging, building the string first (... + ...) and then shifting to the
    // stream will ensure that no other logger outputs between string fragments
    os << ("[" + name_ + "][" + severity + "] " + message) << std::endl;
  }
};

}  // namespace nebula::drivers::loggers
